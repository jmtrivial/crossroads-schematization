from shapely.geometry import Point, LineString, Polygon, LinearRing, MultiPolygon
import numpy as np
import shapely.ops
from numpy import linalg
import geopandas
from enum import Enum
import math

import copy
import osmnx
from more_itertools import locate

from .. import utils as u
from .. import processing as p

class TrafficIsland:

    class Geometry(Enum):
        point = 0
        lines = 1
        polygon = 2

    def __init__(self, island_id, edgelist, osm_input, cr_input, crossings, distance_kerb_footway = 0.5, threshold_small_island = 30):
        self.island_id = island_id
        self.edgelist = [list(map(int, x.split(";"))) for x in edgelist]
        self.osm_input = osm_input
        self.cr_input = cr_input
        self.crossings = crossings

        self.significant_ratio = 2
        self.threshold_small_island = threshold_small_island
        self.distance_kerb_footway = distance_kerb_footway

        self.build_polygon()
        self.build_inner_polygon()


    def build_polygon(self):

        ledges = copy.deepcopy(self.edgelist)
        if len(self.edgelist) == 0:
            self.polygon = []
            return

        self.polygon = ledges.pop()
        
        reverse = False
        while len(ledges) != 0:
            # find next element in ledges
            found = False
            for i, e in enumerate(ledges):
                if e[0] == self.polygon[-1]:
                    self.polygon += e[1:]
                    ledges.pop(i)
                    found = True
                    break
                if e[-1] == self.polygon[-1]:
                    self.polygon += e[::-1][1:]
                    ledges.pop(i)
                    found = True
                    break
            if not found:
                if reverse:
                    print("Error: cannot merge all edges in a single traffic island")
                    return
                else:
                    reverse = True
                    self.polygon = self.polygon[::-1]

        # if the polygon is not closed, a part is missing in the original data (but available in OSM)
        if self.polygon[0] != self.polygon[-1]:
            self.polygon = p.Expander.close_polygon(self.osm_input, self.polygon)

    def build_inner_polygon(self):
        ring = Polygon(self.get_linearring())

        buffered = u.Utils.get_buffered_by_osm(self.polygon, self.osm_input, self.distance_kerb_footway)

        self.inner_polygon = ring.buffer(0).difference(buffered)

    def extends_polygon_with_osm(self):
        next = p.Expander.find_next_edge_simple(self.osm_input, self.polygon[-2], self.polygon[-1])
        while next != None:
            self.polygon.append(next)
            next = p.Expander.find_next_edge_simple(self.osm_input, self.polygon[-2], self.polygon[-1])


    def get_linearring(self):
        points = [self.osm_input.nodes[x] for x in self.polygon]
        return LinearRing([(p["x"], p["y"]) for p in points])


    def compute_center_and_radius(self, crossings):
        local_crossings = [self.crossings[c] for c in crossings if c in self.polygon]
        if len(local_crossings) != 0:
            # use extremity of the crossing
            l = [c.get_location_on_island(self.island_id) for c in local_crossings]
            self.is_reachable = True
        else:
            l = [self.osm_input.nodes[x] for x in self.polygon]
            l = [(c["x"], c["y"]) for c in l]
            self.is_reachable = False
        xs = [c[0] for c in l]
        ys = [c[1] for c in l]
        self.center = (sum(xs) / len(xs), sum(ys) / len(ys))
        ds = [osmnx.distance.euclidean_dist_vec(c[0], c[1], self.center[0], self.center[1]) for c in l]
        self.radius = sum(ds) / len(ds)


    def get_border_sections(self, crossings):
        c_in_poly = [i for i, x in enumerate(self.polygon) if x in crossings.keys()]
        if len(c_in_poly) == 0:
            print("Error: cannot have an island without crossing at this stage")
            return None
        polyshift = self.polygon[c_in_poly[0]:] + self.polygon[:c_in_poly[0]]
        # make it as a loop
        polyshift.append(polyshift[0])

        # build sections along the polygon starting and ending by a crossing
        sections = []
        sections.append([])
        for p in polyshift:
            sections[-1].append(p)
            if p in list(crossings.keys()):
                sections.append([p])

        # only keep sections with one non crossing node
        sections = [s for s in sections if len(s) > 2]

        return sections


    def is_sidewalk_node(self, i, j, k):
        if not i in self.osm_input or not j in self.osm_input or not k in self.osm_input:
            return False
        if j not in self.osm_input[i] or k not in self.osm_input[j]:
            return True
        tags1 = u.Utils.get_initial_edge_tags(self.cr_input, j, i, True)
        tags2 = u.Utils.get_initial_edge_tags(self.cr_input, k, j, True)
        if tags1 == None or tags2 == None:
            return False
        return (tags1["left_sidewalk"] != "" or tags1["right_sidewalk"] != "") and (tags2["left_sidewalk"] != "" or tags2["right_sidewalk"] != "")

    def max_distance_to_center(self, section):
        sidewalk_section = section
        return max([osmnx.distance.euclidean_dist_vec(self.osm_input.nodes[c]["x"], 
                                                      self.osm_input.nodes[c]["y"],
                                                      self.center[0], self.center[1]) for c in sidewalk_section])


    def adjust_extremity(self, center, extremity, shift):
        v = u.Utils.vector(center, extremity)
        d = norm = linalg.norm(np.array(v), 2)
        if d < shift:
            return None
        else:
            return [cc + vv / d * (d - shift) for cc, vv in zip(center, v)]


    def build_subsection_orientations(self, section, length):
        def orient_edge(e, center):
            lc1 = u.Utils.edge_length(center, e[0])
            lc2 = u.Utils.edge_length(center, e[1])
            return e if lc1 < lc2 else (e[1], e[0])
        # only keep edges if their length matches with the given one (avoid virtual edges at the end of the branches)
        edges = [e for e in zip(section, section[1:]) if math.fabs(u.Utils.edge_length(e[0], e[1]) - length) < 1e-5]

        # orient the edges such that they are going away from the center
        oedges = [orient_edge(e, self.center) for e in edges]

        # normalize vectors
        return [u.Utils.normalized_vector(e[0], e[1]) for e in oedges]


    def get_straight_island_direction(self, polylines):
        # linearize the two polylines
        lz = p.Linearization(length=50, initial_step=0.5, exponential_coef=1.2)
        ll1 = lz.process(LineString(polylines[0]))
        ll2 = lz.process(LineString(polylines[1]))

        # use their directions to get a global direction for the island
        n1 = u.Utils.normalized_vector(ll1.coords[0], ll1.coords[1])
        n2 = u.Utils.normalized_vector(ll2.coords[0], ll2.coords[1])
        vectors = [n1, n2]
        final_vector = (sum([v[0] for v in vectors]) / len(vectors), sum([v[1] for v in vectors]) / len(vectors))

        # build a long edge according to this direction
        length = 200
        return Point(self.center[0] + final_vector[0] * length, self.center[1] + final_vector[1] * length)


    def build_polylines_from_section(self, section):

        edges = [e for e in zip(section, section[1:]) if e[0] != e[1]]
        outside = list(locate(edges, lambda e: not u.Utils.edge_in_osm(e[0], e[1], self.osm_input)))

        if len(outside) != 0:
            side1 = section[0:outside[0] + 1]
            side2 = section[outside[-1] + 1:]
            side2.reverse()
            return u.Utils.pathid_to_pathcoords(side1, self.osm_input), u.Utils.pathid_to_pathcoords(side2, self.osm_input)
        else:
            # use length to split
            path = LineString(u.Utils.pathid_to_pathcoords(section, self.osm_input))
            step = 5
            resampled_polyline = [path.interpolate(float(x) / step) for x in range(0, int(path.length * step))]

            distances = [0] + [u.Utils.edge_length(a, b) for a, b in zip(resampled_polyline, resampled_polyline[1:])]
            cumuld_dists = np.cumsum(distances)
            mid = cumuld_dists[-1] / 2
            side1 = [(s.x, s.y) for s, d in zip(resampled_polyline, cumuld_dists) if d < mid]
            side2 = [(s.x, s.y) for s, d in zip(resampled_polyline, cumuld_dists) if d >= mid]
            side2.reverse()
            return side1, side2



    def get_edge_extremity_from_section(self, section, inner_region):
        # build left and right polylines
        polylines = self.build_polylines_from_section(section)

        # compute a straight island
        other_in_edge = self.get_straight_island_direction(polylines)

        if other_in_edge is None:
            return None

        # TODO DEBUG
        # import matplotlib.pyplot as plt
        # for p in polylines:
        #     plt.plot([c[0] for c in p], [c[1] for c in p], 'ok')
        # plt.plot([self.center[0], other_in_edge.x], [self.center[1], other_in_edge.y])
        # plt.show()


        # build a buffered version of the initial polyline, and compute the intersection.
        buffered = u.Utils.get_buffered_by_osm(section, self.osm_input, self.distance_kerb_footway)
        if buffered.is_empty:
            print("Note: Buffered section is empty")
            return None
        elif buffered.intersects(Point(self.center)):
            print("Note: center of island in the buffered section")
            return None
        else:
            # compute intersection
            intersection = buffered.boundary.union(inner_region.boundary).intersection(LineString([self.center, other_in_edge]))
            if intersection.is_empty:
                print("Note: no intersection between the possible edge and the buffered section")
                return None
            else:
                nearest = shapely.ops.nearest_points(Point(self.center[0], self.center[1]), intersection)

                # move it a bit in the inner direction
                extremity = self.adjust_extremity(self.center, nearest[1], self.radius / 4)
                # if this move is not possible, return none
                if extremity is None:
                    return None

                # check if this new point is valid
                edge = LineString([self.center, extremity])
                return (extremity[0], extremity[1])


    # return true if one of the edges of the current island is not in OSM data (i.e. it's part of the border of the inner region,
    # i.e. the island is a medial axis of a branch)
    def is_branch_medial_axis(self):
        for n1, n2 in zip(self.polygon, self.polygon[1:] + [self.polygon[0]]):
            if n1 != n2 and not u.Utils.edge_in_osm(n1, n2, self.osm_input):
                return True

        return False

    
    def is_linear_island_candidate(self, border_sections):
        return len(border_sections) <= 2 or self.is_branch_medial_axis()


    def is_small_island(self):
        return self.inner_polygon.area < self.threshold_small_island


    def is_linear_island_wrt_extremities(self):

        if len(self.extremities) == 0:
            return False

        # maximum distance for a point to be considered close to the middle 
        max_distance = math.sqrt(self.threshold_small_island)

        # compute middle
        edges = [LineString([self.center, e]) for e in self.extremities]
        middle = edges[0]
        for e in edges[1:]:
            middle = middle.union(e)

        distance = 0.0

        for c in self.crossings:
            if c in self.polygon:
                d = middle.distance(Point(self.crossings[c].get_location_on_island(self.island_id)))
                if d > distance:
                    distance = d

        return distance <= max_distance


    def compute_generalization(self, crossings, inner_region):

        # compute crossing's center
        self.compute_center_and_radius(crossings)


        if self.is_reachable:
            

            if self.is_small_island():
                self.generalization = TrafficIsland.Geometry.point
            else:
                border_sections = self.get_border_sections(crossings)
                if self.is_linear_island_candidate(border_sections):
                    sections = [s for s in border_sections if self.max_distance_to_center(s) > self.radius * self.significant_ratio]
                    self.extremities = [self.get_edge_extremity_from_section(s, inner_region) for s in sections]
                    self.extremities = [e for e in self.extremities if not e is None]

                    if self.is_linear_island_wrt_extremities():
                        self.generalization = TrafficIsland.Geometry.lines
                    else:
                        self.generalization = TrafficIsland.Geometry.polygon
                else:
                    self.generalization = TrafficIsland.Geometry.polygon
        else:
            if self.is_small_island():
                self.generalization = TrafficIsland.Geometry.point
            else:
                self.generalization = TrafficIsland.Geometry.polygon


    def getGeometry(self):
        if self.generalization == TrafficIsland.Geometry.point:
            return [Point(self.center)]
        elif self.generalization == TrafficIsland.Geometry.lines:
            return [LineString([self.center, e]) for e in self.extremities]
        else:
            # TODO: simplifier la forme
            return [Polygon(self.inner_polygon)]

    def toGDFTrafficIslands(traffic_islands, only_reachable = True):
        d = {'type': [], 'osm_id': [], 'geometry': []}

        for t in traffic_islands:
            if t.is_reachable or not only_reachable:
                geom = t.getGeometry()
                for g in geom:
                    d["type"].append("traffic_island")
                    d["osm_id"].append(";".join(map(str, t.polygon)))
                    d["geometry"].append(g)

        return geopandas.GeoDataFrame(d, crs=2154)
