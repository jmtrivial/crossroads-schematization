from shapely.geometry import Point, LineString, MultiLineString, LinearRing, Polygon

import numpy as np
from numpy import linalg
import copy
import math
import shapely.ops
import osmnx
from more_itertools import locate
import geopandas
from enum import Enum

from . import utils as u
from . import processing as p


class SimpleWay:
    
    # parameters (and attributes)
    # - n1 is the interior node, n2 is the exterior node  (centripetal orientation)
    # - edge_tags: tags from crdesc
    # - same_osm_orientation: boolean (is the OSM orientation similar)
    def __init__(self, n1, n2, edge_tags, same_osm_orientation):
        self.n1 = n1
        self.n2 = n2
        self.same_osm_orientation = same_osm_orientation
        self.edge_tags = edge_tags


    def has_sidewalk(self):
        return SimpleWay.is_number(self.edge_tags["left_sidewalk"]) or SimpleWay.is_number(self.edge_tags["right_sidewalk"])


    def is_number(value):
        if isinstance(value, str):
            return value != ""
        else:
            return not math.isnan(value)

    def get_sidewalk_id(self):
        if SimpleWay.is_number(self.edge_tags["left_sidewalk"]):
            return self.edge_tags["left_sidewalk"]
        elif SimpleWay.is_number(self.edge_tags["right_sidewalk"]):
            return self.edge_tags["right_sidewalk"]
        else:
            return ""

    def has_sidewalks_both_sides(self):
        return SimpleWay.is_number(self.edge_tags["left_sidewalk"]) and SimpleWay.is_number(self.edge_tags["right_sidewalk"])


    def get_initial_edge_id(self):
        if self.same_osm_orientation:
            return self.get_edge_id()
        else:
            return self.get_edge_id_reverse()


    def get_edge_id(self):
        return str(self.n1) + ";" + str(self.n2)


    def get_edge_id_reverse(self):
        return str(self.n2) + ";" + str(self.n1)


class StraightWay(SimpleWay):
    
    # parameters (and attributes)
    # - a simpleway (defined by nodes n1, n2)
    # - polybranch: an extended path from n1, n2
    def __init__(self, sw, polybranch, G):
        super().__init__(sw.n1, sw.n2, sw.edge_tags, sw.same_osm_orientation)
        self.polybranch = polybranch
        self.edge = None
        self.array = None
        self.G = G
        self.lz = p.Linearization()
        self.maximal_removal = 20 # meters


    def build_from_simpleway(sw, G, left_first):
        result = StraightWay(sw, p.Expander.extend_branch(G, sw.n1, sw.n2, left_first), G) 
        result.compute_linear_edge(G)
        return result


    def compute_linear_edge(self, G):
        self.consolidated_polybranch = p.Expander.remove_non_straight_parts(G, self.polybranch, self.maximal_removal)
        self.edge = self.lz.process(p.Expander.convert_to_linestring(G, self.consolidated_polybranch))
        self.array = np.asarray(self.edge.coords)


    def sum_length(self, edges, G):
        return sum([LineString([(G.nodes[e[0]]["x"], G.nodes[e[0]]["y"]),
                                 (G.nodes[e[1]]["x"], G.nodes[e[1]]["y"])]).length for e in edges])


    def adjust_by_coherency(self, sw, G):
        list1 = list(zip(sw.polybranch, sw.polybranch[1:]))
        list2 = list(zip(self.polybranch, self.polybranch[1:]))
        both = list(set(list1).intersection(list2))
        if len(both) != 0:
            common = [ e for e in list1 if e in both]
            length2 = self.sum_length(list2, G)
            lengthc = self.sum_length(common, G)
            # replace only if this common part is a significative part of the polybranches
            if lengthc > 0.8 * length2:
                # TODO: not perfect if the common elements are not in a continuous section, but should not append
                sw.polybranch = [e[0] for e in common] + [common[-1][1]]
                self.polybranch = sw.polybranch


    def __str__(self):
        return str(((self.n1, self.n2), self.edge, self.same_osm_orientation))


    def point(self, i):
        return Point(self.array[i])


    def build_middle_line(sw1, sw2):
        e1_1 = p.Linearization.project_on_line(Point(sw1.array[0]), sw2.edge)
        e1_2 = p.Linearization.project_on_line(Point(sw1.array[1]), sw2.edge)
        e2_1 = p.Linearization.project_on_line(Point(sw2.array[0]), sw1.edge)
        e2_2 = p.Linearization.project_on_line(Point(sw2.array[1]), sw1.edge)

        # TODO DEBUG
        # import matplotlib.pyplot as plt
        # plt.plot([c[0] for c in sw1.array], [c[1] for c in sw1.array])
        # plt.plot([c[0] for c in sw2.array], [c[1] for c in sw2.array])

        # plt.plot([sw1.osm_input.nodes[sw1.n1]["x"], sw1.osm_input.nodes[sw1.n2]["x"]], [sw1.osm_input.nodes[sw1.n1]["y"], sw1.osm_input.nodes[sw1.n2]["y"]])
        # plt.plot([sw2.osm_input.nodes[sw2.n1]["x"], sw2.osm_input.nodes[sw2.n2]["x"]], [sw2.osm_input.nodes[sw2.n1]["y"], sw2.osm_input.nodes[sw2.n2]["y"]])
        # plt.show()
                
        return LineString([LineString([e1_1, e2_1]).centroid, LineString([e1_2, e2_2]).centroid])


    def get_projection_on_polybranch(self, point):
        line = [(self.G.nodes[x]["x"], self.G.nodes[x]["y"]) for x in self.polybranch]
        nearest = shapely.ops.nearest_points(LineString(line), Point(point))
        pt = nearest[0]

        edges = []
        for e1, e2 in zip(self.polybranch, self.polybranch[1:]):
            if u.Utils.is_in_edge(pt, self.G.nodes[e1], self.G.nodes[e2]):
                edges.append((e1, e2))

        if len(edges) == 0:
            return pt, None
        elif len(edges) == 1:
            return pt, edges[0]
        else:
            # find the edge with the largest estimated width
            ewidths = [(e, u.Utils.evaluate_width_way(self.G[e[0]][e[1]][0])) for e in edges]
            return pt, max(ewidths, key=lambda x: x[1])[0]
        

class StraightSidewalk:

    def __init__(self, edge, straightway, side):
        self.edge = edge
        self.straightway = straightway
        self.description = straightway.edge_tags
        self.same_orientation = straightway.same_osm_orientation
        self.side = side


    def get_polybranch(self):
        return self.straightway.polybranch

    def __str__(self):
        return str(self.edge) + "; side: " + str(self.side) + "; same orientation: " + str(self.same_orientation)


    def update_first_node(self, coords):
        self.edge = LineString([coords, self.edge.coords[1]])


    def extends_start(self, length = 1):
        edgecoords = np.asarray(self.edge.coords)
        start = edgecoords[0]
        end = edgecoords[1]
        v = [end[0] - start[0], end[1] - start[1]]
        v = v / linalg.norm(v)
        self.edge = LineString([(self.edge.coords[0][0] - v[0] * length, self.edge.coords[0][1] - v[1] * length), self.edge.coords[1]])


    def sidewalk_id(self):
        s = self.side
        if not self.same_orientation:
            s = "left" if s == "right" else "right"
        return int(self.description[s + "_sidewalk"])


    def extends(self, length = 200):
        edgecoords = np.asarray(self.edge.coords)
        x = [a[0] for a in edgecoords]
        y = [a[1] for a in edgecoords]
        center = Point(sum(x) / len(x), sum(y) / len(y))
        start = edgecoords[0]
        end = edgecoords[1]
        v = [center.x - start[0], center.y - start[1]]
        v = v / linalg.norm(v)
        return LineString([start - v * length, end + v * length])


    # compute the intersection between the two straight sidewalk lines (considering it as infinite lines)
    def get_intersection(self, sw):
        # extend both LineString
        l1 = self.extends()
        l2 = sw.extends()

        # compute intersection between them
        return l1.intersection(l2)


    def getOSMIds(self):
        return ";".join(self.description["osm_node_ids"])


class TurningSidewalk:

    class TurnShape(Enum):
        BEVELED = 0
        STRAIGHT_ANGLE = 1
        ADJUSTED_ANGLE = 2

        def __str__(self):
                return self.name

        @staticmethod
        def from_string(s):
            try:
                return TurnShape[s]
            except KeyError:
                raise ValueError()
                
    class Point:
        def __init__(self, coord, curvPos = None):
            if isinstance(coord, Point):
                self.coord = (coord.x, coord.y)
            else:
                self.coord = coord
            self.curvPos = curvPos
        
        def set_curvilign_position(self, curvPos):
            self.curvPos = curvPos

    class FlexiblePoint(Point):
        def __init__(self, coord, curvPos = None):
            super().__init__(coord, curvPos)
            self.flexible = True

    class FixedPoint(Point):
        def __init__(self, coord, curvPos = None):
            super().__init__(coord, curvPos)
            self.flexible = False

    class CrossingPoint(FixedPoint):
        pass

        

    def __init__(self, id, str_sidewalks, crossings, 
                 osm_input, cr_input,
                 distance_kerb_footway, ignore_crossings_for_sidewalks,
                 turn_shape = TurnShape.ADJUSTED_ANGLE):
        self.id = id

        self.distance_kerb_footway = distance_kerb_footway
        self.turn_shape = turn_shape
        self.str_sidewalks = str_sidewalks

        self.crossings = crossings
        
        self.osm_input = osm_input
        self.cr_input = cr_input

        self.epsilon_for_merging = 2

        self.build_initial_turn()

        if not ignore_crossings_for_sidewalks:
            self.add_crossings()

        self.adjust_flexible_points()


    def build_initial_turn_basic(self):
        self.way = [TurningSidewalk.FixedPoint(c) for c in self.str_sidewalks[0].edge.coords][::-1] + [TurningSidewalk.FixedPoint(c) for c in self.str_sidewalks[1].edge.coords]


    def add_intersection_point_in_turn(self):
        if TurningSidewalk.is_before_end_of_edge(self.str_sidewalks[0].edge.coords, self.intersection) and \
            TurningSidewalk.is_before_end_of_edge(self.str_sidewalks[1].edge.coords, self.intersection):
            # if the intersection point is within the two end points
            
            if not TurningSidewalk.is_before_begin_of_edge(self.str_sidewalks[0].edge.coords, self.intersection):
                # if the intersection point is within the first edge
                self.way[1] = TurningSidewalk.FixedPoint(self.intersection)
                # if it is also inside the second edge, we remove the supplementary point
                if not TurningSidewalk.is_before_begin_of_edge(self.str_sidewalks[1].edge.coords, self.intersection):
                    del self.way[2]
            else:
                if not TurningSidewalk.is_before_begin_of_edge(self.str_sidewalks[1].edge.coords, self.intersection):
                    # if the intersection point is within the second edge
                    self.way[2] = TurningSidewalk.FixedPoint(self.intersection)
                else:
                    if self.turn_shape == TurningSidewalk.TurnShape.STRAIGHT_ANGLE:
                        # we add a fixed point
                        self.way.insert(2, TurningSidewalk.FixedPoint(self.intersection))
                    else:
                        # we add a flexible point
                        self.way.insert(2, TurningSidewalk.FlexiblePoint(self.intersection))


    def is_sidewalk_edge(self, node1, node2):
        tags = u.Utils.get_initial_edge_tags(self.cr_input, node1, node2, True)
        return tags != None and (tags["left_sidewalk"] == str(self.id) or tags["right_sidewalk"] == str(self.id))

    def find_next_point_on_original_path(self, path):
        last = path[-1]
        previous = path[-2]

        # for all neighbour of the last point
        for nb in self.osm_input[last]:
            if nb != previous and self.is_sidewalk_edge(nb, last):
                return nb
        
        print("Error: cannot found a point along the sidewalk")
        return None


    def compute_original_path(self):
        # find all edges of the original graph that are part of the sidewalk
        polybranch1 = self.str_sidewalks[0].get_polybranch()
        polybranch2 = self.str_sidewalks[1].get_polybranch()
        
        # initialize the final path with the first polybranch
        self.original_path = polybranch1[::-1]
        while self.original_path[-1] != polybranch2[0]:
            self.original_path.append(self.find_next_point_on_original_path(self.original_path))
            if self.original_path[-1] == None:
                # error: no continuity (should not be possible)
                self.original_path = None
                return
        
        # add the final part
        self.original_path += polybranch2[1:]
        self.original_path_linestring = [(self.osm_input.nodes[x]["x"], self.osm_input.nodes[x]["y"]) for x in self.original_path]


    def project_on_original_path(self, point):
        if isinstance(point, TurningSidewalk.Point):
            p = point.coord
        else:
            p = point
        nearest = shapely.ops.nearest_points(LineString(self.original_path_linestring), Point(p))
        return nearest[0]


    def estimate_curvilign_location_by_projection(self, point):
        proj = self.project_on_original_path(point)

        location = 0.0

        for x, y in zip(self.original_path_linestring, self.original_path_linestring[1:]):
            if u.Utils.is_in_edge(proj, x, y):
                location += u.Utils.edge_length(x, proj)
                break
            else:
                location += u.Utils.edge_length(x, y)

        return location


    def compute_curvilign_locations(self):
        for idp, current in enumerate(self.way):
            self.way[idp].set_curvilign_position(self.estimate_curvilign_location_by_projection(current))


    def build_initial_turn(self):
        # create turn with the 4 initial points
        self.build_initial_turn_basic()

        if self.turn_shape != TurningSidewalk.TurnShape.BEVELED:
            # compute middle point
            self.intersection = self.get_intersection()

            # create the initial turn
            if not self.intersection.is_empty:
                # if there is an intersection point
                self.add_intersection_point_in_turn()


        # compute the corresponding path in the OSM graph
        self.compute_original_path()

        # compute curvilign locations
        self.compute_curvilign_locations()

    def add_crossings(self):
        # for each sidewalk point
        for c in self.crossings:
            # identify its curvilign coordinate in the OSM path
            curvPos = self.estimate_curvilign_location_by_projection(c.get_location())
            # get location on the sidewalk
            location = c.get_location_on_sidewalk(self.id)
            # create the crossing point
            p = TurningSidewalk.CrossingPoint(location, curvPos)

            # find the good location along the way
            cid = 0
            while self.way[cid].curvPos < curvPos:
                cid += 1
                if cid >= len(self.way):
                    break
            
            if cid != 0 and abs(curvPos - self.way[cid - 1].curvPos) < self.epsilon_for_merging:
                # replace the existing node by the crossing
                self.way[cid - 1] = p
            elif cid < len(self.way) and abs(curvPos - self.way[cid].curvPos) < self.epsilon_for_merging:
                # replace the existing node by the crossing
                self.way[cid] = p
            else:
                # add it to the sidewalk
                self.way.insert(cid, p)


    def adjust_flexible_points(self):
        buffered_osm = u.Utils.get_buffered_osm(self.osm_input, self.distance_kerb_footway)

        for pred, point, next in zip(self.way, self.way[1:], self.way[2:]):
            if point.flexible:
                middle_sw = point.coord
                
                middle_bevel = Point([(x + y) / 2 for x, y in zip(pred.coord, next.coord)])
                # first check for basic intersection
                line = LineString([middle_sw, middle_bevel])

                if buffered_osm.intersects(line):
                    # build a more complex turn
                    edge = LineString((middle_bevel, middle_sw))
                    
                    elements = buffered_osm.boundary.intersection(edge)
                    if not elements.is_empty:
                        nearest = shapely.ops.nearest_points(middle_bevel, elements)
                        point.coord = (nearest[1].x, nearest[1].y)


    def branch_ids(self):
        return [x.description["id"] for x in self.str_sidewalks]
    

    def sidewalk_id(self):
        return self.str_sidewalks[0].sidewalk_id()


    def get_intersection(self):
        return self.str_sidewalks[0].get_intersection(self.str_sidewalks[1])


    def is_before_end_of_edge(edge, node):
        return u.Utils.norm_and_dot(u.Utils.vector(edge[0], edge[1]), u.Utils.vector(edge[1], node)) < 0


    def is_before_begin_of_edge(edge, node):
        return u.Utils.norm_and_dot(u.Utils.vector(edge[0], edge[1]), u.Utils.vector(edge[0], node)) < 0


    def as_array(self):
        return np.asarray([x.coord for x in self.way])


    def buffer(self, size):
        return self.way.buffer(size)


    def getGeometry(self):
        return LineString([x.coord for x in self.way])


    def getOSMIds(self):
        return ";".join([x.getOSMIds() for x in self.str_sidewalks])

    def toGDFSidewalks(sidewalks):
        d = {'type': [], 'osm_id': [], 'geometry': []}

        for s in sidewalks:
            d["type"].append("sidewalk")
            d["osm_id"].append(s.getOSMIds())
            d["geometry"].append(s.getGeometry())

        return geopandas.GeoDataFrame(d, crs=2154, geometry="geometry")


class TrafficIsland:

    def __init__(self, island_id, edgelist, osm_input, cr_input, crossings):
        self.island_id = island_id
        self.edgelist = [list(map(int, x.split(";"))) for x in edgelist]
        self.osm_input = osm_input
        self.cr_input = cr_input
        self.crossings = crossings

        self.build_polygon()

        self.significant_ratio = 2

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
            self.extends_polygon_with_osm()
            self.polygon = self.polygon[::-1]
            self.extends_polygon_with_osm()
            self.polygon = self.polygon[::-1]


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
        #if len(section) == 2:
        #    sidewalk_section = section
        #else:
        #    sidewalk_section = [section[0]] + [j for i, j, k in zip(section, section[1:], section[2:]) if self.is_sidewalk_node(i, j, k)] + [section[-1]]
        # TODO: revoir ici, peut-être la moyenne des distances
        # et garder cette valeur pour ensuite choisir une longueur
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
        # LineString([Point(self.osm_input.nodes[n]["x"], self.osm_input.nodes[n]["y"]) for n in section])  

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
        buffered = u.Utils.get_buffered_by_osm(section, self.osm_input)
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

    def compute_edges(self, crossings, inner_region):
        border_sections = self.get_border_sections(crossings)

        # only compute edges if it's a linear island
        if len(border_sections) <= 2 or self.is_branch_medial_axis():
            sections = [s for s in border_sections if self.max_distance_to_center(s) > self.radius * self.significant_ratio]
            self.extremities = [self.get_edge_extremity_from_section(s, inner_region) for s in sections]
            self.extremities = [e for e in self.extremities if not e is None]
        else:
            self.extremities = []
        

    def compute_generalization(self, crossings, inner_region):

        # compute crossing's center
        self.compute_center_and_radius(crossings)

        # TODO: if it's a a large region, build a polygon (./example-pdf.sh 13)

        if self.is_reachable:
            #compute supplementary edges if some of points of the polygons are far from the center
            self.compute_edges(crossings, inner_region)
        else:
            self.extremities = []


    def getGeometry(self):
        # TODO: alternative geometry in case of a polygon (cf compute_generalization)
        if len(self.extremities) == 0:
            return [Point(self.center)]
        else:
            return [LineString([self.center, e]) for e in self.extremities]


    def toGDFTrafficIslands(traffic_islands, only_reachable = True):
        d = {'type': [], 'osm_id': [], 'geometry': []}

        for t in traffic_islands:
            if t.is_reachable or not only_reachable:
                geom = t.getGeometry()
                # TODO: add supplementary attribute to distinguish between a polygon and a more generalized island (cf compute_generalization)
                for g in geom:
                    d["type"].append("traffic_island")
                    d["osm_id"].append(";".join(map(str, t.polygon)))
                    d["geometry"].append(g)

        return geopandas.GeoDataFrame(d, crs=2154)


class Crossing:

    # The crossing will be oriented such that it goes from the sidewalk
    # with the smallest ID to the sidewalk with the largest one.
    # If there is an island in one side, this sidewalk comes first.
    # If there are two islands, they are ordered in increasing ID order.

    def __init__(self, node_id, osm_input, cr_input, osm_input_oriented, distance_kerb_footway):

        self.node_id = node_id

        self.osm_input = osm_input
        self.cr_input = cr_input
        self.osm_input_oriented = osm_input_oriented
        self.distance_kerb_footway = distance_kerb_footway

        self.island_width = 0.50 # cm

        self.compute_location()

        self.compute_way_orientations()

        if len(self.roadway_nodes) > 0:

            self.compute_footway_orientations()

            self.flip_orientation_if_required()

            self.compute_final_orientation()

            # split adjacent ways in two groups
            self.compute_adjacent_ways_ditribution()


        self.compute_crossing_profile()


    def get_adjacent_border_from_way(self, tags, side):
        sidewalk_key = side + "_sidewalk"
        island_key = side + "_island"

        if sidewalk_key in tags and isinstance(tags[sidewalk_key], str) and tags[sidewalk_key] != "":
            return tags[sidewalk_key], "sidewalk"
        elif island_key in tags and isinstance(tags[island_key], str) and tags[island_key] != "":
            return tags[island_key], "island"
        else:
            return None, ""

    def get_adjacent_border(self, crossing_orientation):
        # for each adjacent way, compute the angle with the crossing orientation
        orientations = [math.atan2(-x[1], x[0]) for x in self.build_vectors(self.roadway_nodes)]
        angles = [x - crossing_orientation for x in orientations]
        positive_angles = [x if x > 0 else x + 2 * math.pi if x > - 2 * math.pi else x + 4 * math.pi for x in angles]
        # find next way by orientation
        idx = positive_angles.index(min(positive_angles))
        next_node = self.roadway_nodes[idx]
        
        # get tags
        tags = u.Utils.get_initial_edge_tags(self.cr_input, self.node_id, next_node, True)

        if tags is None:
            # if tags are not defined, the crossing is located in border of the input segmentation, we choose the 
            # opposite direction to get sidewalk information

            idx = positive_angles.index(max(positive_angles))
            next_node = self.roadway_nodes[idx]

            tags = u.Utils.get_initial_edge_tags(self.cr_input, self.node_id, next_node, True)

            side = "left" if str(next_node) != str(tags["osm_node_ids"][0]) else "right"

            return self.get_adjacent_border_from_way(tags, side)
        else:
            
            # identify the adjacent border
            side = "right" if str(next_node) != str(tags["osm_node_ids"][0]) else "left"

            return self.get_adjacent_border_from_way(tags, side)


    def flip_orientation_if_required(self):
        # compute the island or sidewalk for the two footways_orientations
        id1, type1 = self.get_adjacent_border(self.footways_orientations[0])
        id2, type2 = self.get_adjacent_border(self.footways_orientations[1])

        self.sides = [(id1, type1), (id2, type2)]
        if (self.sides[0] == self.sides[1]):
            print("ERROR: two sides with the same sidewalk or island")
        # check if oriented in the inverted direction
        if type1 == type2:
            inverted = int(id1) > int(id2)
        else:
            inverted = type1 == "island"

        # if required, invert the orientations
        if inverted:
            self.footways_orientations = self.footways_orientations[::-1]
            self.sides = self.sides[::-1]

    # get ID of the sidewalks adjacent to this crossing
    def get_sidewalk_ids(self):
        return [x[0] for x in self.sides if x[1] == "sidewalk"]


    def compute_way_orientations(self):
        self.roadway_nodes = self.get_adjacent_roadways_nodes()

        vectors = self.build_vectors(self.roadway_nodes)
        self.way_orientations = sorted([math.atan2(-x[1], x[0]) for x in vectors])


    def compute_footway_orientations(self):
        self.footway_nodes = self.get_adjacent_footways_nodes()

        if len(self.footway_nodes) > 2:
            print("Error: bad number of footways:", len(self.footway_nodes))
            self.footway_orientations = []
        if len(self.footway_nodes) != 0:
            vector_footways = self.build_vectors(self.footway_nodes)
            self.footways_orientations = sorted([math.atan2(-x[1], x[0]) for x in vector_footways])
        else:
            # guess the correct split a maximum angle heuristic
            angle_with_pred = [ a - b for a, b in zip(self.way_orientations, [self.way_orientations[-1]- 2 * math.pi] + self.way_orientations)]
            max_id = angle_with_pred.index(max(angle_with_pred))
            pred = max_id - 1
            if pred < 0:
                pred = len(self.way_orientations) - 1
            self.footways_orientations = [u.Utils.angle_mean(self.way_orientations[max_id], self.way_orientations[pred])]
        
        # if required, add the opposite orientation
        if len(self.footways_orientations) == 1:
            opposite = self.footways_orientations[0] + math.pi
            if opposite > math.pi:
                opposite -= 2 * math.pi
            self.footways_orientations.append(opposite)

    def compute_adjacent_ways_ditribution(self):
        # set angles up to angle1 using modulo 2pi 
        angle1 = self.footways_orientations[0]
        angle2 = self.footways_orientations[1] 
        if angle2 < angle1:
            angle2 += 2 * math.pi
        way_orientations_unfold = [a if a > angle1 else a + 2 * math.pi for a in self.way_orientations]

        # identify the side of each way wrt the two footways
        way_side = [0 if a < angle2 else 1 for a in way_orientations_unfold]
        ways_and_orientations = list(zip(self.roadway_nodes, way_side))

        # shift list to reach the angle1 orientation
        while ways_and_orientations[0][1] == ways_and_orientations[-1][1]:
            ways_and_orientations = ways_and_orientations[1:] + ways_and_orientations[:1]

        # finally build the ordered list of ways on each side of the footways
        self.ways_side1 = [w for w, s in ways_and_orientations if s == 0]
        self.ways_side2 = [w for w, s in ways_and_orientations if s == 1]
        

    def compute_crossing_profile_oneside(self, ways, invert):
        # TODO: improve this naive merge where the sequence is not really computed
        # assuming that there is only one set of backward lanes and one set of forward lanes
        nb_backward = 0
        nb_forward = 0
        total_width = 0
        for nw in ways:
            if nw in self.osm_input[self.node_id]:
                edge = self.osm_input[self.node_id][nw][0]
                nb_b, nb_f, w = u.Utils.evaluate_way_composition(edge)
            else:
                edge = self.osm_input[nw][self.node_id][0]
                nb_f, nb_b, w = u.Utils.evaluate_way_composition(edge)
            nb_backward += nb_b
            nb_forward += nb_f
            total_width += w * (nb_f + nb_b)


        if invert:
            return nb_forward, nb_backward, total_width / (nb_forward + nb_backward)
        else:
            return nb_backward, nb_forward, total_width / (nb_forward + nb_backward)


    def compute_crossing_profile(self):
        # TODO: improve this naive approach implemented in this fonction
        # where there is only two directions (island being only between both directions)

        self.has_island = "crossing:island" in self.osm_input.nodes[self.node_id] and self.osm_input.nodes[self.node_id]["crossing:island"] == "yes"

        # for each side, compute the distribution
        profile1 = self.compute_crossing_profile_oneside(self.ways_side1, False)
        profile2 = self.compute_crossing_profile_oneside(self.ways_side2, True)

        # choose the largest side as the final profile
        nbside1 = profile1[0] + profile1[1]
        nbside2 = profile2[0] + profile2[1]

        if nbside1 > nbside2:
            self.nb_lanes_backward = profile1[0]
            self.nb_lanes_forward = profile1[1]
        else:
            self.nb_lanes_backward = profile2[0]
            self.nb_lanes_forward = profile2[1]

        # use the maximum width
        self.lane_width = max(profile1[2], profile2[2])


    def compute_location(self):
        self.x = self.osm_input.nodes[self.node_id]["x"]
        self.y = self.osm_input.nodes[self.node_id]["y"]


    def compute_final_orientation(self):
        self.bearing = u.Utils.angle_mean(self.footways_orientations[0], math.pi + self.footways_orientations[1])
        self.bearing_confidence = len(self.footway_nodes) != 0

    def get_adjacent_roadways_nodes(self):
        return [x for x in self.osm_input[self.node_id] if u.Utils.is_roadway_edge(self.osm_input[self.node_id][x][0])]


    def get_adjacent_footways_nodes(self):
        return [x for x in self.osm_input[self.node_id] if u.Utils.is_footway_edge(self.osm_input[self.node_id][x][0])]


    def build_vectors(self, nodes):
        return [u.Utils.normalized_vector(self.osm_input.nodes[self.node_id], self.osm_input.nodes[n]) for n in nodes]


    def has_adjacent_crossing(osm_input, cr_input, node, radius = 5):
        # check for all nodes near to the given node
        for n in osm_input.nodes:
            if n != node and Crossing.is_crossing(n, cr_input):
                if u.Utils.edge_length(osm_input.nodes[n], osm_input.nodes[node]) < radius:
                    return True

        return False

    
    def create_crossings(osm_input, cr_input, osm_input_oriented, distance_kerb_footway, remove_doubled_crossings):
        crossings = dict([(n, Crossing(n, osm_input, cr_input, osm_input_oriented, distance_kerb_footway)) for n in osm_input.nodes if 
                      osm_input.nodes[n]["type"] == "input" and Crossing.is_crossing(n, cr_input)])

        if remove_doubled_crossings:
            print("Removing double crossings")
            # for each crossing
            for n in list(crossings.keys()):
                # if this crossing is on a traffic light node
                if "highway" in osm_input.nodes[n] and osm_input.nodes[n]["highway"] == "traffic_signals":
                    if Crossing.has_adjacent_crossing(osm_input, cr_input, n):
                        del crossings[n]

        return crossings

    def is_inside(self, region):
        return region.contains(Point(Point(self.osm_input.nodes[self.node_id]["x"], self.osm_input.nodes[self.node_id]["y"])))

    def is_crossing(node, cr_input):
        tags = u.Utils.get_initial_node_tags(cr_input, node)
        return tags and tags["type"] == "crosswalk"


    def get_line_representation(self, length = 1):
        x = self.osm_input.nodes[self.node_id]["x"]
        y = self.osm_input.nodes[self.node_id]["y"]
        shiftx = -math.cos(self.bearing) * length
        shifty = math.sin(self.bearing) * length
        return [(x - shiftx, y - shifty), (x, y), (x + shiftx, y + shifty)]


    def get_location(self):
        x = self.osm_input.nodes[self.node_id]["x"]
        y = self.osm_input.nodes[self.node_id]["y"]
        return (x, y)

    
    def is_first_side(self, id, nature):
        if self.sides[0][1] != nature:
            return False
        else:
            return str(self.sides[0][0]) == str(id)


    def get_location_on_sidewalk(self, id_sidewalk):
        return self.get_location_on_side(id_sidewalk, "sidewalk")


    def get_location_on_island(self, id_island):
        return self.get_location_on_side(id_island, "island")


    def get_location_on_side(self, id, nature):
        x = self.osm_input.nodes[self.node_id]["x"]
        y = self.osm_input.nodes[self.node_id]["y"]

        nb = self.nb_lanes_backward + self.nb_lanes_forward
        length = nb * self.lane_width / 2 + self.distance_kerb_footway

        bearing = self.footways_orientations[0 if self.is_first_side(id, nature) else 1]

        shiftx = -math.cos(bearing) * length
        shifty = math.sin(bearing) * length
        return (x + shiftx, y + shifty)

 

    def getGeometry(self):
        return Point(self.osm_input.nodes[self.node_id]["x"], self.osm_input.nodes[self.node_id]["y"])

    def getCrossingElements(self):
        nb = self.nb_lanes_backward + self.nb_lanes_forward
        start_shift = (nb - 1) * self.lane_width / 2
        total_width = nb * self.lane_width
        if self.has_island:
            lane_width_effective = (total_width - self.island_width) / nb
        else: 
            lane_width_effective = self.lane_width

        elements = []
        x = self.osm_input.nodes[self.node_id]["x"]
        y = self.osm_input.nodes[self.node_id]["y"]

        # crossings
        for i in range(nb):
            shift = -start_shift + i * lane_width_effective + (self.island_width if self.has_island and i >= self.nb_lanes_forward else 0)
            shiftx = -math.cos(self.bearing) * shift
            shifty = math.sin(self.bearing) * shift
            elements.append({ "type": "crossing",
                              "geometry": Point(x + shiftx, y + shifty),
                              "lane_orientation": "forward" if i < self.nb_lanes_forward else "backward",
                              "lane_width": lane_width_effective })

        # separators
        start_shift -= self.lane_width / 2
        for i in range(nb - 1):
            shift = -start_shift + i * lane_width_effective
            if self.has_island:
                if i + 1== self.nb_lanes_forward:
                    shift += self.island_width / 2
                elif i + 1 > self.nb_lanes_forward:
                    shift += self.island_width
            shiftx = -math.cos(self.bearing) * shift
            shifty = math.sin(self.bearing) * shift
            island = self.has_island and i + 1 == self.nb_lanes_backward
            elements.append({ "type": "traffic_island" if island else "lane_separator",
                              "geometry": Point(x + shiftx, y + shifty),
                              "lane_orientation": None,
                              "lane_width": self.island_width if island else 0 })

        return elements


    def toGDFCrossings(crossings, details = True):
        d = {'type': [], 
             'osm_id': [],
             'geometry': [],
             'orientation': [],
             'orientation_confidence': [],
             'lane_width': [] }

        if details:
            d['lane_orientation'] = []
        else:
            d['has_island'] = []
            d['nb_lanes_backward'] = []
            d['nb_lanes_forward'] = []

        for cid in crossings:
            c = crossings[cid]
            if details:
                for e in c.getCrossingElements():
                    d["type"].append(e["type"])
                    d["osm_id"].append(c.node_id)
                    d["geometry"].append(e["geometry"])
                    d["orientation"].append(-c.bearing + math.pi / 2)
                    d['lane_orientation'].append(e["lane_orientation"])
                    d["orientation_confidence"].append(c.bearing_confidence)
                    d["lane_width"].append(e["lane_width"])
            else:
                d["type"].append("crossing")
                d["orientation"].append(-c.bearing + math.pi / 2)
                d["orientation_confidence"].append(c.bearing_confidence)
                d["osm_id"].append(c.node_id)
                d["geometry"].append(c.getGeometry())
                d["has_island"].append(c.has_island)
                d["nb_lanes_backward"].append(c.nb_lanes_backward)
                d["nb_lanes_forward"].append(c.nb_lanes_forward)
                d["lane_width"].append(c.lane_width)

        return geopandas.GeoDataFrame(d, crs=2154)


class Branch:

    def __init__(self, name, id, osm_input, cr_input, distance_kerb_footway):
        self.ways = []
        self.name = name
        self.id = id
        self.osm_input = osm_input
        self.cr_input = cr_input
        self.distance_kerb_footway = distance_kerb_footway

    
    def add_way(self, way):
        self.ways.append(way)

    
    def nbWays(self):
        return len(self.ways)


    # return all the edges contained in the initial intersection
    # that are not part of this branch
    def get_other_edges(self):
        result = []
        for index, elem in self.cr_input.iterrows():
            if elem["type"] in ["branch", "way"] and elem["name"] != self.name:
                ids = list(map(int, elem["osm_node_ids"]))
                result.append(ids)
        return result


    def shift_middle_line(self, shifts, direction):
        edges = [self.middle_line.parallel_offset(s, direction) for s in shifts]
        return LineString([edges[0].coords[0], edges[1].coords[1]])

    def build_two_sidewalks(self, use_fixed_width_on_branches):
        from statistics import mean
        # the shifts corresponds to half the widths of the street
        shifts = [x / 2 for x in self.widths]

        if use_fixed_width_on_branches:
            shifts = [mean(shifts) for x in shifts]
        
        # compute the two lines (one in each side)
        result = [StraightSidewalk(self.shift_middle_line(shifts, "left"),
                                   self.sides[0],
                                   "left"),
                   StraightSidewalk(self.shift_middle_line(shifts, "right"),
                                   self.sides[1],
                                   "right")]

        # shift them if required
        buf = u.Utils.get_edges_buffered_by_osm(self.get_other_edges(), self.osm_input, self.distance_kerb_footway).boundary
        for i, s in enumerate(result):
            if s.edge.intersects(buf):
                intersections = s.edge.intersection(buf)
                if not intersections.is_empty and isinstance(intersections, Point):
                    result[i].update_first_node(intersections)

        self.sidewalks = result



    # maximum distance between two extremity points of the ways
    def get_initial_branche_width(self):
        edges = []
        distance = 0

        for w in self.sides:
            osm = [self.osm_input.nodes[int(x)] for x in w.consolidated_polybranch[:2]] # use the first two elements
            emeters = LineString([(x["x"], x["y"]) for x in osm])
            if len(edges) != 0:
                for ee in edges:
                    d = ee.distance(emeters)
                    if d > distance:
                        distance = d
            edges.append(emeters)

        return distance


    def build_middle_way(self):
        self.middle_line = StraightWay.build_middle_line(self.sides[0], self.sides[1])
            
    
    def compute_widths(self):
        # for each extremity of the middle line
        self.widths = []
        for p in self.middle_line.coords:

            # project it on each polybranches and select two furthest points
            p1, e1 = self.sides[0].get_projection_on_polybranch(p)
            p2, e2 = self.sides[1].get_projection_on_polybranch(p)

            # for each point estimate the width of the way
            interdistance = u.Utils.edge_length(p1, p2)
            w1 = u.Utils.evaluate_width_way(self.osm_input[e1[0]][e1[1]][0]) / 2 + self.distance_kerb_footway
            w2 = u.Utils.evaluate_width_way(self.osm_input[e2[0]][e2[1]][0]) / 2 + self.distance_kerb_footway

            # compute the final width
            self.widths.append(interdistance + w1 + w2)


    def build_sidewalk_straightways(self):
        # get the external simple ways (they are bordered by a sidewalk)
        self.simple_sides = [ w for w in self.ways if w.has_sidewalk()]

        if len(self.simple_sides) > 2:
            print("ERROR: more than two ways with a sidewalk (", self.name, ")")
            return

        self.single_side = False
        # if only one way, we duplicate it
        if len(self.simple_sides) == 1:
            if self.simple_sides[0].has_sidewalks_both_sides():
                self.simple_sides.append(self.simple_sides[0])
            else:
                print("ERROR: only one way in the branch, but with missing sidewalks (", self.name, ")")
                return
        else:
            # order them according to the id of the sidewalk
            self.simple_sides = sorted(self.simple_sides, key=lambda s: int(s.get_sidewalk_id()))
            # and flip in case we are at a branch sharing first and last sidewalks (by id)
            if int(self.simple_sides[1].get_sidewalk_id()) > int(self.simple_sides[0].get_sidewalk_id()) + 1:
                self.simple_sides = [self.simple_sides[1], self.simple_sides[0]]

        # build their extension as straightline
        self.sides = [StraightWay.build_from_simpleway(self.simple_sides[0], self.osm_input, True), # always choose the left
                       StraightWay.build_from_simpleway(self.simple_sides[1], self.osm_input, False)] # always choose the right



    def get_sidewalks(self, use_fixed_width_on_branches):

        self.build_sidewalk_straightways()

        # TODO: shift each extremity of each sidewalk wrt the estimated width of each extremity

        self.build_middle_way()

        self.compute_widths()

        self.build_two_sidewalks(use_fixed_width_on_branches)

        return self.sidewalks

    
    def getGeometry(self):
        return self.middle_line


    def toGDFBranches(branches):
        d = {'type': [], 'osm_id': [], 'geometry': []}

        for bid in branches:
            b = branches[bid]
            d["type"].append("branch")
            d["osm_id"].append(";".join([w.get_edge_id() for w in b.ways]))
            d["geometry"].append(b.getGeometry())

        return geopandas.GeoDataFrame(d, crs=2154)
