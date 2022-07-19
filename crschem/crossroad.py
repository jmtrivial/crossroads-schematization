from shapely.geometry import Point, LineString, MultiLineString, LinearRing, Polygon

import numpy as np
from numpy import linalg
import copy
import math
import shapely.ops
import osmnx

from . import utils as u
from . import processing as p


class StraightWay:
    
    # parameters (and attributes)
    # - n1 is the interior node, n2 is the exterior node  (centripetal orientation)
    # - polybranch: an extended path from n1, n2
    # - edge_tags: tags from crdesc
    # - same_osm_orientation: boolean (is the OSM orientation similar)
    # - is_crossing_interior_node: return true if the first node is a crossing
    def __init__(self, n1, n2, polybranch, edge_tags, same_osm_orientation, is_crossing_interior_node):
        self.n1 = n1
        self.n2 = n2
        self.polybranch = polybranch
        self.edge = None
        self.array = None
        self.same_osm_orientation = same_osm_orientation
        self.edge_tags = edge_tags
        self.is_crossing_interior_node = is_crossing_interior_node
        self.lz = p.Linearization()


    def compute_linear_edge(self, G):
        self.osm_input = G # TODO : remove that
        self.edge = self.lz.process(p.Expander.convert_to_linestring(G, self.polybranch))
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


    def is_crossing_inner_node(self):
        return self.is_crossing_interior_node


    def has_sidewalk(self):
        return self.edge_tags["left_sidewalk"] != "" or self.edge_tags["right_sidewalk"] != ""


    def has_sidewalks_both_sides(self):
        return self.edge_tags["left_sidewalk"] != "" and self.edge_tags["right_sidewalk"] != ""


    def point(self, i):
        return Point(self.array[i])


    def get_initial_edge_id(self):
        if self.same_osm_orientation:
            return self.get_edge_id()
        else:
            return self.get_edge_id_reverse()


    def get_edge_id(self):
        return str(self.n1) + ";" + str(self.n2)


    def get_edge_id_reverse(self):
        return str(self.n2) + ";" + str(self.n1)
    

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
    
    
    # basic evaluation of the width using number of lanes, and type of highway
    def evaluate_width_way(self, osm_graph):
        return u.Utils.evaluate_width_way(osm_graph[self.polybranch[0]][self.polybranch[1]][0])


class StraightSidewalk:

    def __init__(self, edge, description, same_orientation, side, is_crossing_inner_node):
        self.edge = edge
        self.description = description
        self.same_orientation = same_orientation
        self.side = side
        if is_crossing_inner_node:
            # extends the sidewalk in the inner direction if the inner point is a crossing
            self.extends_start()


    def __str__(self):
        return str(self.edge) + "; side: " + str(self.side) + "; same orientation: " + str(self.same_orientation)


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
        return self.description["id"]


class TurningSidewalk:

    def __init__(self, str_sidewalks, osm_input, distance_kerb_footway):
        self.distance_kerb_footway = distance_kerb_footway

        self.str_sidewalks = str_sidewalks
        
        self.osm_input = osm_input

        self.build_simple_turn()

        if self.way is None or self.way.is_empty:
            self.build_beveled_turn()
        else:
            if self.is_flexible_way:
                self.adjust_intersection()


    def get_middle_bevel(self):
        p1 = self.str_sidewalks[0].edge.coords[0]
        p2 = self.str_sidewalks[1].edge.coords[0]
        return [(x + y) / 2 for x, y in zip(p1, p2)]


    def adjust_intersection(self):
        middle_sw = self.way.coords[1]
        buffered_osm = u.Utils.get_buffered_osm(self.osm_input, self.distance_kerb_footway)

        # first check for basic interesction
        middle_bevel = Point(self.get_middle_bevel())
        if buffered_osm.intersects(middle_bevel):
            self.build_beveled_turn()
        else:
            # otherwise build a more complex turn
            edge = LineString((middle_bevel, middle_sw))
            
            elements = buffered_osm.boundary.intersection(edge)
            if not elements.is_empty:
                nearest = shapely.ops.nearest_points(middle_bevel, elements)
                self.build_beveled_turn(nearest[1])


    def branch_names(self):
        return [x.description["name"] for x in self.str_sidewalks]
    

    def sidewalk_id(self):
        return self.str_sidewalks[0].sidewalk_id()


    def get_intersection(self):
        return self.str_sidewalks[0].get_intersection(self.str_sidewalks[1])


    def is_before_end_of_edge(edge, node):
        return u.Utils.norm_and_dot(u.Utils.vector(edge[0], edge[1]), u.Utils.vector(edge[1], node)) < 0


    def is_before_begin_of_edge(edge, node):
        return u.Utils.norm_and_dot(u.Utils.vector(edge[0], edge[1]), u.Utils.vector(edge[0], node)) < 0


    def build_simple_turn(self):
        sw1 = np.asarray(self.str_sidewalks[0].edge.coords)
        sw2 = np.asarray(self.str_sidewalks[1].edge.coords)
        self.intersection = self.get_intersection()
        if not self.intersection.is_empty and (TurningSidewalk.is_before_end_of_edge(self.str_sidewalks[0].edge.coords, self.intersection) and \
           TurningSidewalk.is_before_end_of_edge(self.str_sidewalks[1].edge.coords, self.intersection)) :
            self.way = LineString([sw1[1], self.intersection, sw2[1]])
            self.is_flexible_way = TurningSidewalk.is_before_begin_of_edge(self.str_sidewalks[0].edge.coords, self.intersection) and \
                                    TurningSidewalk.is_before_begin_of_edge(self.str_sidewalks[1].edge.coords, self.intersection)
        else:
            self.way = None
            self.is_flexible_way = True


    def build_beveled_turn(self, middle = None):
        sw1 = np.asarray(self.str_sidewalks[0].edge.coords)
        sw2 = np.asarray(self.str_sidewalks[1].edge.coords)
        if middle is None:
            self.way = LineString(np.concatenate((sw1[::-1], sw2)))
        else:
            self.way = LineString(np.concatenate((sw1[::-1], [middle.coords[0]], sw2)))


    def as_array(self):
        return np.asarray(self.way.coords)


    def buffer(self, size):
        return self.way.buffer(size)


    def getGeometry(self):
        return self.way


    def getOSMIds(self):
        return ";".join([x.getOSMIds() for x in self.str_sidewalks])

    def toGDFSidewalks(sidewalks):
        d = {'type': [], 'osm_id': [], 'geometry': []}

        for s in sidewalks:
            d["type"].append("sidewalk")
            d["osm_id"].append(s.getOSMIds())
            d["geometry"].append(s.getGeometry())

        return geopandas.GeoDataFrame(d, crs=2154)


class TrafficIsland:

    def __init__(self, edgelist, osm_input, cr_input):
        self.edgelist = [list(map(int, x.split(";"))) for x in edgelist]
        self.osm_input = osm_input
        self.cr_input = cr_input

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

        # TODO: if the polygon is not closed, a part is missing in the original data (but available in OSM)



    def get_linearring(self):
        points = [self.osm_input.nodes[x] for x in self.polygon]
        return LinearRing([(p["x"], p["y"]) for p in points])


    def compute_center_and_radius(self, crossings):
        local_crossings = [self.osm_input.nodes[c] for c in crossings if c in self.polygon]
        if len(local_crossings) != 0:
            l = local_crossings
            self.is_reachable = True
        else:
            l = [self.osm_input.nodes[x] for x in self.polygon]
            self.is_reachable = False
        xs = [c["x"] for c in l]
        ys = [c["y"] for c in l]
        self.center = (sum(xs) / len(xs), sum(ys) / len(ys))
        ds = [osmnx.distance.euclidean_dist_vec(c["x"], c["y"], self.center[0], self.center[1]) for c in l]
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


    def get_straight_island_direction(self, section, elength):
        
        def build_sides(section):
            distances = [0] + [u.Utils.edge_length(a, b) for a, b in zip(section, section[1:])]
            max_d = max(distances)
            if max_d > elength * 1.00001:
                # split and only preserve each side of the virtual edge
                from more_itertools import locate
                elements = list(locate(distances, lambda e: e >= elength * 1.00001))
                side1 = section[0:elements[0]]
                section2 = copy.deepcopy(section)
                section2.reverse()
                side2 = section2[0:len(section2) - elements[-1]]
            else:
                # split by mean distance
                cumuld_dists = np.cumsum(distances)
                mid = cumuld_dists[-1] / 2
                side1 = [s for s, d in zip(section, cumuld_dists) if d < mid]
                side2 = [s for s, d in zip(section, cumuld_dists) if d >= mid]
                side2.reverse()
            return LineString(side1), LineString(side2)


        # build two polylines starting from the crosswalks (and stop them in a cleaver way)
        l1, l2 = build_sides(section)

        # linearize the two polylines
        lz = p.Linearization(length=50, initial_step=elength / 2, exponential_coef=1.2)
        ll1 = lz.process(l1)
        ll2 = lz.process(l2)

        # use their directions to get a global direction for the island
        n1 = u.Utils.normalized_vector(ll1.coords[0], ll1.coords[1])
        n2 = u.Utils.normalized_vector(ll2.coords[0], ll2.coords[1])
        vectors = [n1, n2]
        final_vector = (sum([v[0] for v in vectors]) / len(vectors), sum([v[1] for v in vectors]) / len(vectors))

        # build a long edge according to this direction
        length = 200
        return Point(self.center[0] + final_vector[0] * length, self.center[1] + final_vector[1] * length)



    def get_edge_extremity_from_section(self, section, inner_region):
        # build polyline
        polyline = LineString([Point(self.osm_input.nodes[n]["x"], self.osm_input.nodes[n]["y"]) for n in section])

        # discretize it with a 1/step meter step
        step = 5
        resampled_polyline = [polyline.interpolate(float(x) / step) for x in range(0, int(polyline.length * step))]
  

        # only preserve node in the inner region
        selected = [p for p in resampled_polyline if inner_region.intersects(p)]        
        
        if len(selected) == 0:
            return None

        # compute a straight island
        other_in_edge = self.get_straight_island_direction(selected, 1.0 / step)

        if other_in_edge is None:
            return None

        # TODO DEBUG
        # import matplotlib.pyplot as plt
        # plt.plot([c.x for c in selected], [c.y for c in selected], 'ok')
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

        if self.is_reachable:
            #compute supplementary edges if some of points of the polygons are far from the center
            self.compute_edges(crossings, inner_region)


    def getGeometry(self):
        return Point(self.center)


    def toGDFTrafficIslands(traffic_islands, only_reachable = True):
        d = {'type': [], 'osm_id': [], 'geometry': []}

        for t in traffic_islands:
            if t.is_reachable or not only_reachable:
                d["type"].append("traffic_island")
                d["osm_id"].append(";".join(map(str, t.polygon)))
                d["geometry"].append(t.getGeometry())

        return geopandas.GeoDataFrame(d, crs=2154)


class Crossing:

    def __init__(self, node_id, osm_input):

        self.node_id = node_id
        self.osm_input = osm_input

        self.compute_location()
        self.compute_orientation()


    def compute_location(self):
        self.x = self.osm_input.nodes[self.node_id]["x"]
        self.y = self.osm_input.nodes[self.node_id]["y"]


    def compute_orientation(self):
        footways = self.get_adjacent_footways_nodes()
        if len(footways) != 0:
            self.bearing = self.compute_parallel_orientation(footways)
            self.bearing_confidence = False
        else:
            roadways = self.get_adjacent_roadways_nodes()
            if len(roadways) == 0:
                print("Error: no adjacent road")
                for x in self.osm_input[self.node_id]:
                    print(" ", self.osm_input[self.node_id][x][0])
                self.bearing_confidence = Fakse
                self.bearing = 0
            else:
                self.bearing = self.compute_orthogonal_orientation(roadways)
                self.bearing_confidence = True


    def get_adjacent_roadways_nodes(self):
        # TODO: est-ce qu'on n'essayerait pas de prendre des nodes un peu plus loin sur les ways?
        return [x for x in self.osm_input[self.node_id] if u.Utils.is_roadway_edge(self.osm_input[self.node_id][x][0])]


    def get_adjacent_footways_nodes(self):
        return [x for x in self.osm_input[self.node_id] if u.Utils.is_footway_edge(self.osm_input[self.node_id][x][0])]


    def compute_parallel_orientation(self, nodes):
        vectors = self.build_vectors(nodes)
        if len(vectors) == 1:
            return math.atan2(vectors[0][1], vectors[0][0])
        elif len(vectors) == 2:
            return math.atan2(vectors[0][1] - vectors[1][1], vectors[0][0] - vectors[1][0])
        elif len(vectors) == 3:
            # compute the orientations (in gradient) and sort them
            orientations = sorted([math.atan2(x[1], x[0]) for x in vectors])
            # compute the difference between consecutive orientations
            diff = [o[1] - o[0] for o in zip(orientations, orientations[1:] + orientations[:1])]
            diff = [ o + 2 * math.pi if o < 0 else o for o in diff]
            # identify the pair of branches with the smallest difference
            branchID1 = diff.index(min(diff))
            branchID2 = (branchID1 + 1) % len(vectors)
            # identify the other one
            otherBranchID = (branchID2 + 1) % len(vectors)
            # compute a mean of the global orientation, considering the other branch as an opposite one
            return u.Utils.angle_mean(u.Utils.angle_mean(orientations[branchID1], orientations[branchID2]), orientations[otherBranchID] + math.pi)
        else:
            print(vectors, [math.atan2(x[1], x[0]) for x in vectors])
            print("Error: bad number of edges to compute a direction", len(vectors))

    def compute_orthogonal_orientation(self, nodes):
        bearing = self.compute_parallel_orientation(nodes)
        bearing += math.pi / 2
        if bearing > 2 * math.pi:
            bearing -= 2 * math.pi
        return bearing

    def build_vectors(self, nodes):
        return [u.Utils.normalized_vector(self.osm_input.nodes[self.node_id], self.osm_input.nodes[n]) for n in nodes]

    
    def create_crossings(osm_input, cr_input, region = None):
        return dict([(n, Crossing(n, osm_input)) for n in osm_input.nodes if 
                      (region is None or region.contains(Point(osm_input.nodes[n]["x"], osm_input.nodes[n]["y"]))) and
                      osm_input.nodes[n]["type"] == "input" and Crossing.is_crossing(n, cr_input)])



    def is_crossing(node, cr_input):
        tags = u.Utils.get_initial_node_tags(cr_input, node)
        return tags and tags["type"] == "crosswalk"


    def get_line_representation(self, length = 1):
        x = self.osm_input.nodes[self.node_id]["x"]
        y = self.osm_input.nodes[self.node_id]["y"]
        shiftx = math.cos(self.bearing) * length
        shifty = math.sin(self.bearing) * length
        return [(x - shiftx, y - shifty), (x, y), (x + shiftx, y + shifty)]


    def getGeometry(self):
        return Point(self.osm_input.nodes[self.node_id]["x"], self.osm_input.nodes[self.node_id]["y"])

    def toGDFCrossings(crossings):
        d = {'type': [], 'orientation': [], 'osm_id': [], 'geometry': []}

        for cid in crossings:
            c = crossings[cid]
            d["type"].append("crossing")
            d["orientation"].append(c.bearing)
            d["orientation_confidence"].append(c.bearing_confidence)
            d["osm_id"].append(c.node_id)
            d["geometry"].append(c.getGeometry())

        return geopandas.GeoDataFrame(d, crs=2154)


class Branch:

    def __init__(self, name, osm_input, cr_input, distance_kerb_footway):
        self.ways = []
        self.name = name
        self.osm_input = osm_input
        self.cr_input = cr_input
        self.distance_kerb_footway = distance_kerb_footway

    
    def add_way(self, way):
        self.ways.append(way)

    
    def nbWays(self):
        return len(self.ways)


    def build_two_sidewalks(self):
        # the shift corresponds to half the width of the street
        shift = self.width / 2
        
        # compute the two lines (one in each side)
        return [StraightSidewalk(self.middle_line.parallel_offset(shift, "left"), 
                                 self.sides[0].edge_tags,
                                 self.sides[0].same_osm_orientation,
                                 "left", 
                                 self.sides[0].is_crossing_inner_node()), 
                StraightSidewalk(self.middle_line.parallel_offset(shift, "right"),
                                 self.sides[1].edge_tags,
                                 self.sides[1].same_osm_orientation,
                                 "right", 
                                 self.sides[1].is_crossing_inner_node())]


    # maximum distance between two extremity points of the ways
    def get_initial_branche_width(self):
        edges = []
        distance = 0

        for w in self.sides:
            osm = [self.osm_input.nodes[int(x)] for x in w.polybranch[:2]]
            emeters = LineString([(x["x"], x["y"]) for x in osm])
            if len(edges) != 0:
                for ee in edges:
                    d = ee.distance(emeters)
                    if d > distance:
                        distance = d
            edges.append(emeters)

        return distance


    def build_middle_way(self):
        if self.single_side:
            self.middle_line = self.sides[0].edge
        else:
            self.middle_line = StraightWay.build_middle_line(self.sides[0], self.sides[1])
            
    
    def compute_width(self):
        self.width = self.sides[0].evaluate_width_way(self.osm_input) / 2 + \
                    self.sides[1].evaluate_width_way(self.osm_input) / 2 + \
                    self.get_initial_branche_width() + 2 * self.distance_kerb_footway

    def select_sidewalk_ways(self):
        self.sides = [ w for w in self.ways if w.has_sidewalk()]
        self.single_side = False
        if len(self.sides) == 1 and self.sides[0].has_sidewalks_both_sides():
            self.sides.append(self.sides[0])
            self.single_side = True

    def get_sidewalks(self):

        self.select_sidewalk_ways()

        if len(self.sides) != 2:
            print("Not supported configuration:", len(self.sides), "sidewalk on branch", bid)
            self.sidewalks = None
        else:

            self.build_middle_way()

            self.compute_width()

            self.sidewalks = self.build_two_sidewalks()

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
