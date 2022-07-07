from shapely.geometry import Point, LineString, MultiLineString, LinearRing, Polygon

import numpy as np
from numpy import linalg
import copy
import math


from . import utils as u
from . import processing as p

class StraightWay:
    
    def __init__(self, edge, interior_node, n1, n2, cr_input, is_crossing_interior_node):
        self.edge = edge
        self.array = np.asarray(edge.coords)
        self.interior_node = interior_node
        self.is_crossing_interior_node = is_crossing_interior_node
        self.n1 = n1
        self.n2 = n2
        self.cr_input = cr_input
        self.edge_tags = self.get_initial_branch_edge()


    def __str__(self):
        return str((self.edge, self.interior_node, (self.n1, self.n2)))


    def is_crossing_inner_node(self):
        return self.is_crossing_interior_node

    def get_initial_branch_edge(self):
        is_w = (self.cr_input["id"] == self.get_edge_id()) | (self.cr_input["id"] == self.get_edge_id_reverse())
        filtered = self.cr_input[is_w]
        if len(filtered) > 0:
            return filtered.iloc[0, :].to_dict()
        else:
            return None
        
    def point(self, i):
        return Point(self.array[i])
    
    def get_initial_edge_id(self):
        if self.interior_node == self.n1:
            return self.get_edge_id()
        else:
            if self.interior_node != self.n2:
                print("WRONG CONFIGURATION !!!")
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
        
        return LineString([LineString([e1_1, e2_1]).centroid, LineString([e1_2, e2_2]).centroid])
    
    
    # basic evaluation of the width using number of lanes, and type of highway
    def evaluate_width_way(self, osm_graph):
        gEdge = osm_graph[self.n1][self.n2][0]
        if "width" in gEdge and not re.match(r'^-?\d+(?:\.\d+)$', gEdge["width"]) is None:
            return float(gEdge["width"])
        elif "lanes" in gEdge:
            nb = int(gEdge["lanes"])
        else:
            if "oneway" in gEdge and gEdge["oneway"]:
                nb = 1
            else:
                nb = 2
        
        if "highway" in gEdge:
            if gEdge["highway"] in ["motorway", "trunk"]:
                width = 3.5
            elif gEdge["highway"] in ["primary"]:
                width = 3
            elif gEdge["highway"] in ["secondary"]:
                width = 3
            elif gEdge["highway"] in ["service"]:
                width = 2.25
            else:
                width = 2.75
        else:
            width = 3
        
        result = 0
        if ("cycleway" in gEdge and gEdge["cycleway"] == "track") or \
            ("cycleway:left" in gEdge and gEdge["cycleway:left"] == "track") or \
            ("cycleway:right" in gEdge and gEdge["cycleway:right"] == "track"):
            nb += 1 # ~ COVID tracks
        if ("cycleway:right" in gEdge and gEdge["cycleway:right"] == "lane") or \
           ("cycleway:left" in gEdge and gEdge["cycleway:left"] == "lane"):
            result += 1 # one meter per cycle lane


        result += nb * width

        return result


class StraightSidewalk:

    def __init__(self, edge, description, same_orientation, side, is_crossing_inner_node):
        self.edge = edge
        self.description = description
        self.same_orientation = same_orientation
        self.side = side
        if is_crossing_inner_node:
            # extends the sidewalk in the inner direction if the inner point is a crossing
            self.extends_start()


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

    def __init__(self, str_sidewalks, osm_input, osm_buffer_for_bevel = 2):
        self.osm_buffer_for_bevel = osm_buffer_for_bevel

        self.str_sidewalks = str_sidewalks
        
        self.osm_input = osm_input

        self.build_simple_turn()

        if self.way is None or self.way.is_empty or self.is_intersecting_osm():
            self.build_beveled_turn()


    def branch_names(self):
        return [x.description["name"] for x in self.str_sidewalks]
    

    def sidewalk_id(self):
        return self.str_sidewalks[0].sidewalk_id()


    def get_intersection(self):
        return self.str_sidewalks[0].get_intersection(self.str_sidewalks[1])


    def is_before_edge(edge, node):
        v1 = (edge[1][0] - edge[0][0], edge[1][1] - edge[0][1])
        v2 = (node.x - edge[0][0], node.y - edge[0][1])
        v1 /= np.linalg.norm(v1)
        v2 /= np.linalg.norm(v2)
        a = np.dot(v1, v2)
        return a < 0


    def build_simple_turn(self):
        sw1 = np.asarray(self.str_sidewalks[0].edge.coords)
        sw2 = np.asarray(self.str_sidewalks[1].edge.coords)
        intersection = self.get_intersection()
        if TurningSidewalk.is_before_edge(self.str_sidewalks[0].edge.coords, intersection) and \
           TurningSidewalk.is_before_edge(self.str_sidewalks[1].edge.coords, intersection):
            self.way = LineString([sw1[1], intersection, sw2[1]])
        else:
            self.way = None

    def is_intersecting_osm(self):
        for n1 in self.osm_input:
            for n2 in self.osm_input[n1]:
                if u.Utils.is_roadway_edge(self.osm_input[n1][n2][0]):
                    p1 = self.osm_input.nodes[n1]
                    p2 = self.osm_input.nodes[n2]
                    e = LineString([[p1["x"], p1["y"]], [p2["x"], p2["y"]]]).buffer(self.osm_buffer_for_bevel)
                    if not e.intersection(self.way).is_empty:
                        return True

        return False

    def build_beveled_turn(self):
        sw1 = np.asarray(self.str_sidewalks[0].edge.coords)
        sw2 = np.asarray(self.str_sidewalks[1].edge.coords)
        self.way = LineString(np.concatenate((sw1[::-1], sw2)))


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

    def __init__(self, edgelist, osm_input):
        self.edgelist = [list(map(int, x.split(";"))) for x in edgelist]
        self.osm_input = osm_input

        self.build_polygon()

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

    def get_linearring(self):
        points = [self.osm_input.nodes[x] for x in self.polygon]
        return LinearRing([(p["x"], p["y"]) for p in points])


    def compute_generalization(self, crossings):
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
        # TODO: compute supplementary edges if some of points of the polygons are far from 
        # this center


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
                      osm_input.nodes[n]["type"] == "input" and Crossing.is_crossing(n, osm_input)])



    def is_crossing(node, osm_input):
        return "crossing" in osm_input.nodes[node] and osm_input.nodes[node]["crossing"] != "no" and len(osm_input[node]) > 0


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


    def is_initial_edge_same_orientation(self, w):
        is_w = self.cr_input["id"] == w.get_initial_edge_id()
        filtered = self.cr_input[is_w]
        return len(filtered) > 0


    def build_two_sidewalks(self, way, width, 
                            oedge_left, so_left, is_crossing_inner_node_left,
                            oedge_right, so_right, is_crossing_inner_node_right):
        # the shift corresponds to half the width of the street
        shift = width / 2
        
        # compute the two lines (one in each side)
        return [StraightSidewalk(way.parallel_offset(shift, "left"), oedge_left, so_left, "left", is_crossing_inner_node_left), 
                StraightSidewalk(u.Utils.reverse_geom(way.parallel_offset(shift, "right")), oedge_right, so_right, "right", is_crossing_inner_node_right)]


    def get_initial_branche_width(self):
        edges = []
        distance = 0

        for w in self.ways:
            osm = [self.osm_input.nodes[int(x)] for x in [w.n1, w.n2]]
            emeters = LineString([(x["x"], x["y"]) for x in osm])
            if len(edges) != 0:
                for ee in edges:
                    d = ee.distance(emeters)
                    if d > distance:
                        distance = d
            edges.append(emeters)

        return distance

    def get_sidewalks(self):
        wbranches = self.ways
        if len(wbranches) == 1 and wbranches[0].edge_tags["left_sidewalk"] != "" and wbranches[0].edge_tags["right_sidewalk"] != "":
            self.width = wbranches[0].evaluate_width_way(self.osm_input) + 2 * self.distance_kerb_footway
            so = self.is_initial_edge_same_orientation(wbranches[0])
            self.middle_line = wbranches[0].edge
            self.sidewalks = self.build_two_sidewalks(self.middle_line, self.width, 
                                                      wbranches[0].edge_tags, so, wbranches[0].is_crossing_inner_node(),
                                                      wbranches[0].edge_tags, so, wbranches[0].is_crossing_inner_node())
        elif len(wbranches) == 2 and (wbranches[0].edge_tags["left_sidewalk"] != "" or wbranches[0].edge_tags["right_sidewalk"] != "") \
                and (wbranches[1].edge_tags["left_sidewalk"] != "" or wbranches[1].edge_tags["right_sidewalk"] != ""):
            # first build a middle line
            self.middle_line = StraightWay.build_middle_line(wbranches[0], wbranches[1])

            # then compute width of the street
            self.width = wbranches[0].evaluate_width_way(self.osm_input) / 2 + \
                    wbranches[1].evaluate_width_way(self.osm_input) / 2 + \
                    self.get_initial_branche_width() + 2 * self.distance_kerb_footway

            # evaluate orientation of the initial edges
            so = [self.is_initial_edge_same_orientation(wbranches[0]), 
                    self.is_initial_edge_same_orientation(wbranches[1])]
            # build two lines associated to the middle line (wrt correct direction)
            order = [0, 1]
            if not so[0]:
                order.reverse()
            if wbranches[order[0]].edge_tags["left_sidewalk"] == "":
                order.reverse()
            self.sidewalks = self.build_two_sidewalks(self.middle_line, self.width,
                                                            wbranches[order[0]].edge_tags, so[order[0]], wbranches[order[0]].is_crossing_inner_node(),
                                                            wbranches[order[1]].edge_tags, so[order[1]], wbranches[order[1]].is_crossing_inner_node())
        else:
            print("Not supported configuration:", len(wbranches), "sidewalk on branch", bid)

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
