import math
from shapely.geometry import Point
import geopandas

from .. import utils as u
from crseg.utils import Util as ucr

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
        else:
            print("WARNING: no road node")


        self.compute_crossing_profile()

        self.adjust_lane_width_by_bearing()



    def compute_road_orientation_from_side(self, ways):
        vectors = self.build_vectors(ways)
        orientations = [math.atan2(-x[1], x[0]) for x in vectors]

        # ways has been sorted before, thus we can use their order and use
        # only first and last
        return u.Utils.angle_mean(orientations[0], orientations[-1])


    def compute_road_orientation(self):
        b1 = self.compute_road_orientation_from_side(self.ways_side1)
        b2 = self.compute_road_orientation_from_side(self.ways_side2)

        # compute mean
        self.road_bearing = u.Utils.angle_mean(b1, math.pi + b2)

    def adjust_lane_width_by_bearing(self):
        # compute road_bearing
        self.compute_road_orientation()

        # adjust lane_width by combining road bearing and crossing bearing
        self.lane_width = abs(self.lane_width / math.sin(self.bearing - self.road_bearing))


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


    def has_adjacent_crossing(osm_input, cr_input, node, radius = 7):
        if len(osm_input[node]) != 2:
            return False

        # check for all nodes near to the given node
        for n in sum([ucr.get_path_to_biffurcation(osm_input, node, x) for x in osm_input[node]], []):
            if n != node and Crossing.is_crossing(n, cr_input):
                distance = u.Utils.edge_length(osm_input.nodes[n], osm_input.nodes[node])
                if distance < radius:
                    return True

        return False

    
    def is_crossing_osm(node, osm_input):
        return ("highway" in osm_input.nodes[node] and osm_input.nodes[node]["highway"] == "crossing") or ("crossing" in osm_input.nodes[node])

    def create_crossings(osm_input, cr_input, osm_input_oriented, distance_kerb_footway, remove_doubled_crossings):
        crossings = dict([(n, Crossing(n, osm_input, cr_input, osm_input_oriented, distance_kerb_footway)) for n in osm_input.nodes if 
                      osm_input.nodes[n]["type"] == "input" and Crossing.is_crossing(n, cr_input) and Crossing.is_crossing_osm(n, osm_input)])

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
