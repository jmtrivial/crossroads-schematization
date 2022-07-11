from numpy import linalg
from shapely.geometry import LineString
import shapely.ops
import numpy as np
import math


class Utils:

    def is_roadway_edge(osm_edge):
        if not "highway" in osm_edge:
            return False
        with_cars = ["motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential", "service", "living_street"]
        return osm_edge["highway"] in with_cars + [ w + "_link" for w in with_cars]


    def is_footway_edge(osm_edge):
        if "highway" in osm_edge and osm_edge["highway"] == "footway":
            return True
        return False


    def vector(node1, node2):
        if isinstance(node1, list) or isinstance(node1, tuple):
            return [node2[0] - node1[0], node2[1] - node1[1]]
        else:
            return [node2["x"] - node1["x"], node2["y"] - node1["y"]]


    def normalized_vector(node1, node2):
        v = Utils.vector(node1, node2)
        return v / linalg.norm(np.array(v), 2)


    def reverse_geom(geom):
        def _reverse(x, y, z=None):
            if z:
                return x[::-1], y[::-1], z[::-1]
            return x[::-1], y[::-1]

        return shapely.ops.transform(_reverse, geom)


    def get_number_from_label(txt):
        if txt is None:
            return None
        if isinstance(txt, str):
            return int(txt)
        if math.isnan(txt):
            return None
        return int(txt)


    def angle_modulo(a):
        if a < 0:
            return angle_modulo(a + 2 * math.pi)
        elif a > 2 * math.pi:
            return angle_modulo(a - 2 * math.pi)
        else:
            return a


    def angle_mean(a1, a2):
        a1 = Utils.angle_modulo(a1)
        a2 = Utils.angle_modulo(a2)

        m1 = (a1 + a2) / 2
        if m1 > math.pi:
            return m1 - math.pi
        else:
            return m1

    
    def get_buffered_osm(osm, supplementary_width):

        regions = []
        for n1 in osm:
            for n2 in osm[n1]:
                edge = osm[n1][n2][0]
                if Utils.is_roadway_edge(edge):
                    p1 = osm.nodes[n1]
                    p2 = osm.nodes[n2]
                    e = LineString([[p1["x"], p1["y"]], [p2["x"], p2["y"]]]).buffer((Utils.evaluate_width_way(edge) + supplementary_width) / 2)
                    regions.append(e)
        return shapely.ops.unary_union(regions)

    def evaluate_width_way(gEdge):
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


    def get_initial_edge_tags(cr_input, osm_n1, osm_n2):
        is_w = cr_input["id"] == str(osm_n1) + ";" + str(osm_n2)
        filtered = cr_input[is_w]
        if len(filtered) > 0:
            return filtered.iloc[0, :].to_dict()
        else:
            return None