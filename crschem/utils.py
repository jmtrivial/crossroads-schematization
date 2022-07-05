from numpy import linalg
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