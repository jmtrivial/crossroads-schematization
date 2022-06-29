from numpy import linalg
import shapely.ops
import numpy as np


class Utils:

    def is_roadway_edge(osm_edge):
        if not "highway" in osm_edge:
            return False
        with_cars = ["motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential", "service"]
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

