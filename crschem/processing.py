from shapely.geometry import Point, LineString, MultiLineString, LinearRing, Polygon
import shapely.ops
import numpy as np
from numpy import linalg
import osmnx as ox

from . import utils as u


class Linearization:
    
    def __init__(self, length=30, initial_step=1, exponential_coef=1.2):
        self.exponential_coef = exponential_coef
        self.length = length
        self.initial_step = initial_step
    

    def process(self, polyline):
        # discretize the polybranch following a density depending on the linear coordinate
        polydisbranch = self.discretize_polyline(polyline)
        polydisbranchcoords = np.asarray(polydisbranch.coords)

        # compute a a direction line
        line = self.compute_direction_line(polydisbranchcoords)

        # project last point on this line
        return LineString([polydisbranchcoords[0], Linearization.project_on_line(Point(polydisbranchcoords[-1]), line)])


    def exponential_coordinates(self, step1, length):
        result = [0, 1]
        n = 1
        while n < length:
            n *= self.exponential_coef
            result.append(n)
        return result


    def discretize_polyline(self, polyline):
        # exponential interpolation, starting from 1 meter
        return LineString([polyline.interpolate(x) for x in self.exponential_coordinates(self.initial_step, min(polyline.length, self.length))])


    def compute_direction_line(self, polyline):
        x = [a[0] for a in polyline]
        y = [a[1] for a in polyline]
        center = Point(sum(x) / len(x), sum(y) / len(y))
        start = polyline[0]
        v = [center.x - start[0], center.y - start[1]]
        v = v / linalg.norm(v)
        return LineString([start, start + v * self.length])


    def project_on_line(point, line):
        n = shapely.ops.nearest_points(point, line)
        return n[0]


class Expander:

    def __init__(self):
        self.bid = 0


    def reset_bid(self):
        self.bid = 0


    def process(self, G, n1, n2, left_first):
        result = Expander.extend_branch(G, n1, n2, left_first)
        self.bid += 1
        return self.bid, result


    def convert_to_linestring(G, polyline):
        return LineString([Point(G.nodes[x]["x"], G.nodes[x]["y"]) for x in polyline])


    def is_turn(m, c1, c2):
        v1 = [c1["x"] - m["x"], c1["y"] - m["y"]]
        v2 = [c2["x"] - m["x"], c2["y"] - m["y"]]
        uv1 = v1 / np.linalg.norm(v1)
        uv2 = v2 / np.linalg.norm(v2)
        dp = np.dot(uv1, uv2)

        return abs(dp) < 0.5


    def is_similar_edge(G, e1, e2):
        tags_e1 = G[e1[0]][e1[1]][0]
        tags_e2 = G[e2[0]][e2[1]][0]

        if not "name" in tags_e1 or not "name" in tags_e2:
            return False
        if tags_e1["name"] != tags_e2["name"]:
            return False
        if Expander.is_turn(G.nodes[e1[0]], G.nodes[e1[1]], G.nodes[e2[1]]):
            return False
        return True


    def find_next_edge(G, n1, n2, left_first):

        def turn_angle(n1, n2, n3):
            c1 = (G.nodes[n1]["x"], G.nodes[n1]["y"])
            c2 = (G.nodes[n2]["x"], G.nodes[n2]["y"])
            c3 = (G.nodes[n3]["x"], G.nodes[n3]["y"])
            b1 = ox.bearing.calculate_bearing(c2[0], c2[1], c1[0], c1[1])
            b2 = ox.bearing.calculate_bearing(c3[0], c3[1], c1[0], c1[1])
            a = b2 - b1
            if a < 0:
                a += 360
            return a

        other = [n for n in G[n2] if n != n1 and G[n2][n][0]["type"] == "unknown" and
                 Expander.is_similar_edge(G, [n1, n2], [n2, n])]
        if len(other) == 0:
            return None
        elif len(other) == 1:
            return other[0]
        else:
            sorted_other = sorted(other, key=lambda n: turn_angle(n1, n2, n), reverse=not left_first)
            return sorted_other[0]


    def extend_branch(G, n1, n2, left_first):
        # find next edge in the same street
        next = Expander.find_next_edge(G, n1, n2, left_first)
        # if not found, we reach the end of the path
        if next is None:
            return [n1, n2]
        else:
            # if found, we propagate the extension
            return [n1] + Expander.extend_branch(G, n2, next, left_first)
