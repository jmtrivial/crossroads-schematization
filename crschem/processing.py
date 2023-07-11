from shapely.geometry import Point, LineString, MultiLineString, LinearRing, Polygon
import shapely.ops
import numpy as np
from numpy import linalg

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

    # return true if the point p2 (middle point of pts=[p1, p2, p3])
    # is a splitting node, or an node with a strong angle
    def is_split_in_straight_part(G, pts):
        p1, p2, p3 = pts
        if len(G[p2]) > 2:
            return True
        angle = u.Utils.turn_angle(G, p2, p1, p3)
        if angle > 180:
            angle = 360 - angle 
        if abs(angle) > 30:
            return True
        
        return False


    def process(self, G, n1, n2, left_first):
        result = Expander.extend_branch(G, n1, n2, left_first)
        self.bid += 1
        return self.bid, result

    # remove first edges of the polyline if they are before a splitting node
    # or before a node with a strong angle, and within a maximal distance (threshold)
    # from the initial point
    def remove_non_straight_parts(G, polyline, threshold):
        if len(polyline) <= 2:
            return polyline

        # identify if each point is a split (thanks to the angle or its arity)
        middle_points = zip(polyline, polyline[1:], polyline[2:])
        is_split = [False] + [Expander.is_split_in_straight_part(G, p) for p in middle_points] + [False]

        # use threshold to filter these possible splits
        distances = [0] + [u.Utils.edge_length([G.nodes[a]["x"], G.nodes[a]["y"]], [G.nodes[b]["x"], G.nodes[b]["y"]]) for a, b in zip(polyline, polyline[1:])]
        cumuld_dists = np.cumsum(distances)
        is_split = [i and d < threshold for i, d in zip(is_split, cumuld_dists)]

        try:
            last = len(is_split) - is_split[::-1].index(True) - 1
            return polyline[last:]
        except:
            return polyline


    def convert_to_linestring(G, polyline):
        return LineString([Point(G.nodes[x]["x"], G.nodes[x]["y"]) for x in polyline])


    def is_turn(G, m, c1, c2):
        ta = u.Utils.turn_angle(G, m, c1, c2)
        return ta < 90 or ta > 90 * 3


    def is_similar_edge(G, e1, e2):
        tags_e1 = G[e1[0]][e1[1]][0]
        tags_e2 = G[e2[0]][e2[1]][0]

        if not "name" in tags_e1 or not "name" in tags_e2:
            return False
        if tags_e1["name"] != tags_e2["name"]:
            return False
        if Expander.is_turn(G, e1[1], e1[0], e2[1]):
            return False
        return True


    def find_next_edge_simple(G, n1, n2):
        other = [n for n in G[n2] if n != n1 and G[n2][n][0]["type"] == "unknown" and
                 Expander.is_similar_edge(G, [n1, n2], [n2, n])]
        if len(other) == 1:
            return other[0]
        else:
            return None

    def find_next_edge(G, n1, n2, left_first):

        other = [n for n in G[n2] if n != n1 and G[n2][n][0]["type"] == "unknown" and
                 Expander.is_similar_edge(G, [n1, n2], [n2, n])]
        if len(other) == 0:
            return None
        elif len(other) == 1:
            return other[0]
        else:
            sorted_other = sorted(other, key=lambda n: u.Utils.turn_angle(G, n2, n1, n), reverse=not left_first)
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



    def find_next_edge_on_polygon(G, n1, n2, left_first):
        other = [n for n in G[n2] if n != n1 and G[n2][n][0]["type"] == "unknown"]
        if len(other) == 0:
            return None
        elif len(other) == 1:
            return other[0]
        else:
            sorted_other = sorted(other, key=lambda n: u.Utils.turn_angle(G, n2, n1, n), reverse=not left_first)
            return sorted_other[0]


    def extend_polygon(G, path, left_first):

        next = Expander.find_next_edge_on_polygon(G, path[-2], path[-1], left_first)
        # if not found, we reach the end of a path
        if next is None:
            return path
        else:
            np = path + [next]
            if next in path:
                return np
            else:
                return Expander.extend_polygon(G, np, left_first)

    def close_polygon(G, path):
        p1 = Expander.extend_polygon(G, path, True)
        if p1[0] == p1[-1]:
            return p1
        else:
            p2 = Expander.extend_polygon(G, path, False)
            if p2[0] == p2[-1]:
                return p2
            else:
                # when a part of the polygon is outside of the map, choose the best option between one side 
                # and the other
                p1 = p1[::-1]
                p1 = Expander.extend_polygon(G, p1, False)
                d1 = u.Utils.edge_length([G.nodes[p1[0]]["x"], G.nodes[p1[0]]["y"]], [G.nodes[p1[-1]]["x"], G.nodes[p1[-1]]["y"]])
                p2 = p2[::-1]
                p2 = Expander.extend_polygon(G, p2, False)
                d2 = u.Utils.edge_length([G.nodes[p2[0]]["x"], G.nodes[p2[0]]["y"]], [G.nodes[p2[-1]]["x"], G.nodes[p2[-1]]["y"]])
                if d1 < d2:
                    return p1
                else:
                    return p2
