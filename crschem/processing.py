from shapely.geometry import Point, LineString, MultiLineString, LinearRing, Polygon
import numpy as np
from numpy import linalg


class Linearization:
    
    def __init__(self, length = 30):
        self.exponential_coef = 1.2
        self.length = length
    

    def process(self, polyline):
        #polyline = convert_to_linestring(G, polybranch)

        # discretize the polybranch following a density depending on the linear coordinate
        polydisbranch = self.discretize_polyline(polyline)
        polydisbranchcoords = np.asarray(polydisbranch.coords)

        # compute a PCA and deduce a direction line
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
        return LineString([polyline.interpolate(x) for x in self.exponential_coordinates(1, min(polyline.length, self.length))])


    def compute_direction_line(self, polyline):
        x = [a[0] for a in polyline]
        y = [a[1] for a in polyline]
        center = Point(sum(x) / len(x), sum(y) / len(y))
        start = polyline[0]
        v = [center.x - start[0], center.y - start[1]]
        v = v / linalg.norm(v)
        return LineString([start, start + v * self.length])


    def project_on_line(point, line):
        return line.interpolate(line.project(point))


class Expander:

    def __init__(self):
        self.bid = 0


    def reset_bid(self):
        self.bid = 0


    def process(self, G, n1, n2):
        result = self.bid, self.extend_branch(G, n1, n2)
        self.bid += 1
        return result


    def process_to_linestring(self, G, n1, n2):
        bid = self.bid
        polyline = self.process(G, n1, n2)[1]
        return bid, polyline[0], Expander.convert_to_linestring(G, polyline)


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


    def find_next_edge(G, n1, n2):

        for n3 in G[n2]:
            if n3 != n1 and G[n2][n3][0]["type"] == "unknown":
                if Expander.is_similar_edge(G, [n1, n2], [n2, n3]):
                    return n3

        return None


    def extend_branch(self, G, n1, n2, first = True):
        G[n1][n2][0]["branch_id"] = self.bid

        # find next edge in the same street
        n3 = Expander.find_next_edge(G, n1, n2)
        # if not found, we reverse the edge and try to build an extension from the other side
        if n3 == None:
            if first:
                return self.extend_branch(G, n2, n1, False)
            else:
                return [n1, n2]
        else:
            # if found, we propagate the extension
            G[n2][n3][0]["type"] = G[n1][n2][0]["type"]
            return [n1] + self.extend_branch(G, n2, n3, False)
