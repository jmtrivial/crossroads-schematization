from shapely.geometry import Point, LineString
import math
import numpy as np
from numpy import linalg


from .. import utils as u

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
        return u.Utils.extends_edge(self.edge.coords, length)


    # compute the intersection between the two straight sidewalk lines (considering it as infinite lines)
    def get_intersection(self, sw):
        # extend both LineString
        l1 = self.extends()
        l2 = sw.extends()

        # compute intersection between them
        return l1.intersection(l2)


    def getOSMIds(self):
        return ";".join(self.description["osm_node_ids"])

