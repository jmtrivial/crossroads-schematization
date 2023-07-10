import numpy as np
from shapely.geometry import Point, LineString
import shapely.ops

from .. import utils as u
from .. import processing as p

from .simple_way import SimpleWay

class StraightWay(SimpleWay):
    
    # parameters (and attributes)
    # - a simpleway (defined by nodes n1, n2)
    # - polybranch: an extended path from n1, n2
    def __init__(self, sw, polybranch, G):
        super().__init__(sw.n1, sw.n2, sw.edge_tags, sw.same_osm_orientation)
        self.polybranch = polybranch
        self.edge = None
        self.array = None
        self.G = G
        self.lz = p.Linearization()
        self.maximal_removal = 20 # meters


    def build_from_simpleway(sw, G, left_first):
        result = StraightWay(sw, p.Expander.extend_branch(G, sw.n1, sw.n2, left_first), G) 
        result.compute_linear_edge(G)
        return result


    def compute_linear_edge(self, G):
        self.consolidated_polybranch = p.Expander.remove_non_straight_parts(G, self.polybranch, self.maximal_removal)
        self.edge = self.lz.process(p.Expander.convert_to_linestring(G, self.consolidated_polybranch))
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


    def point(self, i):
        return Point(self.array[i])


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


    def get_projection_on_polybranch(self, point):
        line = [(self.G.nodes[x]["x"], self.G.nodes[x]["y"]) for x in self.polybranch]
        nearest = shapely.ops.nearest_points(LineString(line), Point(point))
        pt = nearest[0]

        edges = []
        for e1, e2 in zip(self.polybranch, self.polybranch[1:]):
            if u.Utils.is_in_edge(pt, self.G.nodes[e1], self.G.nodes[e2]):
                edges.append((e1, e2))

        if len(edges) == 0:
            return pt, None
        elif len(edges) == 1:
            return pt, edges[0]
        else:
            # find the edge with the largest estimated width
            ewidths = [(e, u.Utils.evaluate_width_way(self.G[e[0]][e[1]][0])) for e in edges]
            return pt, max(ewidths, key=lambda x: x[1])[0]
        