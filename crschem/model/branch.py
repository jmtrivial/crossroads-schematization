from shapely.geometry import Point, LineString

import geopandas


from .. import utils as u
from .. import processing as p

from .simple_way import SimpleWay
from .straight_way import StraightWay
from .straight_sidewalk import StraightSidewalk


class Branch:

    def __init__(self, name, id, osm_input, cr_input, distance_kerb_footway):
        self.ways = []
        self.name = name
        self.id = id
        self.osm_input = osm_input
        self.cr_input = cr_input
        self.distance_kerb_footway = distance_kerb_footway

    
    def add_way(self, way):
        self.ways.append(way)

    
    def nbWays(self):
        return len(self.ways)


    # return all the edges contained in the initial intersection
    # that are not part of this branch
    def get_other_edges(self):
        result = []
        for index, elem in self.cr_input.iterrows():
            if elem["type"] in ["branch", "way"] and elem["name"] != self.name:
                ids = list(map(int, elem["osm_node_ids"]))
                result.append(ids)
        return result


    def shift_middle_line(self, shifts, direction):
        edges = [self.middle_line.parallel_offset(s, direction) for s in shifts]
        return LineString([edges[0].coords[0], edges[1].coords[1]])

    def build_two_sidewalks(self, use_fixed_width_on_branches):
        from statistics import mean
        # the shifts corresponds to half the widths of the street
        shifts = [x / 2 for x in self.widths]

        if use_fixed_width_on_branches:
            shifts = [mean(shifts) for x in shifts]
        
        # compute the two lines (one in each side)
        result = [StraightSidewalk(self.shift_middle_line(shifts, "left"),
                                   self.sides[0],
                                   "left"),
                   StraightSidewalk(self.shift_middle_line(shifts, "right"),
                                   self.sides[1],
                                   "right")]

        # shift them if required
        buf = u.Utils.get_edges_buffered_by_osm(self.get_other_edges(), self.osm_input, self.distance_kerb_footway).boundary
        for i, s in enumerate(result):
            if s.edge.intersects(buf):
                intersections = s.edge.intersection(buf)
                if not intersections.is_empty and isinstance(intersections, Point):
                    result[i].update_first_node(intersections)

        self.sidewalks = result



    # maximum distance between two extremity points of the ways
    def get_initial_branche_width(self):
        edges = []
        distance = 0

        for w in self.sides:
            osm = [self.osm_input.nodes[int(x)] for x in w.consolidated_polybranch[:2]] # use the first two elements
            emeters = LineString([(x["x"], x["y"]) for x in osm])
            if len(edges) != 0:
                for ee in edges:
                    d = ee.distance(emeters)
                    if d > distance:
                        distance = d
            edges.append(emeters)

        return distance


    def build_middle_way(self):
        self.middle_line = StraightWay.build_middle_line(self.sides[0], self.sides[1])
            
    
    def compute_widths(self):
        # for each extremity of the middle line
        self.widths = []
        for p in self.middle_line.coords:

            # project it on each polybranches and select two furthest points
            p1, e1 = self.sides[0].get_projection_on_polybranch(p)
            p2, e2 = self.sides[1].get_projection_on_polybranch(p)

            # for each point estimate the width of the way
            interdistance = u.Utils.edge_length(p1, p2)
            w1 = u.Utils.evaluate_width_way(self.osm_input[e1[0]][e1[1]][0]) / 2 + self.distance_kerb_footway
            w2 = u.Utils.evaluate_width_way(self.osm_input[e2[0]][e2[1]][0]) / 2 + self.distance_kerb_footway

            # compute the final width
            self.widths.append(interdistance + w1 + w2)


    def build_sidewalk_straightways(self):
        # get the external simple ways (they are bordered by a sidewalk)
        self.simple_sides = [ w for w in self.ways if w.has_sidewalk()]

        if len(self.simple_sides) > 2:
            print("ERROR: more than two ways with a sidewalk (", self.name, ")")
            return

        self.single_side = False
        # if only one way, we duplicate it
        if len(self.simple_sides) == 1:
            if self.simple_sides[0].has_sidewalks_both_sides():
                self.simple_sides.append(self.simple_sides[0])
            else:
                print("ERROR: only one way in the branch, but with missing sidewalks (", self.name, ")")
                return
        else:
            # order them according to the id of the sidewalk
            self.simple_sides = sorted(self.simple_sides, key=lambda s: int(s.get_sidewalk_id()))
            # and flip in case we are at a branch sharing first and last sidewalks (by id)
            if int(self.simple_sides[1].get_sidewalk_id()) > int(self.simple_sides[0].get_sidewalk_id()) + 1:
                self.simple_sides = [self.simple_sides[1], self.simple_sides[0]]

        # build their extension as straightline
        self.sides = [StraightWay.build_from_simpleway(self.simple_sides[0], self.osm_input, True), # always choose the left
                       StraightWay.build_from_simpleway(self.simple_sides[1], self.osm_input, False)] # always choose the right



    def get_sidewalks(self, use_fixed_width_on_branches):

        self.build_sidewalk_straightways()

        # TODO: shift each extremity of each sidewalk wrt the estimated width of each extremity

        self.build_middle_way()

        self.compute_widths()

        self.build_two_sidewalks(use_fixed_width_on_branches)

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
