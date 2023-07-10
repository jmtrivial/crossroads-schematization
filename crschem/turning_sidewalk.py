from shapely.geometry import Point, LineString
import shapely.ops
from enum import Enum
import numpy as np
import geopandas

from . import utils as u

class TurningSidewalk:

    class TurnShape(Enum):
        BEVELED = 0
        STRAIGHT_ANGLE = 1
        ADJUSTED_ANGLE = 2

        def __str__(self):
                return self.name

        @staticmethod
        def from_string(s):
            try:
                return TurnShape[s]
            except KeyError:
                raise ValueError()
                
    class Point:
        def __init__(self, coord, curvPos = None):
            if isinstance(coord, Point):
                self.coord = (coord.x, coord.y)
            else:
                self.coord = coord
            self.curvPos = curvPos
        
        def set_curvilign_position(self, curvPos):
            self.curvPos = curvPos

    class FlexiblePoint(Point):
        def __init__(self, coord, curvPos = None):
            super().__init__(coord, curvPos)
            self.flexible = True

    class FixedPoint(Point):
        def __init__(self, coord, curvPos = None):
            super().__init__(coord, curvPos)
            self.flexible = False

    class CrossingPoint(FixedPoint):
        pass

        

    def __init__(self, id, str_sidewalks, crossings, 
                 osm_input, cr_input,
                 distance_kerb_footway, ignore_crossings_for_sidewalks,
                 turn_shape = TurnShape.ADJUSTED_ANGLE):
        self.id = id

        self.distance_kerb_footway = distance_kerb_footway
        self.turn_shape = turn_shape
        self.str_sidewalks = str_sidewalks

        self.crossings = crossings
        
        self.osm_input = osm_input
        self.cr_input = cr_input

        self.epsilon_for_merging = 2

        self.build_initial_turn()

        if not ignore_crossings_for_sidewalks:
            self.add_crossings()

        self.adjust_flexible_points()


    def build_initial_turn_basic(self):
        self.way = [TurningSidewalk.FixedPoint(c) for c in self.str_sidewalks[0].edge.coords][::-1] + [TurningSidewalk.FixedPoint(c) for c in self.str_sidewalks[1].edge.coords]


    def add_intersection_point_in_turn(self):
        if TurningSidewalk.is_before_end_of_edge(self.str_sidewalks[0].edge.coords, self.intersection) and \
            TurningSidewalk.is_before_end_of_edge(self.str_sidewalks[1].edge.coords, self.intersection):
            # if the intersection point is within the two end points
            
            if not TurningSidewalk.is_before_begin_of_edge(self.str_sidewalks[0].edge.coords, self.intersection):
                # if the intersection point is within the first edge
                self.way[1] = TurningSidewalk.FixedPoint(self.intersection)
                # if it is also inside the second edge, we remove the supplementary point
                if not TurningSidewalk.is_before_begin_of_edge(self.str_sidewalks[1].edge.coords, self.intersection):
                    del self.way[2]
            else:
                if not TurningSidewalk.is_before_begin_of_edge(self.str_sidewalks[1].edge.coords, self.intersection):
                    # if the intersection point is within the second edge
                    self.way[2] = TurningSidewalk.FixedPoint(self.intersection)
                else:
                    if self.turn_shape == TurningSidewalk.TurnShape.STRAIGHT_ANGLE:
                        # we add a fixed point
                        self.way.insert(2, TurningSidewalk.FixedPoint(self.intersection))
                    else:
                        # we add a flexible point
                        self.way.insert(2, TurningSidewalk.FlexiblePoint(self.intersection))


    def is_sidewalk_edge(self, node1, node2):
        tags = u.Utils.get_initial_edge_tags(self.cr_input, node1, node2, True)
        return tags != None and (tags["left_sidewalk"] == str(self.id) or tags["right_sidewalk"] == str(self.id))

    def find_next_point_on_original_path(self, path):
        last = path[-1]
        previous = path[-2]

        # for all neighbour of the last point
        for nb in self.osm_input[last]:
            if nb != previous and self.is_sidewalk_edge(nb, last):
                return nb
        
        print("Error: cannot found a point along the sidewalk")
        return None


    def compute_original_path(self):
        # find all edges of the original graph that are part of the sidewalk
        polybranch1 = self.str_sidewalks[0].get_polybranch()
        polybranch2 = self.str_sidewalks[1].get_polybranch()
        
        # initialize the final path with the first polybranch
        self.original_path = polybranch1[::-1]
        while self.original_path[-1] != polybranch2[0]:
            self.original_path.append(self.find_next_point_on_original_path(self.original_path))
            if self.original_path[-1] == None:
                # error: no continuity (should not be possible)
                self.original_path = None
                return
        
        # add the final part
        self.original_path += polybranch2[1:]
        self.original_path_linestring = [(self.osm_input.nodes[x]["x"], self.osm_input.nodes[x]["y"]) for x in self.original_path]


    def project_on_original_path(self, point):
        if isinstance(point, TurningSidewalk.Point):
            p = point.coord
        else:
            p = point
        nearest = shapely.ops.nearest_points(LineString(self.original_path_linestring), Point(p))
        return nearest[0]


    def estimate_curvilign_location_by_projection(self, point):
        proj = self.project_on_original_path(point)

        location = 0.0

        for x, y in zip(self.original_path_linestring, self.original_path_linestring[1:]):
            if u.Utils.is_in_edge(proj, x, y):
                location += u.Utils.edge_length(x, proj)
                break
            else:
                location += u.Utils.edge_length(x, y)

        return location


    def compute_curvilign_locations(self):
        for idp, current in enumerate(self.way):
            self.way[idp].set_curvilign_position(self.estimate_curvilign_location_by_projection(current))


    def build_initial_turn(self):
        # create turn with the 4 initial points
        self.build_initial_turn_basic()

        if self.turn_shape != TurningSidewalk.TurnShape.BEVELED:
            # compute middle point
            self.intersection = self.get_intersection()

            # create the initial turn
            if not self.intersection.is_empty:
                # if there is an intersection point
                self.add_intersection_point_in_turn()


        # compute the corresponding path in the OSM graph
        self.compute_original_path()

        # compute curvilign locations
        self.compute_curvilign_locations()

    def add_crossings(self):
        # for each sidewalk point
        for c in self.crossings:
            # identify its curvilign coordinate in the OSM path
            curvPos = self.estimate_curvilign_location_by_projection(c.get_location())
            # get location on the sidewalk
            location = c.get_location_on_sidewalk(self.id)
            # create the crossing point
            p = TurningSidewalk.CrossingPoint(location, curvPos)

            # find the good location along the way
            cid = 0
            while self.way[cid].curvPos < curvPos:
                cid += 1
                if cid >= len(self.way):
                    break
            
            if cid != 0 and abs(curvPos - self.way[cid - 1].curvPos) < self.epsilon_for_merging:
                # replace the existing node by the crossing
                self.way[cid - 1] = p
            elif cid < len(self.way) and abs(curvPos - self.way[cid].curvPos) < self.epsilon_for_merging:
                # replace the existing node by the crossing
                self.way[cid] = p
            else:
                # add it to the sidewalk
                self.way.insert(cid, p)


    def adjust_flexible_points(self):
        buffered_osm = u.Utils.get_buffered_osm(self.osm_input, self.distance_kerb_footway)

        for pred, point, next in zip(self.way, self.way[1:], self.way[2:]):
            if point.flexible:
                middle_sw = point.coord
                
                middle_bevel = Point([(x + y) / 2 for x, y in zip(pred.coord, next.coord)])
                # first check for basic intersection
                line = LineString([middle_sw, middle_bevel])

                if buffered_osm.intersects(line):
                    # build a more complex turn
                    edge = LineString((middle_bevel, middle_sw))
                    
                    elements = buffered_osm.boundary.intersection(edge)
                    if not elements.is_empty:
                        nearest = shapely.ops.nearest_points(middle_bevel, elements)
                        point.coord = (nearest[1].x, nearest[1].y)


    def branch_ids(self):
        return [x.description["id"] for x in self.str_sidewalks]
    

    def sidewalk_id(self):
        return self.str_sidewalks[0].sidewalk_id()


    def get_intersection(self):
        return self.str_sidewalks[0].get_intersection(self.str_sidewalks[1])


    def is_before_end_of_edge(edge, node):
        return u.Utils.norm_and_dot(u.Utils.vector(edge[0], edge[1]), u.Utils.vector(edge[1], node)) < 0


    def is_before_begin_of_edge(edge, node):
        return u.Utils.norm_and_dot(u.Utils.vector(edge[0], edge[1]), u.Utils.vector(edge[0], node)) < 0


    def as_array(self):
        return np.asarray([x.coord for x in self.way])


    def buffer(self, size):
        return self.way.buffer(size)


    def getGeometry(self):
        return LineString([x.coord for x in self.way])


    def getOSMIds(self):
        return ";".join([x.getOSMIds() for x in self.str_sidewalks])

    def toGDFSidewalks(sidewalks):
        d = {'type': [], 'osm_id': [], 'geometry': []}

        for s in sidewalks:
            d["type"].append("sidewalk")
            d["osm_id"].append(s.getOSMIds())
            d["geometry"].append(s.getGeometry())

        return geopandas.GeoDataFrame(d, crs=2154, geometry="geometry")
