from shapely.geometry import Point, LineString, MultiLineString, LinearRing, Polygon
import osmnx
import networkx
import numpy as np
import copy
import itertools
import geopandas
import re
import matplotlib.pyplot as plt


from . import utils as u
from . import processing as p
from . import crossroad as c

class CrossroadSchematization:

    node_tags_to_keep = [
        # general informations
        'highway',
        # crosswalk informations
        'crossing',
        'tactile_paving',
        # traffic signals informations
        'traffic_signals',
        'traffic_signals:direction',
        'traffic_signals:sound',
        'button_operated'
        #sidewalk informations
        'kerb',
        #island informations
        'crossing:island'
    ]

    def __init__(self, cr_input, 
                 osm_buffer_size_meters = 200, 
                 distance_kerb_footway = 0.5,
                 white_space_meter = 1.5,
                 osm_buffer_for_bevel = 1):
        self.osm_buffer_size_meters = osm_buffer_size_meters
        self.distance_kerb_footway = distance_kerb_footway
        self.white_space_meter = white_space_meter
        self.osm_buffer_for_bevel = osm_buffer_for_bevel
        self.cr_input = cr_input

        self.load_osm()
        self.label_osm_from_input()

        # TODO: if two ways are connected in the exterior side, then consider the next edge as the branch edge
        self.extend_ways()
        
        # pairing branch ways
        self.build_branches()

        # compute for each branch two long edges *S1* and *S2* corresponding to the sidewalks:
        self.build_sidewalks()

        # assemble sidewalks
        self.assemble_sidewalks()

        # compute inner region 
        self.build_inner_region()

        # add a white space
        self.add_white_space_inner_region()

        # build traffic islands
        self.build_traffic_islands()

        # add pedestrian crossings
        print("Creating crossings")
        self.crossings = c.Crossing.create_crossings(self.osm_input, self.cr_input)

        # compute traffic island shape
        for island in self.traffic_islands:
            island.compute_generalization(self.crossings)

        # add a supplementary point on the sidewalks where a crossing is reaching it
        # to preserve the estimated distance
        # TODO


    def load_osm(self):
        # load OSM data from the same crossroad (osmnx:graph)
        bounds = self.cr_input.total_bounds
        center = [(bounds[1] + bounds[3]) / 2, (bounds[0] + bounds[2]) / 2]

        print("Loading OpenStreetMap data " + str(center))
        osmnx.settings.use_cache = True
        osmnx.settings.useful_tags_node = list(set(osmnx.settings.useful_tags_node + CrossroadSchematization.node_tags_to_keep))
        osm_input_init = osmnx.graph.graph_from_point(center, 
                                                      self.osm_buffer_size_meters, 
                                                      network_type="all", 
                                                      retain_all=False, 
                                                      truncate_by_edge=True, 
                                                      simplify=False)
        # project to Lambert93 (France) for a metric approximation
        self.osm_input_oriented = osmnx.projection.project_graph(osm_input_init, to_crs = "EPSG:2154")
        # convert to undirected graph
        self.osm_input = osmnx.utils_graph.get_undirected(self.osm_input_oriented)


    def label_osm_from_input(self):
        # label edges of the graph from cr_input
        print("Label OSM network")
        networkx.set_edge_attributes(self.osm_input, values="unknown", name="type")
        networkx.set_edge_attributes(self.osm_input, values="created", name="type_origin")
        networkx.set_edge_attributes(self.osm_input, values=-1, name="branch_id")
        networkx.set_node_attributes(self.osm_input, values="unknown", name="type")
        for index, elem in self.cr_input.iterrows():
            if elem["type"] in ["branch", "way"]:
                ids = list(map(int, elem["id"].split(";")))
                self.osm_input[ids[0]][ids[1]][0]["type"] = elem["type"]
                self.osm_input[ids[0]][ids[1]][0]["type_origin"] = "input"
                self.osm_input.nodes[ids[0]]["type"] = "input"
                self.osm_input.nodes[ids[1]]["type"] = "input"


    def extend_ways(self):
        lz = p.Linearization()
        e = p.Expander()
        print("Extending branches")

        # compute for each way[type=branch] its extension
        # and fit two long edges on each polyline
        self.linear_ways = {}
        for n1 in self.osm_input:
            for n2 in self.osm_input[n1]:
                if n1 > n2 and self.osm_input[n1][n2][0]["type"] == "branch" and self.osm_input[n1][n2][0]["type_origin"] == "input":
                    bid, first_id, polybranch = e.process_to_linestring(self.osm_input, n1, n2)
                    self.linear_ways[bid] = c.StraightWay(lz.process(polybranch), first_id, n1, n2, self.cr_input)


    def build_branches(self):
        print("Pairing ways by branch")
        self.branches = {}

        for wid in self.linear_ways:
            w = self.linear_ways[wid]
            e = w.edge_tags
            if e is not None and (e["left_sidewalk"] != "" or e["right_sidewalk"] != ""):
                bname = e["name"]
                if not bname in self.branches:
                    self.branches[bname] = c.Branch(bname, self.osm_input, self.cr_input, self.distance_kerb_footway)
                self.branches[bname].add_way(w)


    def build_sidewalks(self):
        print("Building sidewalks")
        self.sidewalks = {}
        
        for bid in self.branches:
            self.sidewalks[bid] = self.branches[bid].get_sidewalks()

    
    def get_sidewalk_ids(self):
        result = set()
        for bid in self.sidewalks:
            for sw in self.sidewalks[bid]:
                result.add(sw.sidewalk_id())
        return list(result)


    def get_sidewalks_by_id(self, sid):
        result = []

        for bid in self.sidewalks:
            for sw in self.sidewalks[bid]:
                if sw.sidewalk_id() == sid:
                    result.append(sw)
        
        return result

    def assemble_sidewalks(self):
        print("Assembling sidewalks")
        self.cr_input.replace('', np.nan, inplace=True)
        original_sidewalks_ids = self.get_sidewalk_ids()
        
        self.merged_sidewalks = []

        for sid in original_sidewalks_ids:
            self.merged_sidewalks.append(c.TurningSidewalk(self.get_sidewalks_by_id(sid), self.osm_input, self.osm_buffer_for_bevel))


    def build_inner_region(self):
        print("Building inner region")
        open_sides = copy.copy(self.merged_sidewalks)

        # order sidewalks
        final_shape = [(open_sides.pop(), True)]
        while len(open_sides) != 0:
            cid = final_shape[-1][0].branch_names()[1 if final_shape[-1][1] else 0]
            found = False
            for i, o in enumerate(open_sides):
                if o.branch_names()[0] == cid:
                    final_shape.append((open_sides.pop(i), True))
                    found = True
                    break
                elif o.branch_names()[1] == cid:
                    final_shape.append((open_sides.pop(i), False))
                    found = True
                    break
            if not found:
                print("Error: cannot found next sidewalk")
                return
        
        # flatten list and make it as a ring
        final_shape = [x[0].as_array() if x[1] else x[0].as_array()[::-1] for x in final_shape]
        final_shape = list(itertools.chain(*[list(x) for x in final_shape]))
        final_shape.append(final_shape[0])

        self.inner_region = Polygon(final_shape)


    def add_white_space_inner_region(self):
        print("Adding white space")
        
        # compute a buffer arround each sidewalk
        # and remove these regions from the inner region
        for p in [Polygon(x.buffer(self.white_space_meter)) for x in self.merged_sidewalks]:
            self.inner_region = self.inner_region.difference(p)


    def build_traffic_islands(self):
        print("Building traffic island")
        traffic_islands_edges = {}

        # first group edges by island id
        for index, elem in self.cr_input.iterrows():
            if elem["type"] in ["branch", "way"]:                
                for side in ["left", "right"]:
                    if not isinstance(elem[side + "_island"], float):
                        id = int(elem[side + "_island"])
                        if not id in traffic_islands_edges:
                            traffic_islands_edges[id] = []
                        traffic_islands_edges[id].append(elem["id"])
                
        # then build traffic islands
        self.traffic_islands = []
        for eid in traffic_islands_edges:
            self.traffic_islands.append(c.TrafficIsland(traffic_islands_edges[eid], self.osm_input))


    def toSvg(self, filename):
        # TODO
        print("not yet implemented")


    def toGDFInnerRegion(self):
        d = {'type': ['inner_region'], 'geometry': [self.inner_region]}
        return geopandas.GeoDataFrame(d, crs=2154)


    def toGeojson(self, filename):
        df = self.toGDFInnerRegion()
        df = df.append(c.TurningSidewalk.toGDFSidewalks(self.merged_sidewalks))
        df = df.append(c.Branch.toGDFBranches(self.branches))
        df = df.append(c.TrafficIsland.toGDFTrafficIslands(self.traffic_islands))
        df = df.append(c.Crossing.toGDFCrossings(self.crossings))
        
        df.to_file(filename, driver='GeoJSON')


    def show(self, 
             osm_graph = False,
             linear_ways = False,
             branches = False,
             simple_sidewalks = False,
             merged_sidewalks = True,
             inner_region = True,
             exact_islands = False,
             crossings = True,
             islands = True):
        colors = [ 'r', 'y', 'b', 'g', "orange", 'purple', 'b']

        if inner_region:
            p = geopandas.GeoSeries(self.inner_region)
            p.plot(facecolor="#DDDDDD")

        if osm_graph:
            for n1 in self.osm_input:
                for n2 in self.osm_input[n1]:
                    if u.Utils.is_roadway_edge(self.osm_input[n1][n2][0]):
                        p1 = self.osm_input.nodes[n1]
                        p2 = self.osm_input.nodes[n2]
                        plt.plot([p1["x"], p2["x"]], [p1["y"], p2["y"]], color = "grey")

        if linear_ways:
            for geom in self.linear_ways:
                x, y = self.linear_ways[geom].edge.xy
                plt.plot(x, y, color = "gray")
                plt.plot(x[0],y[0],'ok')

        if branches:
            for geom in self.branches:
                for ee in self.branches[geom].ways:
                    x, y = ee.edge.xy
                    plt.plot(x, y, color = "black")
                    plt.plot(x[0],y[0],'ok')

        if simple_sidewalks:
            for sid in self.sidewalks:
                for sw in self.sidewalks[sid]:
                    x, y = sw.edge.xy

                    plt.plot(x, y, color = colors[sw.sidewalk_id() % len(colors)])
                    plt.plot(x[0],y[0],'ok')

        if merged_sidewalks:
            for sw in self.merged_sidewalks:
                x, y = sw.way.xy
                plt.plot(x, y, color = colors[sw.sidewalk_id() % len(colors)], linewidth=3)

        if exact_islands:
            for i, sw in enumerate(self.traffic_islands):
                x, y = sw.get_linearring().xy
                plt.plot(x, y, color = colors[i % len(colors)], linewidth=1)

        if crossings:
            for c in self.crossings:
                xy = self.crossings[c].get_line_representation()
                x = [e[0] for e in xy]
                y = [e[1] for e in xy]
                plt.plot(x, y, color = "black", linewidth=2)
                plt.plot(x[1], y[1], "ok")

        if islands:
            for sw in self.traffic_islands:
                x, y = sw.center
                plt.plot(x, y, "ok", markersize=12, linewidth=12)

        plt.show()
