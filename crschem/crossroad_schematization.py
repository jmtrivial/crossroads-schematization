from shapely.geometry import Point, LineString, MultiLineString, LinearRing, Polygon
import osmnx
import os
import networkx
import numpy as np
import copy
import itertools
import geopandas
import pandas
import re
import matplotlib.pyplot as plt
import crseg.segmentation as cseg
import shutil
import mapnik
import mapnik.printing
from mapnik.printing.conversions import m2px
import sys
from osgeo import gdal, osr
import tempfile
from enum import Enum

from . import utils as u
from . import processing as p
from . import crossroad as c



class CrossroadSchematization:

    class Layout(Enum):
        A5_portrait = 0
        A5_landscape = 1
        A4_portrait = 2
        A4_landscape = 3
        
        def __str__(self):
                return self.name

        def width(self, margin = 0.01):
            if self == CrossroadSchematization.Layout.A5_landscape or self == CrossroadSchematization.Layout.A4_portrait:
                return 0.21 - margin * 2
            elif self == CrossroadSchematization.Layout.A5_portrait:
                return 0.1485 - margin * 2
            else: # self == Layout.A4_landscape
                return 0.297 - margin * 2

        def height(self, margin = 0.01):
            if self == CrossroadSchematization.Layout.A5_portrait or self == CrossroadSchematization.Layout.A4_landscape:
                return 0.21 - margin * 2
            elif self == CrossroadSchematization.Layout.A5_landscape:
                return 0.1485 - margin * 2
            else: # self == Layout.A4_portrait
                return 0.297 - margin * 2

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
        'crossing:island',
        'foot'
    ]

    # If the OSM data has been previously loaded, do not load it again
    def __init__(self, cr_input, 
                 osm_oriented = None,
                 osm_unoriented = None,
                 ignore_crossings_for_sidewalks = False,
                 use_fixed_width_on_branches = False,
                 turn_shape = c.TurningSidewalk.TurnShape.ADJUSTED_ANGLE,
                 osm_buffer_size_meters = 200, 
                 distance_kerb_footway = 0.5,
                 white_space_meter = 1.5):
        self.osm_buffer_size_meters = osm_buffer_size_meters
        self.distance_kerb_footway = distance_kerb_footway
        self.white_space_meter = white_space_meter
        self.cr_input = cr_input
        self.ignore_crossings_for_sidewalks = ignore_crossings_for_sidewalks
        self.use_fixed_width_on_branches = use_fixed_width_on_branches
        self.turn_shape = turn_shape

        self.load_osm(osm_oriented, osm_unoriented)

        # get crossroad center
        is_n = cr_input["type"] == "crossroads"
        self.center = cr_input[is_n]["geometry"][0]



    def build(latitude, longitude,
              C0, C1, C2,
              ignore_crossings_for_sidewalks = False,
              use_fixed_width_on_branches = False,
              turn_shape = c.TurningSidewalk.TurnShape.ADJUSTED_ANGLE,
              verbose = True,
              ignore_cache = False,
              overpass = False,
              log_files = False):

        import crseg.segmentation as cseg
        import crseg.utils as cru
        import crmodel.crmodel as cm
        import osmnx as ox
        from copy import deepcopy
        import os

        # load data from OSM
        if verbose:
            print("Loading data from OpenStreetMap")
        ox.settings.use_cache = not ignore_cache
        ox.settings.useful_tags_node = list(set(ox.settings.useful_tags_node + CrossroadSchematization.node_tags_to_keep))
        G_init = cru.Util.get_osm_data(latitude, longitude, 200, overpass)#, ["cycleway", "cycleway:right", "cycleway:left", "psv"])

        # segment intersection(from https://github.com/jmtrivial/crossroads-segmentation)
        if verbose:
            print("Segmenting intersection")
        # remove sidewalks, cycleways, service ways
        G = cseg.Segmentation.prepare_network(deepcopy(G_init))
        # build an undirected version of the graph
        undirected_G = ox.utils_graph.get_undirected(G)
        
        # segment it using topology and semantic
        seg = cseg.Segmentation(undirected_G, C0 = C0, C1 = C1, C2 = C2, max_cycle_elements = 10)
        seg.process()

        tmp1 = tempfile.NamedTemporaryFile(mode='w', delete=False)
        seg.to_json(tmp1.name, longitude, latitude)
        # convert it as a model (from https://gitlab.limos.fr/jeremyk6/crossroads-description)
        print("Converting graph as a model")

        model = cm.CrModel()
        model.computeModel(G, tmp1.name)

        if log_files:
            print("Segmentation:", tmp1.name)
        else:
            os.unlink(tmp1.name)

        # save this model as a temporary file
        tmp2 = tempfile.NamedTemporaryFile(mode='w', delete=False)
        with tmp2 as fp:
            content = model.getGeoJSON()
            fp.write(content)
            fp.close()

        cr_input = geopandas.read_file(tmp2.name)

        if log_files:
            print("Model:", tmp2.name)
        else:
            os.unlink(tmp2.name)

        return CrossroadSchematization(cr_input, G_init, 
                                        ignore_crossings_for_sidewalks=ignore_crossings_for_sidewalks, 
                                        use_fixed_width_on_branches=use_fixed_width_on_branches,
                                        turn_shape=turn_shape)

    def is_valid_model(self):
        for index, elem in self.cr_input.iterrows():
            if elem["type"] in ["branch", "way"]:                
                for side in ["left", "right"]:
                    for obj in ["_island", "_sidewalk"]:
                        key = side + obj
                        if not (isinstance(elem[key], float) or elem[key] is None):
                            print(key, "=", elem[key])
                            return False


            
        return True


    def process(self):
        self.label_osm_from_input()
        
        # grouping ways by branch
        print("Creating branches")
        self.build_branches()

        # compute for each branch two long edges *S1* and *S2* corresponding to the sidewalks:
        print("Creating sidewalks")
        self.build_sidewalks()

        # add pedestrian crossings
        print("Creating crossings")
        self.crossings = c.Crossing.create_crossings(self.osm_input, self.cr_input, 
                                                     self.osm_input_oriented,
                                                     self.distance_kerb_footway)

        # assemble sidewalks
        print("Assembling sidewalks")
        self.assemble_sidewalks()

        # compute inner region 
        print("Computing inner region")
        self.build_inner_region()

        # filtering crossings
        print("Filtering crossings")
        self.filter_crossings()
        # build traffic islands
        print("Building traffic islands")
        self.build_traffic_islands()

        print("Computing traffic island shape")
        # compute traffic island shape
        for island in self.traffic_islands:
            island.compute_generalization(self.crossings, self.inner_region)


    def filter_crossings(self):
        def function_is_inside(pair):
            return pair[1].is_inside(self.inner_region)
        self.crossings = dict(filter(function_is_inside, self.crossings.items()))

    def load_osm(self, osm_oriented, osm_unoriented):
        # load OSM data from the same crossroad (osmnx:graph)
        bounds = self.cr_input.total_bounds
        center = [(bounds[1] + bounds[3]) / 2, (bounds[0] + bounds[2]) / 2]

        if osm_oriented is None:
            print("Loading OpenStreetMap data " + str(center))
            osmnx.settings.use_cache = True
            osmnx.settings.useful_tags_node = list(set(osmnx.settings.useful_tags_node + CrossroadSchematization.node_tags_to_keep))
            self.osm_input_oriented = osmnx.graph.graph_from_point(center, 
                                                                   self.osm_buffer_size_meters, 
                                                                   network_type="all", 
                                                                   retain_all=False, 
                                                                   truncate_by_edge=True, 
                                                                   simplify=False)
        else:
            self.osm_input_oriented = cseg.Segmentation.prepare_network(copy.deepcopy(osm_oriented), remove_footways=False)

        # project to Lambert93 (France) for a metric approximation
        self.osm_input_oriented = osmnx.projection.project_graph(self.osm_input_oriented, to_crs = "EPSG:2154")

        if osm_unoriented is None:
            # convert to undirected graph
            self.osm_input = osmnx.utils_graph.get_undirected(self.osm_input_oriented)
        else:
            self.osm_input = osm_unoriented


    def label_osm_from_input(self):
        # label edges of the graph from cr_input
        print("Label OSM network")
        networkx.set_edge_attributes(self.osm_input, values="unknown", name="type")
        networkx.set_edge_attributes(self.osm_input, values="created", name="type_origin")
        networkx.set_node_attributes(self.osm_input, values="unknown", name="type")
        for index, elem in self.cr_input.iterrows():
            if elem["type"] in ["branch", "way"]:
                ids = list(map(int, elem["osm_node_ids"]))
                self.osm_input[ids[0]][ids[1]][0]["type"] = elem["type"]
                self.osm_input[ids[0]][ids[1]][0]["type_origin"] = "input"
                self.osm_input.nodes[ids[0]]["type"] = "input"
                self.osm_input.nodes[ids[1]]["type"] = "input"


    def is_boundary_node(self, node):
        for n in self.osm_input[node]:
            if self.osm_input[node][n][0]["type"] == "way":
                return True
        return False


    def build_branches(self):
        print("Grouping ways by branch")
        self.branches = {}

        bid = 0
        for index, elem in self.cr_input.iterrows():
            if elem["type"] == "branch":
                ids = list(map(int, elem["osm_node_ids"]))
                osm_n1 = ids[0] # first id in the OSM direction
                osm_n2 = ids[1] # last id in the OSM direction
                n1 = osm_n1 if self.is_boundary_node(osm_n1) else osm_n2
                n2 = osm_n2 if n1 == osm_n1 else osm_n1
                e = u.Utils.get_initial_edge_tags(self.cr_input, osm_n1, osm_n2)
                if e is not None:
                    id = e["id"]
                    bname = e["name"]
                    if not id in self.branches:
                        self.branches[id] = c.Branch(bname, id, self.osm_input, self.cr_input, self.distance_kerb_footway)
                    self.branches[id].add_way(c.SimpleWay(n1, n2, e, osm_n1 == n1))


    def build_sidewalks(self):
        self.sidewalks = {}
        
        for bid in self.branches:
            self.sidewalks[bid] = self.branches[bid].get_sidewalks(self.use_fixed_width_on_branches)

    
    def get_sidewalk_ids(self):
        result = set()
        for bid in self.sidewalks:
            if self.sidewalks[bid]:
                for sw in self.sidewalks[bid]:
                    result.add(sw.sidewalk_id())
        return list(result)


    def get_sidewalks_by_id(self, sid):
        result = []
        for bid in self.sidewalks:
            if self.sidewalks[bid]:
                for sw in self.sidewalks[bid]:
                    if sw.sidewalk_id() == sid:
                        result.append(sw)
        return result

    def get_crossings_by_sidewalks_ids(self, sid):
        result = []
        for cid in self.crossings:
            if str(sid) in self.crossings[cid].get_sidewalk_ids():
                result.append(self.crossings[cid])
        return result


    def assemble_sidewalks(self):
        self.cr_input.replace('', np.nan, inplace=True)
        original_sidewalks_ids = self.get_sidewalk_ids()
        self.merged_sidewalks = []

        # TODO: find crossings that should be part of the sidewalks and
        # integrate them to the final shape

        for sid in original_sidewalks_ids:
            self.merged_sidewalks.append(c.TurningSidewalk(sid,
                                                            self.get_sidewalks_by_id(sid), 
                                                            self.get_crossings_by_sidewalks_ids(sid),
                                                            self.osm_input, self.cr_input, self.distance_kerb_footway,
                                                            self.ignore_crossings_for_sidewalks,
                                                            self.turn_shape))


    def build_inner_region(self):
        open_sides = copy.copy(self.merged_sidewalks)

        # order sidewalks
        final_shape = [(open_sides.pop(), True)]
        while len(open_sides) != 0:
            cid = final_shape[-1][0].branch_ids()[1 if final_shape[-1][1] else 0]
            found = False
            for i, o in enumerate(open_sides):
                if o.branch_ids()[0] == cid:
                    final_shape.append((open_sides.pop(i), True))
                    found = True
                    break
                elif o.branch_ids()[1] == cid:
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


    def build_traffic_islands(self):
        traffic_islands_edges = {}

        # first group edges by island id
        for index, elem in self.cr_input.iterrows():
            if elem["type"] in ["branch", "way"]:
                for side in ["left", "right"]:
                    id = u.Utils.get_number_from_label(elem[side + "_island"])
                    if not id is None:
                        if not id in traffic_islands_edges:
                            traffic_islands_edges[id] = []
                        traffic_islands_edges[id].append(";".join(elem["osm_node_ids"]))
        
        # then build traffic islands
        self.traffic_islands = []
        for eid in traffic_islands_edges:
            self.traffic_islands.append(c.TrafficIsland(traffic_islands_edges[eid], self.osm_input, self.cr_input))


    def to_printable_internal(self, filename, log_files, dpi = -1, crs = 3857):
        from qgis.core import QgsApplication, QgsProject, QgsPrintLayout, QgsLayout, QgsVectorLayer, QgsLayoutExporter, QgsLayoutItemPage, QgsReadWriteContext, QgsRectangle, QgsCoordinateReferenceSystem, QgsCoordinateTransform, QgsFeatureRequest, QgsExpression
        from qgis.PyQt.QtXml import QDomDocument
        import tempfile
        import os

        tmp = tempfile.NamedTemporaryFile(mode='w', delete=False) 
        self.toGeojson(tmp.name, True)
        qgs = QgsApplication([], False)
        qgs.initQgis()
        
        # get project
        project = QgsProject.instance()
        composition = QgsPrintLayout(project)
        project.setCrs(QgsCoordinateReferenceSystem(crs))
        
        # load layers
        geojson = tmp.name
        points_crossings_layer = QgsVectorLayer(geojson + '|geometrytype=Point', "points", "ogr")
        points_islands_layer = QgsVectorLayer(geojson + '|geometrytype=Point', "points", "ogr")
        points_space_layer = QgsVectorLayer(geojson + '|geometrytype=Point', "points", "ogr")
        lines_layer = QgsVectorLayer(geojson + '|geometrytype=LineString', "lines", "ogr")
        lines_space_layer = QgsVectorLayer(geojson + '|geometrytype=LineString', "lines", "ogr")
        polygons_layer = QgsVectorLayer(geojson + '|geometrytype=Polygon', "polygons", "ogr") # 4326
        # project them on map
        project.addMapLayer(polygons_layer)
        project.addMapLayer(lines_space_layer)
        project.addMapLayer(points_space_layer)
        project.addMapLayer(points_crossings_layer)
        project.addMapLayer(points_islands_layer)
        project.addMapLayer(lines_layer)

        # compute region of interest
        polygons_layer.selectAll() # the polygon corresponding to the car ways
        pg_box = polygons_layer.boundingBoxOfSelected()
        points_space_layer.selectByRect(pg_box) # only keep points inside (crossings, islands)
        pt_box = points_space_layer.boundingBoxOfSelected()
        # then combine both rectangles to zoom on the points within the main region
        pp = 2
        pb = 1
        sourceCrs = QgsCoordinateReferenceSystem(polygons_layer.crs())
        destCrs = QgsCoordinateReferenceSystem(project.crs())
        tr = QgsCoordinateTransform(sourceCrs, destCrs, project)

        box = QgsRectangle((pt_box.xMinimum() * pp + pg_box.xMinimum() * pb) / (pp + pb),
                           (pt_box.yMinimum() * pp + pg_box.yMinimum() * pb) / (pp + pb),
                           (pt_box.xMaximum() * pp + pg_box.xMaximum() * pb) / (pp + pb),
                           (pt_box.yMaximum() * pp + pg_box.yMaximum() * pb) / (pp + pb))
        box = tr.transform(box)

        # load layout
        layout = QgsLayout(project)
        layout.initializeDefaults()
        layout.pageCollection().page(0).setPageSize('A5', QgsLayoutItemPage.Orientation.Landscape)
        template_file = open(os.path.dirname(__file__) + "/resources/tactile-a5.qpt")
        template_content = template_file.read()
        template_file.close()
        document = QDomDocument()
        document.setContent(template_content)
        items, ok = layout.loadFromTemplate(document, QgsReadWriteContext(), False)
        for i in items:
            if i.id() == "Carte 1":
                # zoom on the intersection
                i.zoomToExtent(box)
                # if the zoom is more than 1:400, we come back to this scale
                if i.scale() < 400:
                    i.setScale(400)
        # to solve the SVG relative path in qml files, use a trick [BEGIN]
        filename = os.path.abspath(filename)
        cwd = os.getcwd()
        os.chdir(os.path.dirname(__file__))

        # load layer styles and assign them to the layers
        points_islands_style = os.path.dirname(__file__) + "/resources/rendering-nodes-islands.qml" # TODO: integrate them for pipe
        points_crossings_style = os.path.dirname(__file__) + "/resources/rendering-nodes-crossings.qml"
        points_space_style = os.path.dirname(__file__) + "/resources/rendering-nodes-space.qml"
        lines_style = os.path.dirname(__file__) + "/resources/rendering-polylines.qml"
        polygons_style = os.path.dirname(__file__) + "/resources/rendering-areas.qml" # sld
        lines_space_style = os.path.dirname(__file__) + "/resources/rendering-polylines-space.qml"
        
        lines_layer.loadNamedStyle(lines_style)
        points_space_layer.loadNamedStyle(points_space_style)
        points_crossings_layer.loadNamedStyle(points_crossings_style)
        points_islands_layer.loadNamedStyle(points_islands_style)
        lines_space_layer.loadNamedStyle(lines_space_style)
        polygons_layer.loadNamedStyle(polygons_style)
        
        exporter = QgsLayoutExporter(layout)
        settings = exporter.PdfExportSettings()
        settings.rasterizeWholeImage = False
        if filename.endswith(".pdf"):
            exporter.exportToPdf(filename, settings)
        else:
            settings = QgsLayoutExporter.ImageExportSettings()
            if dpi > 0:
                settings.dpi = dpi
            exporter.exportToImage(filename, settings)
        
        project.clear()
        qgs.exit()
        # trick [END]: go back to the initial directory
        os.chdir(cwd)
        
        # delete temporary file if not required
        if log_files:
            print("geojson:", tmp.name)
        else:
            os.unlink(tmp.name)



    def toPdf(self, filename, log_files = False):
        self.to_printable_internal(filename, log_files)


    def toTifInternal(self, dirName, filename, log_files, resolution, scale, layout, marginCM):
        widthMeter = layout.width(marginCM / 100)
        heightMeter = layout.height(marginCM / 100)

        # scale (ie 1cm in the map is scale "scale" * 1 cm in reality)
        scale = 400

        width = int(m2px(widthMeter, resolution))
        height = int(m2px(heightMeter, resolution))

        mapfile = dirName + "/style-" + str(resolution) + ".xml"
        output = filename

        pseudo_mercator = mapnik.Projection('+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +no_defs +over')
        mercator = mapnik.Projection('+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs')
        trans = mapnik.ProjTransform(mercator, pseudo_mercator)


        # make a new Map object for the given mapfile
        m = mapnik.Map(width, height)
        mapnik.load_map(m, mapfile)

        # ensure the target map projection is pseudo-mercator
        m.srs = pseudo_mercator.params()

        # get crossroads center

        pmerc_centre = trans.forward(mapnik.Coord(self.center.x, self.center.y))

        # compute min and max coordinates
        dx = widthMeter / 2 * scale
        minx = pmerc_centre.x - dx
        maxx = pmerc_centre.x + dx

        # grow the height bbox, as we only accurately set the width bbox
        m.aspect_fix_mode = mapnik.aspect_fix_mode.ADJUST_BBOX_HEIGHT

        bounds = mapnik.Box2d(minx, pmerc_centre.y - 10, maxx, pmerc_centre.y + 10) # the y bounds will be fixed by mapnik due to ADJUST_BBOX_HEIGHT
        m.zoom_to_box(bounds)

        # render the map image to a file
        mapnik.render_to_file(m, output)

        # set geotiff information
        gdal.UseExceptions()
        pxSize = 1 / m2px(1, resolution) * scale
        ds = gdal.Open(output, gdal.GA_Update)
        gt = [
            #GT(0) x-coordinate of the upper-left corner of the upper-left pixel.
            m.envelope()[0],
            #GT(1) w-e pixel resolution / pixel width.
            pxSize,
            #GT(2) row rotation (typically zero).
            0.0,
            #GT(3) y-coordinate of the upper-left corner of the upper-left pixel.
            m.envelope()[3],
            #GT(4) column rotation (typically zero).
            0.0,
            #GT(5) n-s pixel resolution / pixel height (negative value for a north-up image).
            -pxSize
        ]
        ds.SetGeoTransform(gt)

        sr = osr.SpatialReference()
        sr.SetFromUserInput(pseudo_mercator.params())
        wkt = sr.ExportToWkt()
        ds.SetProjection(wkt)



    def toTif(self, filename, log_files = False, resolution = 300, scale = 400, layout=Layout.A5_portrait, margin=1):
        # first export to shapefiles in a temporary directory
        dirName = tempfile.mkdtemp()
        if log_files:
            print('Temporary directory (styling):', dirName)
        self.toShapefiles(dirName + "/crossroad.shp")

        # then move style file (xml) in this directory
        if resolution in [96, 300]:
            for f in ["style-" + str(resolution) + ".xml",
                        "crossing-3-" + str(resolution) + ".svg", 
                        "point-" + str(resolution) + ".svg",
                        "island-" + str(resolution) + ".svg",
                        "island-" + str(resolution) + "-white.svg"]:
                shutil.copy(os.path.dirname(__file__) + "/resources/" + f, dirName)
        else:
            print("not supported DPI")
            return

        # finally render the image
        self.toTifInternal(dirName, filename, log_files, resolution, scale, layout, margin)

        # then delete the temporary directory
        if not log_files:
            shutil.rmtree(dirName)
        

    def toSvg(self, filename, only_reachable_islands = False):
        # TODO
        print("not yet implemented")


    def toGDFInnerRegion(self):
        d = {'type': ['inner_region'], 'geometry': [self.inner_region]}
        return geopandas.GeoDataFrame(d, crs=2154)


    def toGeojson(self, filename, only_reachable_islands = False, crs = "EPSG:4326"):
        df = pandas.concat([self.toGDFInnerRegion().to_crs(crs),
                            c.TurningSidewalk.toGDFSidewalks(self.merged_sidewalks).to_crs(crs),
                            c.Branch.toGDFBranches(self.branches).to_crs(crs),
                            c.TrafficIsland.toGDFTrafficIslands(self.traffic_islands, only_reachable_islands).to_crs(crs),
                            c.Crossing.toGDFCrossings(self.crossings).to_crs(crs)])
        
        df.to_file(filename, driver='GeoJSON')


    def toShapefiles(self, filename, only_reachable_islands = False, crs = "EPSG:4326"):
        filename, file_extension = os.path.splitext(filename)

        self.toGDFInnerRegion().to_crs(crs).to_file(filename + "-inner" + file_extension) # region
        c.TurningSidewalk.toGDFSidewalks(self.merged_sidewalks).to_crs(crs).to_file(filename + "-sidewalks" + file_extension) # lines
        c.Branch.toGDFBranches(self.branches).to_crs(crs).to_file(filename + "-branches" + file_extension) # lines
        
        # islands can be points and lines
        islands = c.TrafficIsland.toGDFTrafficIslands(self.traffic_islands, only_reachable_islands).to_crs(crs)
        islands[islands.geometry.type == 'LineString'].to_file(filename + "-islands-lines" + file_extension)
        islands[islands.geometry.type == 'Point'].to_file(filename + "-islands-points" + file_extension)

        # points
        c.Crossing.toGDFCrossings(self.crossings).to_crs(crs).to_file(filename + "-crossings" + file_extension)


    def show(self, 
             osm_graph = False,
             branches = False,
             simple_sidewalks = False,
             merged_sidewalks = True,
             inner_region = True,
             exact_islands = False,
             crossings = True,
             islands = True,
             only_reachable_islands = True):
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


        if branches:
            for geom in self.branches:
                for ee in self.branches[geom].sides:
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
                x = [p.coord[0] for p in sw.way]
                y = [p.coord[1] for p in sw.way]
                plt.plot(x, y, color = colors[sw.sidewalk_id() % len(colors)], linewidth=3)

        if exact_islands:
            for i, sw in enumerate(self.traffic_islands):
                if sw.is_reachable or not only_reachable_islands:
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
                if sw.is_reachable or not only_reachable_islands:
                    if len(sw.extremities) == 0:
                        x, y = sw.center
                        plt.plot(x, y, "ok", markersize=12, linewidth=12)
                    else:
                        for e in sw.extremities:
                            x = [sw.center[0], e[0]]
                            y = [sw.center[1], e[1]]
                            plt.plot(x, y, color="black", solid_capstyle='round', markersize=12, linewidth=12)

                    


        plt.show()
