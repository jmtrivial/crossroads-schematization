#!/usr/bin/env python3
#encoding: utf-8

import argparse
import geopandas
import tempfile
import osmnx as ox
from copy import deepcopy
import os
import sys

import crseg.segmentation as cseg
import crseg.utils as cru
import crmodel.crmodel as cm
import crschem.crossroad_schematization as cs

# a trick to avoid the creation of files given as parameters
class FileOpener(argparse.FileType):
    # delayed FileType;
    # sample use:
    # with args.input.open() as f: f.read()
    def __call__(self, string):
        # optionally test string
        self.filename = string
        return self
    def open(self):
        return super(FileOpener,self).__call__(self.filename)
    file =  property(open, None, None, 'open file property')



parser = argparse.ArgumentParser(description="Generate a schematized representation of a given crossroad.")

group_input = parser.add_argument_group('Input', "Define the input from OSM (by coordinates or by name) or from a computed model")

input_params = group_input.add_mutually_exclusive_group(required=True)
input_params.add_argument('-i', '--input-model', help='input geojson file generated by crossroads describer', type=FileOpener('r'))
input_params.add_argument('-c', '--by-coordinates', nargs=2, help='Load input from OSM using the given latitude, apply segmentation then model generation', type=float)

group_input.add_argument('--overpass', help='Use Overpass to download data instead of the OSM api', action='store_true')
group_input.add_argument('--ignore-cache', help='Ignore local cache', action='store_true')

group_preprocess = parser.add_argument_group('Preprocessing', "Parameters of the preprocesses (crseg, crdesc)")
group_preprocess.add_argument('--c0', help='Initial intersection size (distance between boundaries and middle of the initial intersection). Default: 2.', type=float, default=2)
group_preprocess.add_argument('--c1', help='Intersection size (aggregation by adjacency). Default: 2.', type=float, default=2)
group_preprocess.add_argument('--c2', help='Intersection size (aggregation by cycle detection). Default: 4.', type=float, default=4)


group_output = parser.add_argument_group("Output", "Display, log or save results")
group_output.add_argument('-l', '--log-files', help='keep intermediate files and give their name in output', action='store_true')
group_output.add_argument('-d', '--display-all', help='display all steps', action='store_true')
group_output.add_argument('--display-preview', help='display a preview of the crossroad schematization', action='store_true')
group_output.add_argument('-o', '--output', help='output file (supported format: geojson, pdf, tif)', type=FileOpener('w'))

group_preview = parser.add_argument_group("Preview options", "Parameters used by the preview display")
group_preview.add_argument('--osm', help='display OpenStreetMap network', action='store_true')
group_preview.add_argument('--branches', help='display branches', action='store_true')
group_preview.add_argument('--sidewalks-on-branches', help='display sidewalks only on branches', action='store_true')
group_preview.add_argument('--exact-islands', help='display exact shape of the islands', action='store_true')
group_preview.add_argument("--non-reachable-islands", help="display non reachable islands.", action='store_true')

args = parser.parse_args()


input_file = ""
if args.input_model:
    input_file = args.input_model.filename
    G_init = None
    print("Loading input geojson (" + input_file + ")")
else:
    latitude = args.by_coordinates[0]
    longitude = args.by_coordinates[1]

    # load data from OSM
    print("Loading data from OpenStreetMap")
    ox.settings.use_cache = not args.ignore_cache
    ox.settings.useful_tags_node = list(set(ox.settings.useful_tags_node + cs.CrossroadSchematization.node_tags_to_keep))
    G_init = cru.Util.get_osm_data(latitude, longitude, 200, args.overpass)#, ["cycleway", "cycleway:right", "cycleway:left", "psv"])

    # segment intersection(from https://github.com/jmtrivial/crossroads-segmentation)
    print("Segmenting intersection")
    # remove sidewalks, cycleways, service ways
    G = cseg.Segmentation.prepare_network(deepcopy(G_init))
    # build an undirected version of the graph
    undirected_G = ox.utils_graph.get_undirected(G)
    
    # segment it using topology and semantic
    seg = cseg.Segmentation(undirected_G, C0 = args.c0, C1 = args.c1, C2 = args.c2, max_cycle_elements = 10)
    seg.process()

    if args.display_all:
        print("Displaying segmentation")
        cr = seg.get_crossroad(longitude, latitude)
        ec = seg.get_regions_colors_from_crossroad(cr)
        nc = seg.get_nodes_regions_colors_from_crossroad(cr)
        ox.plot.plot_graph(undirected_G, edge_color=ec, node_color=nc)

    tmp1 = tempfile.NamedTemporaryFile(mode='w', delete=False)
    seg.to_json(tmp1.name, longitude, latitude)
    if args.log_files:
        print("Segmentation:", tmp1.name)


    # convert it as a model (from https://gitlab.limos.fr/jeremyk6/crossroads-description)
    print("Converting graph as a model")

    model = cm.CrModel()
    model.computeModel(G, tmp1.name)

    # save this model as a temporary file
    tmp2 = tempfile.NamedTemporaryFile(mode='w', delete=False)
    with tmp2 as fp:
        content = model.getGeoJSON()
        fp.write(content)
        fp.close()

    input_file = tmp2.name

    if args.log_files:
        print("Model:", tmp2.name)


# load geojson data from Jérémy's tool
cr_input = geopandas.read_file(input_file)

try:
    crschem = cs.CrossroadSchematization(cr_input, G_init)

    '''if not crschem.is_valid_model():
        print("Error: the model is not valid")
        exit(1)'''

    crschem.process()

    if args.display_preview or args.display_all:
        crschem.show(only_reachable_islands=not args.non_reachable_islands, osm_graph=args.osm,
                     branches=args.branches,
                     simple_sidewalks=args.sidewalks_on_branches, merged_sidewalks=not args.sidewalks_on_branches,
                     exact_islands=args.exact_islands)

    if args.output:
        if len(args.output.filename) < 4:
            "Cannot deduce required format with small file names"
            
    if args.output:
    
        if args.output.filename.endswith(".pdf"):
            print("Exporting as pdf:", args.output.filename)
            crschem.toPdf(args.output.filename, args.log_files)
        elif args.output.filename.endswith(".tif"):
            print("Exporting as tif:", args.output.filename)
            crschem.toTif(args.output.filename, args.log_files)
        elif args.output.filename.endswith(".svg"):
            print("Exporting as svg:", args.output.filename)
            crschem.toSvg(args.output.filename, args.non_reachable_islands)
        elif args.output.filename.endswith(".geojson"):
            print("Exporting as geojson:", args.output.filename)
            crschem.toGeojson(args.output.filename, args.non_reachable_islands)
        else:
            print("Unknown output format")
            

except ValueError as e:
    print("Error:", e)
    print("Intermediate files:", input_file)
