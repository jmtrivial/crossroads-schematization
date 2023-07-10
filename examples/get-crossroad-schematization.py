#!/usr/bin/env python3
#encoding: utf-8

import argparse
import geopandas

import sys

import crschem.crossroad_schematization as cs
from crschem.model.turning_sidewalk import TurningSidewalk

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
group_preprocess.add_argument('--similar-direction-angle', help='Maximum angle for lanes to be considered inside the same branch (in degree). Default: 60.', type=int, default=60)

group_process = parser.add_argument_group("Processing", "Parameters of the processing")
group_process.add_argument('--keep-doubled-crossings', help='In case of double crossings (current bad configuration in OSM data with traffic lights), keep both nodes.', action='store_true')
group_process.add_argument('--ignore-crossings-for-sidewalks', help='Do not use crossings to shape the sidewalks', action='store_true')
group_process.add_argument('--use-fixed-width-on-branches', help='Use a fixed width on each branch (do not evaluate the width adjustment)', action='store_true')
group_process.add_argument('--turn-shape', help='Turn shape.', type=lambda s: TurningSidewalk.TurnShape[s], choices=list(TurningSidewalk.TurnShape))

group_output = parser.add_argument_group("Output", "Display, log or save results")
group_output.add_argument('-l', '--log-files', help='keep intermediate files and give their name in output', action='store_true')
group_output.add_argument('-d', '--display-all', help='display all steps', action='store_true')
group_output.add_argument('--display-preview', help='display a preview of the crossroad schematization', action='store_true')
group_output.add_argument('-o', '--output', help='output file (supported format: geojson, pdf, tif, shp)', type=FileOpener('w'))
group_output.add_argument('--scale', help='Scale of the map. Default: 400 (for 1:400)', type=int, default=400, choices=[400, 500])
group_output.add_argument('--dpi', help='dpi for tif export', type=int, choices=[96, 300], default=300)
group_output.add_argument('--layout', help='Map layout.', type=lambda s: cs.CrossroadSchematization.Layout[s], choices=list(cs.CrossroadSchematization.Layout), default = cs.CrossroadSchematization.Layout.A5_landscape)
group_output.add_argument('--margin', help='Margin in cm. Default: 1.0cm', type=float, default=1)

group_preview = parser.add_argument_group("Preview options", "Parameters used by the preview display")
group_preview.add_argument('--osm', help='display OpenStreetMap network', action='store_true')
group_preview.add_argument('--branches', help='display branches', action='store_true')
group_preview.add_argument('--sidewalks-on-branches', help='display sidewalks only on branches', action='store_true')
group_preview.add_argument('--exact-islands', help='display exact shape of the islands', action='store_true')
group_preview.add_argument("--non-reachable-islands", help="display non reachable islands.", action='store_true')

args = parser.parse_args()

try:

    if args.input_model:
        input_file = args.input_model.filename
        G_init = None
        print("Loading input geojson (" + input_file + ")")
        crschem = cs.CrossroadSchematization(cr_input, G_init)
    else:
        latitude = args.by_coordinates[0]
        longitude = args.by_coordinates[1]
        crschem = cs.CrossroadSchematization.build(latitude, longitude,
                                                   args.c0, args.c1, args.c2,
                                                   similar_direction_angle = args.similar_direction_angle,
                                                   verbose = True,
                                                   ignore_crossings_for_sidewalks = args.ignore_crossings_for_sidewalks,
                                                   use_fixed_width_on_branches = args.use_fixed_width_on_branches,
                                                   turn_shape = args.turn_shape,
                                                   remove_doubled_crossings = not args.keep_doubled_crossings,
                                                   ignore_cache = args.ignore_cache,
                                                   overpass = args.overpass,
                                                   log_files = args.log_files)


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
            crschem.toPdf(args.output.filename, args.log_files, resolution=args.dpi, layout=args.layout, margin=args.margin, scale=args.scale, only_reachable_islands=not args.non_reachable_islands)
        elif args.output.filename.endswith(".tif"):
            print("Exporting as tif:", args.output.filename)
            crschem.toTif(args.output.filename, args.log_files, resolution=args.dpi, layout=args.layout, margin=args.margin, scale=args.scale, only_reachable_islands=not args.non_reachable_islands)
        elif args.output.filename.endswith(".svg"):
            print("Exporting as svg:", args.output.filename)
            crschem.toSvg(args.output.filename, args.log_files, resolution=args.dpi, layout=args.layout, margin=args.margin, scale=args.scale, only_reachable_islands=not args.non_reachable_islands)
        elif args.output.filename.endswith(".geojson"):
            print("Exporting as geojson:", args.output.filename)
            crschem.toGeojson(args.output.filename, not args.non_reachable_islands)
        elif args.output.filename.endswith(".shp"):
            print("Exporting as shapefile:", args.output.filename)
            crschem.toShapefiles(args.output.filename, not args.non_reachable_islands)
        else:
            print("Unknown output format")
            

except ValueError as e:
    print("Error:", e)
    print("Intermediate files:", input_file)
