# Crossroads schematization

## Pipeline

First compute for each branch two long edges *S1* and *S2* corresponding to the sidewalks:

* for each edge part of a branch, identify the corresponding polyline (continue a couple of 10 meters outside of the crossing)
* fit a long edge *E* on this polyline, starting from the beginning of the polyline (e.g. 50 meters for example), and with a fixed length in the exterior direction)
* estimate the width of the way (using the number of lanes, the classification of the way)
* reconstruct a linear description of the sidewalk by shifting *E* 
* if the branch is composed of more than one way, identify the two adjacent sidewalks and make them parallel

Each sidewalk is part of two branches (see crossroads-description by Jérémy Kalsron), thus is described by two long edges *[Sa_i, Sa_e]* and *[Sa_i, Sa_e]* (*e* for exterior, *i* for interior).

* we compute the intersection $m$ between *Sa* and *Sb*, and build a new representation with a polyline made of 3 points: *Sa_e*, *m*, *Sb_e*
* If a one of the original segments of the crossing intersects *Sa-m-Sb*, the sidewalk is described by a polyline made of 4 points: *Sa_e*, *Sa_i*, *Sb_i*, *Sb_e*

The inner part of the crossroad is computed by assembling all the sidewalks as a closed polyline (assembling each polyline at they extremity). From this region, we can apply a negative buffer to obtain the inner part of the road (with a white space between sidewalks and inner part of the road).

Each traffic island is available as a polygon from crossroads-description. We compute its compactness and size. A description is produced depending on these parameters:

* if the traffic island is large, the rendering is done as if it was bordered by sidewalks
* if the traffic island is small:
  * if the shape is compact, we describe it by a disc
  * if the shape is not compact, we describe it by two half discs connected by lines (a long shape)
  
Each pedestrian crossing is described by a dedicated pictogram, aligned with the corresponding edge from the initial data

## Open questions

* the final rendering is an svg. How to create pictograms and draw them from geodata?

## Possible tools

* OSMnx: 
  * shapely
    * object.parallel_offset
    * object.simplify
    * line.interpolate
  * ```osmnx.utils_graph.graph_to_gdfs```

* convert to svg:
  * http://kuanbutts.com/2018/09/06/geodataframe-to-svg-2/
  * https://svgis.readthedocs.io/en/stable/

