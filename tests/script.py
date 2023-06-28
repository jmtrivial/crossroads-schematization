import mapnik
import mapnik.printing
from mapnik.printing.conversions import m2px
import sys
from osgeo import gdal, osr

# inspired from https://gist.github.com/andrewharvey/1290744/e855eeb1c29f7d5ff54c779b2abeeecf9fb62136

# resolution
resolution = 300

# printed size (in meter)
widthMeter = 0.2
heightMeter = 0.14

# scale (ie 1cm in the map is scale "scale" * 1 cm in reality)
scale = 400

width = int(m2px(widthMeter, resolution))
height = int(m2px(heightMeter, resolution))

mapfile = "ballainvilliers-" + str(resolution) + ".xml"
output = "ballainvilliers.tif"

merc = mapnik.Projection('+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +no_defs +over')

 # make a new Map object for the given mapfile
m = mapnik.Map(width, height)
mapnik.load_map(m, mapfile)

# ensure the target map projection is mercator
m.srs = merc.params()

# transform the centre point into the target coord sys
m.zoom_all()
merc_centre = m.envelope().center()

# compute min and max coordinates
dx = widthMeter / 2 * scale
minx = merc_centre.x - dx
maxx = merc_centre.x + dx

# grow the height bbox, as we only accurately set the width bbox
m.aspect_fix_mode = mapnik.aspect_fix_mode.ADJUST_BBOX_HEIGHT

bounds = mapnik.Box2d(minx, merc_centre.y - 10, maxx, merc_centre.y + 10) # the y bounds will be fixed by mapnik due to ADJUST_BBOX_HEIGHT
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
sr.SetFromUserInput(merc.params())
wkt = sr.ExportToWkt()
ds.SetProjection(wkt)


    

sys.stdout.write('output image to %s!\n' % output)

