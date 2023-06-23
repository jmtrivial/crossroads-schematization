import mapnik
import sys

# inspired from https://gist.github.com/andrewharvey/1290744/e855eeb1c29f7d5ff54c779b2abeeecf9fb62136



width = 2048
height = 2048
zoom = 21 # TODO

mapfile = "ballainvilliers.xml"
output = "ballainvilliers.png"

merc = mapnik.Projection('+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +no_defs +over')


 # make a new Map object for the given mapfile
m = mapnik.Map(width, height)
mapnik.load_map(m, mapfile)

# ensure the target map projection is mercator
m.srs = merc.params()

# transform the centre point into the target coord sys
m.zoom_all()
merc_centre = m.envelope().center()
print(merc_centre)

# 360/(2**zoom) degrees = 256 px
# so in merc 1px = (20037508.34*2) / (256 * 2**zoom)
# hence to find the bounds of our rectangle in projected coordinates + and - half the image width worth of projected coord units
dx = ((20037508.34*2*(width/2)))/(256*(2 ** (zoom)))
minx = merc_centre.x - dx
maxx = merc_centre.x + dx
print(minx, maxx)

# grow the height bbox, as we only accurately set the width bbox
m.aspect_fix_mode = mapnik.aspect_fix_mode.ADJUST_BBOX_HEIGHT

bounds = mapnik.Box2d(minx, merc_centre.y - 10, maxx, merc_centre.y + 10) # the y bounds will be fixed by mapnik due to ADJUST_BBOX_HEIGHT
m.zoom_to_box(bounds)

# render the map image to a file
mapnik.render_to_file(m, output)
    
sys.stdout.write('output image to %s!\n' % output)

