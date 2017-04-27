import os
import xml.etree.ElementTree as ET

from PIL import Image

import numpy as np

import matplotlib
#this is for the magic that extracts the array out
matplotlib.use("agg") 

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.collections as mcollections

def xml2pgm(xml_data, height, width):
    name, _ = os.path.splitext(xml_data)
    with open(xml_data, 'rt') as f:
        tree = ET.parse(f)

    walls = []
    for wall in tree.getroot():
        points = wall.find("polygon").findall("point")
        wall = np.empty((len(points),2))
        for i, point in enumerate(points):
            wall[i] = [point.find("X").text, point.find("Y").text] 
        walls.append(wall)

    fig, ax = plt.subplots(figsize=(height, width))

    for wall in walls :
        ax.fill(wall[:,0], wall[:,1], color='k')

    ax.axis('off')
    fig.tight_layout(pad=0)

    fig.canvas.draw()
    tab = fig.canvas.copy_from_bbox(fig.bbox).to_string_argb()
    ncols, nrows = fig.canvas.get_width_height()
    data = np.fromstring(tab, dtype = np.uint8).reshape(nrows, ncols, 4)

    im = Image.fromarray(data)
    out_name = "{}.pgm".format(name)
    im.save(out_name)
    return out_name

if __name__ == '__main__':
    import sys
    # replace with code from gui that gets in data
    xml_data = 'simple_wall_geom.xml'
    #mess with figsize to figure out resolution
    height, width = (10,10)
    """above can be replaced with following to be cmdline script:
    import sys
    xml_data = sys.argv[1]
    height = sys.argv[2]
    width = sys.argv[3]
    """
    xml2pgm(xml_data, height, width)
