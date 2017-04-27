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

# replace with code from gui that gets in data

xml_data = 'simple_wall_geom.xml'
name, _ = os.path.splitext(xml_data)

with open(xml_data, 'rt') as f:
    tree = ET.parse(f)

walls = []
for wall in tree.getroot():
    w = dict()
    w['ID'] = wall.find("ID").text
    w['type'] = wall.find("type").text
    w['height'] = wall.find("height").text 
    polygon = wall.find("polygon")
    points = polygon.findall("point")
    #array of points
    w['polygon'] = np.empty((len(points),3)) * np.nan
    for i, point in enumerate(points):
        w['polygon'][i] = [point.find(p).text
                           for p in ["X", "Y", "Z"]]
          
    walls.append(w)



#mess with figsize to figure out resolution
fig, ax = plt.subplots()
patches = []
#for wall_type in df['type'].unique():
#    walls = df[df['type'] == wall_type]
for xyz in df['polygon']:
    ax.fill(xyz[:,0], xyz[:,1], color='k')

ax.axis('off')
fig.tight_layout(pad=0)


fig.canvas.draw()
tab = fig.canvas.copy_from_bbox(fig.bbox).to_string_argb()
ncols, nrows = fig.canvas.get_width_height()
data = np.fromstring(tab, dtype = np.uint8).reshape(nrows, ncols, 4)


im = Image.fromarray(data)
im.save("{}.pgm".format(name))


