#!/usr/bin/python3

import sys
import tempfile
import numpy as np

N = 500
XLIM = 42
YLIM = 42

TREE_MODEL_URI = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine Tree'

filename = tempfile.mkstemp()[-1]

if len(sys.argv) > 1:
    filename = sys.argv[1]

print(f'Output will be written to {filename}')


xcoords = np.random.uniform(-XLIM, XLIM, N)
ycoords = np.random.uniform(-YLIM, YLIM, N)
yaws = np.random.uniform(-np.pi, np.pi, N)

trees = '''
    <!-- trees {-->'''

for i in range(N):
    x = xcoords[i]
    y = ycoords[i]
    yaw = yaws[i]
    raw_text = f'''
    <include>
        <uri>{TREE_MODEL_URI}</uri>
        <name>tree_pine{i}</name>
        <pose>{x} {y} 0 0 0 {yaw}</pose>
    </include>
    '''

    trees += raw_text

trees += '<!--}-->'

with open(filename, 'w', encoding='utf-8') as f:
    f.write(trees)

print(f'Trees generated to {filename}, copy the content into your world file')
