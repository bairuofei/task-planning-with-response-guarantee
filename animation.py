#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 19 16:59:44 2020

@author: buffa
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.colors as mcolors
import matplotlib.collections as mcoll

fig, ax = plt.subplots()
cmap = mcolors.ListedColormap(['white', 'black'])
bounds = [-0.5, 0.5, 1.5]
norm = mcolors.BoundaryNorm(bounds, cmap.N)
data = np.random.rand(10, 10) * 2 - 0.5
im = ax.imshow(data, cmap=cmap, norm=norm)

grid = np.arange(-0.5, 11, 1)
xmin, xmax, ymin, ymax = -0.5, 10.5, -0.5, 10.5
lines = ([[(x, y) for y in (ymin, ymax)] for x in grid]
         + [[(x, y) for x in (xmin, xmax)] for y in grid])
grid = mcoll.LineCollection(lines, linestyles='solid', linewidths=2,
                            color='teal')
ax.add_collection(grid)

def animate(i):
    data = np.random.rand(10, 10) * 2 - 0.5
    im.set_data(data)
    # return a list of the artists that need to be redrawn
    return [im, grid]

anim = animation.FuncAnimation(
    fig, animate, frames=200, interval=1000, blit=True, repeat=True)
plt.show()