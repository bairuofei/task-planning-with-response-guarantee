#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 19 10:53:18 2020

@author: buffa
"""

from matplotlib import pyplot as plt
from matplotlib import colors

def grid_map(m: int, n: int, path: list, obs: list, tasks: list, servs: list, save_name = "path_fig.png"):
    
    # free, obstacles, tasks, service
    cmap = colors.ListedColormap(['white','dimgray','orangered', "limegreen", "lightseagreen"])
    data = [[0 for _ in range(n)] for _ in range(m)]
    Figure = plt.figure(figsize = (n, m))
    for x, y in obs:
        data[x][y] = 1
    for x, y, label in tasks:
        data[x][y] = 2
    for x, y, label in servs:
        data[x][y] = 3
    x_pos = [x + 0.4 for x in range(n)]
    y_pos = [y + 0.4 for y in range(m)]
    x_labels = [x for x in range(n)]
    y_labels = [y for y in range(m)]
    for i in range(len(path)):
        plt.xticks(x_pos, x_labels)
        plt.yticks(y_pos, y_labels)
        for x, y, label in tasks:
            plt.text(y+0.4, x+0.4, label)
        for x, y, label in servs:
            plt.text(y+0.4, x+0.4, label)
        x, y = path[i]
        data[x][y] = 4
        plt.pcolormesh(data,cmap=cmap,edgecolors='k', linewidths=1.5, alpha = 0.6)
        plt.show()
        plt.pause(0.3) # seconds
        if i != len(path)-1:
            Figure.clear()  
    plt.savefig(save_name)
    plt.pause(1.3)
#        plt.show()
#        plt.pause(1)
        
        
if __name__ == "__main__":
    m, n = 6, 12
    obs = [(2, 3), (3, 3), (4, 3)]
    tasks = [[3, 2, "t11"], [5, 9, "t12"]]
    servs = [[2, 5, "s1"]]
    
#    path = [(x, y) for x in range(m) for y in range(n)]
    path = [(0,1), (0, 2), (0, 3)]
    grid_map(m, n, path, obs, tasks, servs)