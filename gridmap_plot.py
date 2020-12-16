#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 19 10:53:18 2020

@author: buffa
"""
# from matplotlib.collections import QuadMesh
from matplotlib import pyplot as plt
from matplotlib import colors
from collections import deque

def grid_map(m: int, n: int, path: list, obs: list, tasks: list, servs: list, save_name = "path_fig.png"):
    # path_list is a list of paths
    # free, obstacles, tasks, service
    cmap = colors.ListedColormap(['white','dimgray','orangered', "limegreen", "lightseagreen"])
    data = [[0 for _ in range(n)] for _ in range(m)]
    fig, ax = plt.subplots(figsize = (n, m))
    plt.axis('equal')
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
    ax.set_xticks(x_pos)
    ax.set_yticks(y_pos, y_labels)
    for x, y, label in tasks:
        ax.text(y+0.4, x+0.4, label)
    for x, y, label in servs:
        ax.text(y+0.4, x+0.4, label)
    for i in range(len(path)):
        x, y = path[i]
        if data[x][y] == 0:
            data[x][y] = 4 # 修改路径颜色
        _ = ax.pcolormesh(data, cmap=cmap, edgecolors='k', linewidths=1.5, alpha = 0.6) # as one collection
        # ax.axis('equal')
        plt.show()
        plt.pause(0.3) # seconds
        if i != len(path) - 1:
            del ax.collections[-1]
        
    fig.savefig(save_name)
    plt.pause(1.1)
    plt.close()
#        plt.show()
#        plt.pause(1)
        
    
def animate_path(m: int, n: int, path_list: list, alloc: list, task_cap: dict, obs: list, tasks: list, servs: list, save_name = "path_fig.png"):
    servs_dic = {}
    for x, y, label in servs:
        servs_dic[(x, y)] = label
        
    nrobot = len(path_list)
    new_alloc = [deque(x) for x in alloc]
    color_record = ['white','dimgray','orangered', "limegreen", "lightseagreen", \
                    "darkviolet", "lightpink", "greenyellow","lightslategrey", \
                    "orange", "gold", "y", "yellowgreen", "palegreen", "turquoise", \
                    "aqua", "dodgerblue", "darkorchid", "deeppink"]
    
    cmap = colors.ListedColormap(color_record[:4+nrobot])
    data = [[0 for _ in range(n)] for _ in range(m)]
    fig, ax = plt.subplots(figsize = (n, m))
    plt.axis('equal')
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
    ax.set_xticks(x_pos)
    ax.set_yticks(y_pos, y_labels)
    for x, y, label in tasks:
        ax.text(y+0.4, x+0.4, label)
    for x, y, label in servs:
        ax.text(y+0.4, x+0.4, label+" "+str(task_cap[label]))
    ex_text_len = len(ax.texts)
    idx = [0 for _ in range(nrobot)]
    sum_path_len = sum([len(path) for path in path_list])
    wait_flag = [False for _ in range(nrobot)]
    while sum(idx) < sum_path_len:
        del ax.texts[ex_text_len:]
        for k in range(nrobot):
            path = path_list[k]
            i_alloc = new_alloc[k]
            if idx[k] == len(path): continue
            x, y = path[idx[k]]
            ax.text(y+0.2, x+0.2, s = "R"+str(k)) # 横纵坐标plot时是反的
            if data[x][y] == 0 or (data[x][y] != 2 and data[x][y] != 3):
                data[x][y] = 4+k
            # modify next position
            if (x, y) in servs_dic: # need to wait
                co_task = servs_dic[(x, y)]
                if i_alloc and i_alloc[0] == co_task: # need to wait
                    if task_cap[co_task] == 0:
                        i_alloc.popleft()
                        wait_flag[k] = False
                        idx[k] += 1
                        continue
                    if not wait_flag[k]: #如果还没开始等
                        task_cap[co_task] -= 1
                        wait_flag[k] = True
                        continue
                else:
                    idx[k] += 1
            else:
                idx[k] += 1
        _ = ax.pcolormesh(data, cmap=cmap, edgecolors='k', linewidths=1.5, alpha = 0.6) # as one collection      
        print(alloc)
        print(task_cap)
        plt.show()
        plt.pause(0.8)
        if sum(idx) != sum_path_len:
            del ax.collections[-1]         
    fig.savefig(save_name)
    plt.pause(1.1)
    plt.close()   
        
if __name__ == "__main__":
    m, n = 6, 12
    obs = [(2, 3), (3, 3), (4, 3)]
    tasks = [[3, 2, "t11"], [5, 9, "t12"]]
    servs = [[2, 5, "s1"]]
    
#    path = [(x, y) for x in range(m) for y in range(n)]
    path = [(0, i) for i in range(10)]
    grid_map(m, n, path, obs, tasks, servs)