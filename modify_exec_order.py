# -*- coding: utf-8 -*-
from planning import extract_transition
from collections import defaultdict


def find_exec_node(pa, ts, t_pos, tasks):
    """return an dic(list) for each task"""
    exe_nodes = defaultdict(set)
    for n1, n2 in list(pa.edges()):
        label = pa[n1][n2]["label"]
        label_require = extract_transition(label)
        key_list = list(label_require.keys())
        # set containing the enable task
        enable_set = label_require[key_list[0]][0]
        ts_name = pa.nodes[n2]["ts"]
        grid_pos = ts.nodes[ts_name]["pos"]
        for t in tasks:
            if t in enable_set and t_pos[t] == grid_pos:
                exe_nodes[t].add(n2)
    return exe_nodes


def check_exec_node(pa, ts, n1, n2, t, pos):
    """t: task name; 
       pos: task exec position"""
    label = pa[n1][n2]["label"]
    label_require = extract_transition(label)
    key_list = list(label_require.keys())
    # set containing the enable task
    enable_set = label_require[key_list[0]][0]
    ts_name = pa.nodes[n2]["ts"]
    # print(type(ts), ts)
    grid_pos = ts.nodes[ts_name]["pos"]
    if t in enable_set and pos == grid_pos:
        return True
    return False


def find_exec_idx(robot, path, s_pos: dict):
    idx = []
    i = 0
    t = robot.c_tasks[i]
    pos = s_pos[t]  # tuple
    for j in range(1, len(path)):
        if check_exec_node(robot.pa, robot.ts, path[j-1], \
                           path[j], t, pos):
            idx.append(j)
            i += 1
            if i == len(robot.c_tasks):
                break
            t = robot.c_tasks[i]
            pos = s_pos[t]
    if len(idx) != len(robot.c_tasks):
        print("Error occur when find exec idx in given plan. #exec_nodes != #tasks")
        assert False
    return idx


def compute_finish_time(cost: list, idxs: list, alloc: list, task_order: list, twist_task: dict):
    """the function will not change cost and idxs"""
    t2r = defaultdict(list)
    for i, tasks in enumerate(alloc):
        for j, t in enumerate(tasks):
            t2r[t].append((i, j)) # i: robot_idx; j: robot c_task idx
    for t0, tasks in twist_task.items():
        for t in tasks:
            t2r[t0] += t2r[t]
    n = len(cost)
    delta = [0 for _ in range(n)]
    exec_time = [[-1 for _ in range(n)] for _ in range(len(task_order))]
    for k, t in enumerate(task_order):
        latest = 0
        for i, j in t2r[t]:
            latest = max(latest, idxs[i][j]+delta[i])
            exec_time[k][i] = idxs[i][j] + delta[i]
        for i, j in t2r[t]:
            delta[i] = latest - idxs[i][j]
    # print("haha", cost, delta)
    end_time = [cost[i] + delta[i] for i in range(n)]
    return end_time, exec_time
