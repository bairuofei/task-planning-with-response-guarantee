# -*- coding: utf-8 -*-
# -*- coding: utf-8 -*-label_require

import sys
from z3 import *
from collections import deque, defaultdict
import time
import threading
import copy
import random

from gridmap_plot import *
from planning import *
from modify_exec_order import *
from pickle_func import *


def trace_accept(trace: list, ba, init_nodes: list, accept_nodes: list) -> bool:
    queue = init_nodes  # 存储node
    step = 0
    while queue:
        if step < len(trace):
            cur_ap = trace[step]
            step += 1
        else:
            break
        new_queue = []
        visited = set()
        while queue:
            cur = queue.pop()
            if cur in visited:
                continue
            visited.add(cur)
            for node in list(ba[cur]):
                # print(cur, node)
                label = ba[cur][node]["label"]
                if can_transit(label, cur_ap):
                    # print(can_transit(label, cur_ap), label, cur_ap, cur, node)
                    new_queue.append(node)
        queue = new_queue

    if step < len(trace):
        return False
    accept_set = set(accept_nodes)
    for node in queue:
        if node in accept_set:
            return True
    return False


def decomposition(ba, init_nodes: list, accept_nodes: list, label_require=None) -> set:
    if label_require == None:
        label_require = {}
        for start, end, label in list(ba.edges.data("label")):
            # return dict, key for each sub_clause, and value = (enable_set, disable_set)
            label_require[label] = extract_transition(label)
    decompose = set(init_nodes + accept_nodes)
    for node in list(ba.nodes()):
        if node in decompose:
            continue
        for init in init_nodes:
            find = False  # 控制退出for循环
            try:
                path1 = nx.dijkstra_path(ba, init, node)
            except nx.NetworkXNoPath:
                continue
            trace1 = []
            for i in range(len(path1)-1):
                label = ba[path1[i]][path1[i+1]]["label"]
                key_list = list(label_require[label].keys())
                trace1.append(list(label_require[label][key_list[0]][0]))

            for accept in accept_nodes:
                try:
                    path2 = nx.dijkstra_path(ba, node, accept)
                except nx.NetworkXNoPath:
                    continue
                # each node related to a list of aps
                trace2 = []
                for i in range(len(path2) - 1):
                    label = ba[path2[i]][path2[i+1]]["label"]
                    key_list = list(label_require[label].keys())
                    trace2.append(list(label_require[label][key_list[0]][0]))
                # complete trace

                if trace_accept(trace2 + trace1, ba, init_nodes[:], accept_nodes[:]):
                    # print(node, trace1, trace2)
                    decompose.add(node)
                    find = True
                    break
            if find:
                break
    return decompose


def task_decompose(ba, init_nodes: list, accept_nodes: list, decompose: set, alpha: float) -> list:
    # multi-obj path search
    # J = alpha * cost[0] + （1-alpha) * cost[1]
    # labels[node][0]: temporal labels, labels[node][1]: premenent labels
    def pareto_le(a: tuple, b: tuple) -> bool:
        for i in range(len(a)):
            if a[i] > b[i]:
                return False
        return True
    accept_set = set(accept_nodes)
    heap = []
    labels = {}
    useless = set()

    for node in init_nodes:
        labels[node] = (set(), [])
        # cost = (steps, d_nodes) 目标是steps少，且d_nodes多，alpha取接近于0的数
        cost = (0, -1)
        J = alpha * cost[0] + (1-alpha) * cost[1]
        J = round(J, 3)
        # (J: int, cost: tuple, cur: str, father: str, idx: int)
        # cost用tuple的原因是，否则无法存入set中，无法hash
        temp = (J, cost, node, "", -1)
        heapq.heappush(heap, temp)
        labels[node][0].add(temp)
    ret = None
    while heap:
        temp = heapq.heappop(heap)
        if str(temp) in useless:
            continue
        J, cost, cur, father, idx = temp
        try:
            labels[cur][0].remove(temp)
        except KeyError:
            print(KeyError)
        labels[cur][1].append(temp)
        if cur in accept_set:
            ret = temp
            break
        for post in list(ba[cur]):
            if post == cur:
                continue
            new_cost = [cost[0] + 1, cost[1]]
            if post in decompose:
                new_cost[1] -= 1
            new_cost = tuple(new_cost)
            new_J = alpha * new_cost[0] + (1 - alpha) * new_cost[1]
            new_J = round(new_J, 3)
            new_temp = (new_J, new_cost, post, cur, len(labels[cur][1]) - 1)

            if post not in labels:
                labels[post] = (set(), [])
                labels[post][0].add(new_temp)
                heapq.heappush(heap, new_temp)
                continue
            useful = True
            # prem
            for old in labels[post][1]:
                if pareto_le(old[1], new_cost):
                    useful = False
                    break
            if not useful:
                continue
            # temp
            to_remove = []
            for old in labels[post][0]:
                if pareto_le(old[1], new_cost):
                    useful = False
                    break
                elif pareto_le(new_cost, old[1]):
                    to_remove.append(old)
            if not useful:
                continue
            # modify post labels
            for old in to_remove:
                try:
                    labels[post][0].remove(old)
                    useless.add(str(old))
                except KeyError:
                    print(KeyError)
            labels[post][0].add(new_temp)
            heapq.heappush(heap, new_temp)

    if ret == None:
        return []
    path = deque()
    while ret[3]:
        _, _, cur, father, idx = ret
        path.appendleft(cur)
        ret = labels[father][1][idx]
    path.appendleft(ret[2])
    return path


def decompose_tasks(path, decompose):
    tasks = []
    for i in range(len(path)-1):
        if path[i] in decompose:
            tasks.append([])
        label = ba[path[i]][path[i+1]]["label"]
        # return dict, key for each sub_clause, and value = (enable_set, disable_set)
        label_require = extract_transition(label)
        # 直接取第一个转移条件
        key_list = list(label_require.keys())
        # set containing the enable task
        enable_set = label_require[key_list[0]][0]
        tasks[-1].append(list(enable_set))
    return tasks


def ba_deterministic(ba):
    for start, end, label in list(ba.edges.data("label")):
        # return dict, key for each sub_clause, and value = (enable_set, disable_set)
        label_split = label.split(" || ")
        label = label_split[0]
        ba[start][end]["label"] = label


def ba_feasible_check(ba, init_nodes, accept_nodes, num_r, task_cap) -> bool:
    # 判断当前LTL表达式要求能否由现有的机器人完成
    for start, end, label in list(ba.edges.data("label")):
        label_require = extract_transition(label)
        # 直接取第一个转移条件
        key_list = list(label_require.keys())
        enable_set = label_require[key_list[0]][0]
        count = 0
        for ap in enable_set:
            count += task_cap[ap]
        if count > num_r:
            ba.remove_edge(start, end)
    # path existence check
    for init in init_nodes:
        for accept in accept_nodes:
            try:
                _ = nx.dijkstra_path(ba, init, accept)
                return True
            except nx.NetworkXNoPath:
                continue
    return False


def cooperation_task(tasks: list, alloc: list) -> list:
    # 将ＳＭＴ的求解结果划分成每个机器人的任务集合
    task_list = [tasks[j][k][l] for j in range(len(tasks)) for k in range(len(tasks[j]))
                 for l in range(len(tasks[j][k]))]
    count = 0
    idx = 0
    alloc_task = []
    while idx < len(alloc):
        if count == 0:
            alloc_task.append([])
        if alloc[idx] == 1:
            alloc_task[-1].append(task_list[count])
        idx += 1
        count += 1
        if count == len(task_list):
            count = 0
    return alloc_task


def optimal_path_tree(graph, source, target):
    """compute optimal path from source(list) to target(list)"""
    min_cost = float("inf")
    path = []
    for init in source:
        try:
            cost = nx.shortest_path_length(
                graph, init, weight="weight")  # return dic
            for accept in target:
                if cost[accept] < min_cost:
                    key = accept
                    min_cost = cost[accept]
            path = nx.shortest_path(graph, init, key, weight="weight")
        except nx.NetworkXNoPath:
            continue
    return min_cost, path


def iteration(MAS):
    """calculate optimal plan for each robot in MAS"""
    iter_start = time.time()
    for i, robot in MAS.items():
        ts = robot.ts
        print(f"Robot {i}:")
        TIME_start = time.time()
        ba, init_nodes, accept_nodes = ltl_formula_to_ba(robot.f)
        print(f"BA. Time: {time.time() - TIME_start}s. #node {len(ba)}: ")
        pa, pa_init, pa_accept = product_automaton(
            ts, ba, init_nodes, accept_nodes, init_pos=robot.pos)
        robot.set_pa(pa, pa_init, pa_accept)
        print(
            f"PA. Time: {time.time() - TIME_start}s. #init: {len(pa_init)}. #accept: {len(pa_accept)}. #node: {len(pa)}.")
        source = []
        for init in pa_init:
            ts_node = pa.nodes[init]["ts"]
            if ts.nodes[ts_node]["pos"] == robot.pos:
                source.append(init)
        # TODO: 这里可以对pa_accept进行缩减，只保留task的位置对应的accept状态
        min_cost, pa_path = optimal_path_tree(
            pa, source, pa_accept)  # path contains pa states
        robot.set_pa_path(pa_path)
        robot.set_path_cost(min_cost)
        path_label = []  # print labels along the path
        for node in pa_path:
            ts_node = pa.nodes[node]["ts"]
            if ts.nodes[ts_node]["label"]:
                path_label.append(ts.nodes[ts_node]["label"])
        print(robot.f)
        print(path_label)
        print(f"Time Used: {time.time()-TIME_start}s.")
        print("---------------------------------------------")
        # animation: real_path is a list of real_path
        # grid_map(env, real_path, save_name = "min-cost-path")
    print("Finish PA construction.")
    print(f"Time for all PA: {time.time() - iter_start}s.")
    return


def construct_formula(alloc):
    coor_formula = []  # record global formula pieces after decomposition
    for i, t_list in enumerate(alloc):
        exp_f = deque()
        for t in reversed(t_list):
            if exp_f:
                exp_f.appendleft("(F(" + t + " && ")
                exp_f.append("))")
            else:
                exp_f.append("(F " + t + ")")
        coor_formula.append("".join(exp_f))
    return coor_formula


def optimise_latest(robot, t_idx, exec_time, end_sum, old_cost, old_idxs,
                    alloc, task_order, twist_task, s_pos):
    """from all nodes in cand, find one cand that can form better path.
       compare with: 1. cur_max; 2. end_sum"""
    # preprocessing
    # considering twist task and get the actual task name
    j = robot.id
    t = task_order[t_idx]
    if t in twist_task and t not in robot.c_tasks:
        for t_twist in twist_task[t]:
            if t_twist in robot.c_tasks:
                t = t_twist
                break
    path = robot.pa_path
    source = [path[0]]
    cand = copy.deepcopy(robot.pa_exec[t])  # set
    prev_time = 0
    cur_max = exec_time[t_idx][j]
    new_path = []
    k = robot.c_tasks.index(t)  # currently modified c_task idx
    cur = path[old_idxs[j][k]]
    try:
        cand.remove(cur)
    except:
        assert False
    if k > 0:
        prev = path[old_idxs[j][k-1]]
        # exec time of prev task, considering wait
        p_t = find_leader_t(task_order, twist_task, robot.c_tasks[k-1])
        prev_time = exec_time[task_order.index(p_t)][j]
        new_path += path[:old_idxs[j][k-1]+1]
        source = [prev]
    pa = robot.pa
    for init in source:
        try:
            short_cost = nx.shortest_path_length(pa, init, weight="weight")
            for new in cand:
                if new not in short_cost or short_cost[new] + prev_time > cur_max:
                    continue
                tail1 = nx.shortest_path(pa, init, new, weight="weight")
                if cand in robot.pa_accept:
                    new_path1 = new_path + tail1
                else:
                    suf_cost, suf_path = nx.single_source_dijkstra(
                        pa, new, weight="weight")
                    min_ac = None
                    min_suf_cost = float("inf")
                    for ac in robot.pa_accept:
                        if suf_cost[ac] < min_suf_cost:
                            min_suf_cost = suf_cost[ac]
                            min_ac = ac
                    tail2 = suf_path[min_ac]
                    new_path1 = new_path + tail1 + tail2[1:]
                if cand_is_better(robot, new_path1, end_sum, old_cost, old_idxs,
                                  alloc, task_order, twist_task, s_pos):
                    new_path += tail1 + tail2[1:]
                    robot.pa_path = new_path
                    return True
        except nx.NetworkXNoPath:
            continue
    # do not exist better
    return False


def cand_is_better(robot, new_path, end_sum, old_cost, old_idxs,
                   alloc, task_order, twist_task, s_pos):
    j = robot.id
    idx = find_exec_idx(robot, new_path, s_pos)
    test_idxs = copy.deepcopy(old_idxs)
    test_cost = copy.deepcopy(old_cost)
    # print(new_path)
    test_cost[j] = compute_path_cost(robot.pa, new_path)
    if idx != None:
        test_idxs[j] = idx
    else:
        print("Error when finding exec node in plan!")
    end_time, _ = compute_finish_time(
        test_cost, test_idxs, alloc, task_order, twist_task)
    if sum(end_time) < end_sum:
        print("Better Cand", sum(end_time), end_sum)
        return True
    print("Worse Cand", sum(end_time), end_sum)
    return False


def find_leader_t(task_order, twist_task, t):
    if t in task_order:
        return t
    for leader, tasks in twist_task.items():
        if t in tasks:
            return leader

# TOOD: path cost　is not consistent with time consuming
def compute_path_cost(pa, path):
    cost = 0
    for i in range(len(path)-1):
        cost += pa[path[i]][path[i+1]]["weight"]
    return cost


def optimise_time_without2(MAS, alloc, task_order, twist_task, s_pos):
    # global variables
    # optimise = None
    idxs = []
    idea_cost = []
    for t_i, robot in MAS.items():
        idx = find_exec_idx(robot, robot.pa_path, s_pos)
        idxs.append(idx)
        robot.set_pa_exec(find_exec_node(
            robot.pa, robot.ts, s_pos, robot.c_tasks))
        idea_cost.append(robot.path_cost)

    # exec_time: each step record given last step(considering synchronization), compute current step execution time for each robot
    end_time, exec_time = compute_finish_time(
        idea_cost, idxs, alloc, task_order, twist_task)
    # optimise.append(sum(idea_cost))
    # optimise.append(sum(end_time))
    
    # iterating over: cost, idxs, pa_path_list, end_time, exec_time
    t_i = 0
    changed_cnt = 0
    iter_cnt = -1
    while True:
        iter_cnt += 1
        j, max_v = -1, 0
        for k, v in enumerate(exec_time[t_i]):
            if v > max_v:
                max_v = v
                j = k
        robot = MAS[j]
        can_opt = optimise_latest(robot, t_i, exec_time, sum(end_time), idea_cost,
                                  idxs, alloc, task_order, twist_task, s_pos)
        if can_opt:
            idea_cost[j] = compute_path_cost(robot.pa, robot.pa_path)
            idxs[j] = find_exec_idx(robot, robot.pa_path, s_pos)
            end_time, exec_time = compute_finish_time(
                idea_cost, idxs, alloc, task_order, twist_task)
            changed_cnt += 1
        else:
            if t_i == len(task_order) - 1:
                if changed_cnt == 0:
                    break
                changed_cnt = 0
        t_i = (t_i+1) % len(task_order)

    # print(sum(end_time))
    print("Opt1-Iteration: ", iter_cnt)
    return sum(end_time)


def optimise_time(MAS, alloc, task_order, twist_task, s_pos):
    # global variables
    optimise = []
    idxs = []
    idea_cost = []
    for t_i, robot in MAS.items():
        idx = find_exec_idx(robot, robot.pa_path, s_pos)
        idxs.append(idx)
        robot.set_pa_exec(find_exec_node(
            robot.pa, robot.ts, s_pos, robot.c_tasks))
        idea_cost.append(robot.path_cost)

    # exec_time: each step record given last step(considering synchronization), compute current step execution time for each robot
    end_time, exec_time = compute_finish_time(
        idea_cost, idxs, alloc, task_order, twist_task)
    optimise.append(sum(idea_cost))
    optimise.append(sum(end_time))   
    # if best_sum < optimise[0]:
    #     print("AAAAAAAAAAAAAA")

    # iterating over: cost, idxs, pa_path_list, end_time, exec_time
    t_i = 0
    changed_cnt = 0
    iter_cnt = 0
    while True:
        iter_cnt += 1
        j, max_v = -1, 0
        for k, v in enumerate(exec_time[t_i]):
            if v > max_v:
                max_v = v
                j = k
        robot = MAS[j]
        can_opt = optimise_latest(robot, t_i, exec_time, sum(end_time), idea_cost,
                                  idxs, alloc, task_order, twist_task, s_pos)
        if can_opt:
            idea_cost[j] = compute_path_cost(robot.pa, robot.pa_path)
            idxs[j] = find_exec_idx(robot, robot.pa_path, s_pos)
            end_time, exec_time = compute_finish_time(
                idea_cost, idxs, alloc, task_order, twist_task)
            changed_cnt += 1
        else:
            # TODO: modify the earliest robot execution time
            j, min_v = -1, float("inf")
            for k, v in enumerate(exec_time[t_i]):
                if v != -1 and v < min_v:
                    min_v = v
                    j = k
            robot = MAS[j]
            print("opt2 start")
            can_opt2 = optimise_earliest(robot, t_i, min_v, max_v, exec_time, sum(end_time), idea_cost,
                                         idxs, alloc, task_order, twist_task, s_pos)
            print("opt2 end")
            if can_opt2:
                idea_cost[j] = compute_path_cost(robot.pa, robot.pa_path)
                idxs[j] = find_exec_idx(robot, robot.pa_path, s_pos)
                end_time, exec_time = compute_finish_time(
                    idea_cost, idxs, alloc, task_order, twist_task)
                changed_cnt += 1
            else:
                if t_i == len(task_order) - 1:
                    if changed_cnt == 0:
                        break
                    changed_cnt = 0
        t_i = (t_i+1) % len(task_order)

    print(sum(end_time))
    print("Opt12-Iteration: ", iter_cnt)
    optimise.append(sum(end_time))
    return optimise


def optimise_earliest(robot, t_i, min_v, max_v, exec_time, end_sum, old_cost, old_idxs,
                      alloc, task_order, twist_task, s_pos) -> bool:
    """modify the earliest robot pa_path"""
    # preprocessing
    # considering twist task and get the actual task name
    j = robot.id
    t = task_order[t_i]
    if t in twist_task and t not in robot.c_tasks:
        for t_twist in twist_task[t]:
            if t_twist in robot.c_tasks:
                t = t_twist
                break
    path = robot.pa_path
    source = [path[0]]
    cand = copy.deepcopy(robot.pa_exec[t])  # set
    prev_time = 0
    new_path = []
    k = robot.c_tasks.index(t)  # currently modified c_task idx
    cur = path[old_idxs[j][k]]
    try:
        cand.remove(cur)
    except:
        assert False
    if k > 0:
        prev = path[old_idxs[j][k-1]]
        # exec time of prev task, considering wait
        p_t = find_leader_t(task_order, twist_task, robot.c_tasks[k-1])
        prev_time = exec_time[task_order.index(p_t)][j]
        new_path += path[:old_idxs[j][k-1]+1]
        source = [prev]
    pa = robot.pa
    for init in source:
        try:
            short_cost = nx.shortest_path_length(pa, init, weight="weight")
            for new in cand:
                if new not in short_cost or short_cost[new] + prev_time > max_v \
                        or short_cost[new] + prev_time < min_v:
                    continue
                tail1 = nx.shortest_path(pa, init, new, weight="weight")
                if cand in robot.pa_accept:
                    new_path1 = new_path + tail1
                else:
                    suf_cost, suf_path = nx.single_source_dijkstra(
                        pa, new, weight="weight")
                    min_ac = None
                    min_suf_cost = float("inf")
                    for ac in robot.pa_accept:
                        if suf_cost[ac] < min_suf_cost:
                            min_suf_cost = suf_cost[ac]
                            min_ac = ac
                    tail2 = suf_path[min_ac]
                    new_path1 = new_path + tail1 + tail2[1:]
                if cand_is_better(robot, new_path1, end_sum, old_cost, old_idxs,
                                  alloc, task_order, twist_task, s_pos):
                    new_path += tail1 + tail2[1:]
                    robot.pa_path = new_path
                    return True
        except nx.NetworkXNoPath:
            continue
    # do not exist better
    return False


def extract_ts_path(pa, ts, path):
    """given pa path ,return ts path"""
    res = []
    for n in path:
        ts_node = pa.nodes[n]["ts"]
        res.append(ts.nodes[ts_node]["pos"])
    return res


def is_worse_smt(new, old_set):
    # 存在解比其中一个worse,就返回True
    for old in old_set:
        worse = True
        for i, value in enumerate(new):
            if value < old[i]:  # 　说明不会比当前的old worse
                worse = False
                break
        if worse:
            return True
    return False


def remove_useless_res(new, old_set):
     # 存在解比res_list worse,就删除它
    new_set = set()
    for old in old_set:
        worse = True
        for i, value in enumerate(new):
            if old[i] < value:
                worse = False
                break
        if not worse:
            new_set.add(old)
    return new_set


class SmtObj:
    def __init__(self):
        self.s = Solver()

    def add_constraint(self, cons):
        self.s.add(cons)

    def add_constraints(self, cons_list):
        for cons in cons_list:
            self.s.add(cons)

    def check(self):
        return self.s.check()

    def model(self):
        return self.s.model()

    def add_task_cons(self, tasks, num_r):
        self.X = [[[[Int("x_%i_%i_%i_%i" % (i, j, k, l)) for l in range(len(tasks[j][k]))]
                    for k in range(len(tasks[j]))]
                   for j in range(len(tasks))]
                  for i in range(num_r)]
        self.val_c = [And(0 <= self.X[i][j][k][l], self.X[i][j][k][l] <= 1) for i in range(num_r)
                      for j in range(len(tasks)) for k in range(len(tasks[j]))
                      for l in range(len(tasks[j][k]))]
        self.task_c = [sum([self.X[i][j][k][l] for i in range(num_r)]) >= task_cap[tasks[j][k][l]]
                       for j in range(len(tasks)) for k in range(len(tasks[j]))
                       for l in range(len(tasks[j][k]))]
        self.mutual_c = [sum([self.X[i][j][k][l] for l in range(len(tasks[j][k]))]) <= 1 for i in range(num_r)
                         for j in range(len(tasks)) for k in range(len(tasks[j]))]
        self.connect_c = [Or([Implies(Or([self.X[i][j][k][l] == 1 for l in range(len(tasks[j][k]))]),
                                      Or([self.X[i][j][k+1][l] == 1 for l in range(len(tasks[j][k+1]))]))
                              for i in range(num_r)]) for j in range(len(tasks)) for k in range(len(tasks[j])-1)]
        self.add_constraints(
            [self.val_c, self.task_c, self.mutual_c, self.connect_c])


class Environment:
    def __init__(self, m=10, n=10):
        self.grid = [[[] for _ in range(n)] for _ in range(m)]
        self.size = (m, n)
        self.s_pos = {}
        self.obs = set()
        self.t = set()
        self.ct = set()

    def add_obs(self, pos_list):
        for x, y in pos_list:
            self.grid[x][y].append("obs")
            self.obs.add((x, y))

    def add_t(self, task_label):
        for x, y, ap in task_label:
            self.grid[x][y].append(ap)
            self.t.add((x, y))

    def add_ct(self, task_label):
        for x, y, ap in task_label:
            self.grid[x][y].append(ap)
            self.s_pos[ap] = (x, y)
            self.ct.add((x, y))


class Robot:
    def __init__(self, id=0, pos=None, local_f="", coor_f=""):
        self.id = id
        self.local_f = local_f
        self.coor_f = coor_f
        self.f = self.local_f + " && " + self.coor_f
        self.pos = pos

    def set_task_formula(self, formula):
        self.f = formula

    def set_pos(self, pos):
        self.pos = pos

    def set_ts(self, ts):
        self.ts = ts

    def set_pa(self, pa=None, pa_init=None, pa_accept=None):
        self.pa = pa
        self.pa_init = pa_init
        self.pa_accept = pa_accept

    def set_pa_path(self, path):
        self.pa_path = path
        self.ts_path = extract_ts_path(self.pa, self.ts, self.pa_path)
        
    def set_path_cost(self, cost):
        self.path_cost = cost

    def set_coor_f(self, f):
        self.coor_f = f
        self.f = self.local_f + " && " + self.coor_f

    def set_pa_exec(self, exec_dict):  # dict of exec node for each ct in PA
        self.pa_exec = exec_dict

    def set_c_tasks(self, alloc):
        self.c_tasks = alloc



if __name__ == "__main__":
    time_prefix = str(int(time.time()))[-3:]
    
    ## configurations:
    num_r = 4
    # environment setting
    m, n = 30, 30
    # hronzital
    h_obs = [(10, 10+i) for i in range(4)]+[(10, 16+i) for i in range(4)] +\
            [(19, 10+i) for i in range(4)]+[(19, 16+i) for i in range(4)]
    # vertical
    v_obs = [(11+i, 10) for i in range(3)]+[(16+i, 10) for i in range(3)] +\
            [(11+i, 19) for i in range(3)]+[(16+i, 19) for i in range(3)]
    # task definition
    tasks_collection = [[[0, 0, "t1"], [1, 7, "t2"], [7, 3, "t3"], [5, 11, "t4"]],
                        [[20, 0, "t5"], [21, 7, "t6"], [27, 3, "t7"], [25, 11, "t8"]],
                        [[0, 20, "t9"], [1, 27, "t10"], [7, 23, "t11"], [5, 29, "t12"]],
                        [[20, 20, "t13"], [21, 27, "t14"], [27, 23, "t15"], [25, 28, "t16"]]]
    coor_tasks = [[15, 6, "s1"], [25, 5, "s2"],
                  [3, 3, "s3"], [16, 0, "s4"]]
    task_cap = {"s1": 1, "s2": 3, "s3": 2, "s4": 2}
    c_formula = '(F s1) && (F s2) && (F s4) && (!s3 U s2) && (G(s4 -> (F s3)))'
    # tasks capability
    local_formula = ["(F t1) && (F t2) && (F t3) && (F t4) && (!t1 U t4)",
                    "(F t5) && (F t6) && (F t7) && (F t8) && (!t6 U t8)",
                    "(F t9) && (F t10) && (F t11) && (F t12) && (!t10 U t12)",
                    "(F t13) && (F t14) && (F t15) && (F t16) && (!t16 U t15)"] 
    init_pos = [(3, 0), (22, 0), (3, 27), (24, 28)]
    
    T_start = time.time()  # starting point of whole program

    envs = {}
    for i in range(num_r):
        envs[i] = Environment(m, n)
        envs[i].add_obs(h_obs)
        envs[i].add_obs(v_obs)
        envs[i].add_t(tasks_collection[i])
        envs[i].add_ct(coor_tasks)
    MAS = {}
    for i in range(num_r):
        MAS[i] = Robot(i, pos=init_pos[i], local_f=local_formula[i])
        ts = grid2map(envs[i].grid)
        MAS[i].set_ts(ts)     
    print("TS successfully constructed.")
    
    # decompose global task formula
    ba, init_nodes, accept_nodes = ltl_formula_to_ba(
        c_formula)  # convert to BA
    print(f"Convert c_formula to BA. #state: {len(ba)}")
    # show_BA(ba)
    ba_deterministic(ba)  # remove "or" transition condition
    exist_path = ba_feasible_check(
        ba, init_nodes, accept_nodes, num_r, task_cap)
    # show_BA(ba, title="feasible_ba")
    if not exist_path:
        print("Task Requirement can not be Satisfied. Program Break.")
        sys.exit()
    else:
        decompose = decomposition(ba, init_nodes, accept_nodes)  # 求解分割节点
        alpha = 0.1  # 在ba上搜索可以分割的路径
        path = task_decompose(
            ba, init_nodes, accept_nodes, decompose, alpha)
        tasks = decompose_tasks(path, decompose)  # 　根据路径分离任务
        task_order = [t[0] for seg in tasks for t in seg]
        twist_task = {}
        for seg in tasks:
            for t in seg:
                if len(t) > 1:
                    twist_task[t[0]] = t[1:]
        
        print("Finishing decompose tasks from global BA.")
        smt = SmtObj()
        smt.add_task_cons(tasks, num_r)
        iter_cnt = 0
        sol_record = set()  # record all past useful solution to filt newly obtained solution
        smt_filt = []
        opt_res = []
        solution = []
        best_ever = float("inf")
        print("Starting Iteration:")
        remove_cnt = 0
        while smt.check() == sat:
            iter_cnt += 1  # useful solution idx, filt with pareto optimal
            sm = smt.model()
            res = [[[[sm.evaluate(smt.X[i][j][k][l]) for l in range(len(tasks[j][k]))]
                     for k in range(len(tasks[j]))] for j in range(len(tasks))]
                   for i in range(num_r)]
            # 添加非命题
            f = And([smt.X[i][j][k][l] == res[i][j][k][l] for i in range(num_r)
                     for j in range(len(tasks)) for k in range(len(tasks[j]))
                     for l in range(len(tasks[j][k]))])
            # neg_f = Not(f)
            smt.add_constraint(Not(f))
            # 判断是否要计入对比
            x_sol = [res[i][j][k][l].as_long() for i in range(num_r) for j in range(len(tasks))
                     for k in range(len(tasks[j])) for l in range(len(tasks[j][k]))]
            # filt smt solution
            if sol_record != None:
                if is_worse_smt(x_sol, sol_record):  # 新的解没意义
                    iter_cnt -= 1
                    remove_cnt += 1
                    continue
                # remove old useless assignments
                sol_record = remove_useless_res(x_sol, sol_record)
            sol_record.add(tuple(x_sol))
            smt_filt.append([len(sol_record), remove_cnt]) # record the filted sol
            remove_cnt = 0
            
            print("*********************")
            print(f"Iteration {iter_cnt}:")
            print("*********************")

            alloc = cooperation_task(tasks, x_sol)  # list[list]
            coor_formula = construct_formula(alloc)
            for i, rb in MAS.items():
                rb.set_coor_f(coor_formula[i])
                rb.set_c_tasks(alloc[i])

            # TODO: change ts from public to a private list
            iteration(MAS)

            # animation
            # copy_task_cap = copy.deepcopy(task_cap)
            # animate_path(env, ts_path_list, alloc, copy_task_cap)
            pa_path_list = []
            idea_cost = []
            for r, robot in MAS.items():
                pa_path_list.append(robot.pa_path)
                idea_cost.append(robot.path_cost)
                      
            print("Opt1+Opt2:")
            optimise = optimise_time(MAS, alloc, task_order,
                                     twist_task, envs[0].s_pos)
            for r, robot in MAS.items():
                robot.set_pa_path(pa_path_list[r])
                robot.set_path_cost(idea_cost[r])
            print("Opt1:")
            optimise.append(optimise_time_without2(MAS, alloc, task_order,
                                                  twist_task, envs[0].s_pos))
            optimise.append(min(best_ever, optimise[2]))
            opt_res.append(optimise)
            solution.append(tuple(x_sol))
            # optimise: LB; UB; opt1+opt2; opt1; cur_best.
            file_name1 = save_variable(
                opt_res, "./data/" + time_prefix + "opt.txt")
            file_name2 = save_variable(
                solution, "./data/" + time_prefix + "sol.txt")
            file_name3 = save_variable(
                smt_filt, "./data/" + time_prefix + "filt.txt")
            print(
                f"End of Iteration {iter_cnt}. Program running {time.time() - T_start}s.")
        else:
            print("fail to solve")

