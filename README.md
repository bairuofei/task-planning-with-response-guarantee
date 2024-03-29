# Bottom-up Multi-robot Task Planning

The project aims to synthesis task planning for a multi-robot system, given global LTL task specification and local task specification.

## Guide

1. Add local task and global task LTL formual in `ltl_decomposition.py`. And also specify the task requirements for each coorperative task.
2. execute `ltl_decomposition.py`.

## Notes
在classical_plan.py中，算出来的结果对应的总任务执行时间，就是最优路径的cost乘以机器人数量。
这是因为在执行协同任务时，classical_plan中算出的plan就是同步之后的。

需要注意的是，由于不同机器人结束任务有先后，所以对于结束任务较早的机器人，后面的原地停留时间不应该
算入总任务执行时间，需要识别并进行剔除。

## TODO
1、目前在cand.remove时会出现问题；
２、就目前看，该算法没有效果。考虑将cur加入到cand中验证正确性。并进一步考虑调整
每个任务中最早执行的机器人的执行时间。

## Problems


## TOLEARN:
1. python position parameters and key word parameters
2. python is versus ==
3. python concurrent programming

### 已解决
Each feasible execution maintains a list of tuple, in which each tuple records
 the starting and ending node of the transition.
 
Then we only label the latter node accoriding to the pair in PA, not just 
using the latter node.

So checking whether one node is a candidate can be done when constructing the 
PA.

一个节点Ａ放入待选节点中，表明：
１、至少存在一条路径，到达A时满足了转移条件中对于A的要求。
但是依然存在其他路径，可能并没有通过执行A而到达了节点Ａ。

defaultdict(set,
            {'s1': {'T0_S0',
              'T0_S10',
              'T0_S11',
              'T0_S4',
              'T0_S5',
              'T0_S7',
              'T0_S9',
              'accept_all'},
             's2': {'T0_S10', 'T0_S11', 'T0_S9', 'accept_all'}})
             
- 等待时间优化算法分为两个步骤：
１、把最晚的机器人时间调早一点。对该机器人而言，如果尝试所有的移动方式，均无法降低总
执行时间，则切换到第二个步骤；
２、把最早的机器人时间调晚一点。对该机器人而言，如果尝试所有的待选路径，均无法在延迟
自身执行时间的情况下降低全局的执行时间，则停止迭代，将停止迭代的标志位置为True。
注意这里置迭代标识位时，如果task_order中上一个任务的标识位不是True，则不进行置位，
因为上游节点可能在后续的调整中使得当前节点可以继续调整。

- 循环迭代过程中维持什么？
循环过程中维持path和cost
             
# -*- coding: utf-8 -*-label_require

import sys
from z3 import *
from collections import deque, defaultdict
import time

from gridmap_plot import *
from planning import *
from modify_exec_order import *

# def feasible_trace(trace: list, ba, init_nodes, accept_nodes) -> bool:
#
#    queue = []
#    for node in init_nodes:


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
                #                print(cur, node)
                label = ba[cur][node]["label"]
                if can_transit(label, cur_ap):
                    #                    print(can_transit(label, cur_ap), label, cur_ap, cur, node)
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
        enable_set = label_require[key_list[0]][0] # set containing the enable task
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


def is_worse_smt(new, old_list):
    # 存在解比其中一个worse,就返回True
    for old in old_list:
        worse = True
        for i, value in enumerate(new):
            if value < old[i]:  # 　说明不会比当前的old worse
                worse = False
                break
        if worse:
            return True
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
            cost = nx.shortest_path_length(graph, init, weight = "weight") # return dic
            for accept in target:
                if cost[accept] < min_cost:
                    key = accept
                    min_cost = cost[accept]
            path = nx.shortest_path(graph, init, key, weight = "weight")
        except nx.NetworkXNoPath:
                continue
    return min_cost, path


def iteration(num_r: int, local_formula: list, coor_formula: list, ts: nx.DiGraph, init_pos: list):
    """return a list of list, containing [min_cost, real_path]"""
    path_cost = []
    pa_dict = {}
    for i in range(num_r):
        print(f"Robot {i}:")
        TIME_start = time.time()
        robot_pos = init_pos[i]  # initial pos of robot 
        new_f = local_formula[i] + " && " + coor_formula[i]
        ba, init_nodes, accept_nodes = ltl_formula_to_ba(new_f)
        print(f"BA. Time: {time.time() - TIME_start}s. #node {len(ba)}: ")
        pa, pa_init, pa_accept = product_automaton(ts, ba, init_nodes, accept_nodes, init_pos = robot_pos)
        print(f"PA. Time: {time.time() - TIME_start}s. #init: {len(pa_init)}. #accept: {len(pa_accept)}. #node: {len(pa)}.")
        pa_dict[i] = [pa, pa_init, pa_accept]
        source = []
        for init in pa_init:
            ts_node = pa.nodes[init]["ts"]
            if ts.nodes[ts_node]["pos"] == robot_pos:
                source.append(init)    
        # TODO: 这里可以对pa_accept进行缩减，只保留task的位置对应的accept状态
        min_cost, path = optimal_path_tree(pa, source, pa_accept) # path contains pa states
        ## extract real path on grid map
        real_path, path_label = [], []
        for node in path:
            ts_node = pa.nodes[node]["ts"]
            real_path.append(ts.nodes[ts_node]["pos"]) # consisting of position coordination
            if ts.nodes[ts_node]["label"]:
                path_label.append(ts.nodes[ts_node]["label"])
        path_cost.append([min_cost, real_path, path])                                                                                         
        print(new_f)
        print(path_label)
        print(f"Time Used: {time.time()-TIME_start}s.")   
        print("---------------------------------------------")
        # animation: real_path is a list of real_path
        # grid_map(m, n, real_path, obs, tasks_collection, coor_tasks, save_name = "min-cost-path")
    return path_cost, pa_dict




def remove_useless_res(new, old_list):
     # 存在解比res_list worse,就删除它
    new_list = []
    for old in old_list:
        worse = True
        for i, value in enumerate(new):
            if old[i] < value:
                worse = False
                break
        if not worse:
            new_list.append(old)
    return new_list

def construct_formula(alloc):
    coor_formula = [] # record global formula pieces after decomposition
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

   
def cand_check(pa, start, cand, accept, prev_time, max_time, new_path, ts, \
               s_pos, cost, alloc, idxs, task_order, twist_task, sums_time):
    for init in start:
        try:
            cost = nx.shortest_path_length(pa, init, weight = "weight")
            for new in cand:
                if cost[new] + prev_time >= max_time: continue
                tail1 = nx.shortest_path(pa, init, new, weight = "weight")
                if cand in accept:
                    new_path += tail1
                else:
                    tail2 = nx.shortest_path(pa, new, accept, weight = "weight")
                    new_path += tail1 + tail2[1:]
                if cand_is_better(pa, ts, new_path, s_pos, cost, alloc, idxs, task_order, twist_task, sums_time):
                    return True, new_path
        except nx.NetworkXNoPath:
            continue
    # do not exist better
    return False, new_path
                
                
def cand_is_better(pa, ts, new_path, s_pos, cost, alloc, idxs, task_order, twist_task, sums_time):
    idx = find_exec_idx(pa, ts, path, r_task, s_pos)
    if idx != None:
        idxs[j] = idx
    else:
        print("Error when finding exec node in plan!")
    end_time, exec_time = compute_finish_time(cost, alloc, idxs, task_order, twist_task)
    if sum(end_time) <= sums_time:
        return True
    return False


def find_leader_t(task_order, twist_task, t):
    if t in task_order: return t
    for leader, tasks in twist_task.items():
        if t in tasks: return leader
        
def compute_path_cost(pa, path):
    cost = 0
    for i in range(len(path)-1):
        cost += pa[path[i]][path[i+1]]["weight"]
    return cost


def optimise_time(pa_path_list, pa_dict, ts, cost, alloc, task_order, twist_task, s_pos, servs):
    idxs = []
    for i, path in enumerate(pa_path_list):
        r_task = alloc[i]
        pa = pa_dict[i][0]
        idx = find_exec_idx(pa, ts, path, r_task, s_pos)
        if idx != None:
            idxs.append(idx)
        else:
            print("Error when finding exec node in plan!")
    # exec_time: each step record given last step(considering synchronization), compute current step execution time for each robot
    end_time, exec_time = compute_finish_time(cost, alloc, idxs, task_order, twist_task)
    all_pa_exec = []
    for i, pa_list in pa_dict.items():
        pa = pa_list[0]
        all_pa_exec.append(find_exec_node(pa, ts, servs))
   
    stop = [False for _ in range(len(task_order))]
    i = 0
    while True:
        t = task_order[i]
        j = exec_time[i].index(max(exec_time[i])) # slowest robot
        # considering twist task and get the actual task name
        if t in twist_task and t not in alloc[j]:
            for ts in twist_task[t]:
                if ts in alloc[j]:
                    t = ts
                    break         
        pa, pa_init, pa_accept = pa_dict[j]
        start = pa_init
        pa_exec = all_pa_exec[j] # dict of sets of exec nodes for each alloced task
        cand = pa_exec[t]
        prev_time = 0
        max_time = exec_time[i][j]
        sums_time = sum(end_time)
        new_path = []
        k = alloc[j].index(t) # currently modified c_task idx
        cur = path[idxs[j][k]]
        cand.remove(cur)
        if k > 0:
            path = pa_path_list[j]
            prev = path[idxs[j][k-1]]
            # exec time of prev task, considering wait
            p_t = alloc[j][k-1]
            p_t = find_leader_t(task_order, twist_task, p_t)
            prev_time = exec_time[task_order.index(p_t)][j]               
            new_path += path[:idxs[j][k-1]+1]
            start = [prev]
        ## given: start(list),accept(list),cand(list), find feasible node in cand
        can_opt, new_path = cand_check(pa, start, cand, pa_accept, prev_time, max_time, new_path, ts,\
                                       s_pos, cost, alloc, idxs, task_order, twist_task, sums_time)
        if can_opt:
            pa_path_list[j] = new_path
            # renew cost, idxs, exectime
            cost[j] = compute_path_cost(pa, new_path)
            idx = find_exec_idx(pa, ts, path, alloc[j], s_pos)
            end_time, exec_time = compute_finish_time(cost, alloc, idxs, task_order, twist_task)                          
        else:
            if i == 0 or stop[i-1] == True:
                stop[i] = True
                if i == len(task_order) - 1:
                    break
        i = (i+1) % len(task_order)
        print(sums(end_time))
    return sums(end_time), end_time
        
        
    

if __name__ == "__main__":
    ## environment setting
    m, n = 30, 30
    # hronzital
    obs = [(10, 10+i) for i in range(4)]
    obs += [(10, 16+i) for i in range(4)]
    obs += [(19, 10+i) for i in range(4)]
    obs += [(19, 16+i) for i in range(4)]
    # vertical
    obs += [(11+i, 10) for i in range(3)]
    obs += [(16+i, 10) for i in range(3)]
    obs += [(11+i, 19) for i in range(3)]
    obs += [(16+i, 19) for i in range(3)]
    # task definition
    tasks_collection = [[0, 0, "t1"], [1, 7, "t2"], [7, 3, "t3"], [5, 11, "t4"],
                        [20, 0, "t5"], [21, 7, "t6"], [27, 3, "t7"], [25, 11, "t8"],
                        [0, 20, "t9"], [1, 27, "t10"], [7, 23, "t11"], [5, 29, "t12"],
                        [20, 20, "t13"], [21, 27, "t14"], [27, 23, "t15"], [25, 28, "t16"]]
    coor_tasks = [[12, 12, "s1"], [18, 11, "s2"], [13, 17, "s3"], [16, 18, "s4"]]
    s_pos = {}
    for x, y, t in coor_tasks: # s_pos is a relation function from c_task to its position
        s_pos[t] = (x, y)
    grid = [[[] for _ in range(n)] for _ in range(m)]
    for x, y in obs:
        grid[x][y].append("obs")
    for x, y, ap in tasks_collection:
        grid[x][y].append(ap)
    for x, y, ap in coor_tasks:
        grid[x][y].append(ap)
    
    T_start = time.time() # starting point of whole program

    ## environment transition system
    ts = grid2map(grid)
    print("TS successfully constructed.")

    ## tasks capability
    num_r = 4
    task_cap = {"s1": 1, "s2": 3, "s3": 2, "s4": 2}
    local_formula = ["(F t1) && (F t2) && (F t3) && (F t4) && (!t1 U t4)",
                     "(F t5) && (F t6) && (F t7) && (F t8) && (!t6 U t8)",
                     "(F t9) && (F t10) && (F t11) && (F t12) && (!t10 U t12)",
                     "(F t13) && (F t14) && (F t15) && (F t16) && (!t16 U t15)"]
    
    c_formula = '(F s1) && (F s2) && (F s4) && (!s3 U s2) && (G(s4 -> (F s3)))'
    
    ## decompose global task formula   
    ba, init_nodes, accept_nodes = ltl_formula_to_ba(c_formula) # convert to BA
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
        # print(decompose)   
        alpha = 0.1  # 在ba上搜索可以分割的路径
        path = task_decompose(ba, init_nodes, accept_nodes, decompose, alpha)
        # print(path)
        tasks = decompose_tasks(path, decompose)  #　根据路径分离任务
        # print(tasks)
        print("Finishing decompose tasks from global BA.")

        X = [[[[Int("x_%i_%i_%i_%i" % (i, j, k, l)) for l in range(len(tasks[j][k]))]
               for k in range(len(tasks[j]))]
              for j in range(len(tasks))]
             for i in range(num_r)]
        val_c = [And(0 <= X[i][j][k][l], X[i][j][k][l] <= 1) for i in range(num_r)
                 for j in range(len(tasks)) for k in range(len(tasks[j]))
                 for l in range(len(tasks[j][k]))]
        task_c = [sum([X[i][j][k][l] for i in range(num_r)]) >= task_cap[tasks[j][k][l]]
                  for j in range(len(tasks)) for k in range(len(tasks[j]))
                  for l in range(len(tasks[j][k]))]
        mutual_c = [sum([X[i][j][k][l] for l in range(len(tasks[j][k]))]) <= 1 for i in range(num_r)
                    for j in range(len(tasks)) for k in range(len(tasks[j]))]
        connect_c = [Or([Implies(Or([X[i][j][k][l] == 1 for l in range(len(tasks[j][k]))]),
                                 Or([X[i][j][k+1][l] == 1 for l in range(len(tasks[j][k+1]))]))
                         for i in range(num_r)]) for j in range(len(tasks)) for k in range(len(tasks[j])-1)]

        s = Solver()
        s.add(val_c + task_c + mutual_c + connect_c)
        count = 0
        last_res = [] # record all past useful solution to filt newly obtained solution
        finish_time = [] # record finishing time for all robots at each iteration
        best_alloc_path = None
        best_end = None
        print("Starting Iteration:")
        while s.check() == sat:
            count += 1 # useful solution idx, filt with pareto optimal 
            sm = s.model()
            res = [[[[sm.evaluate(X[i][j][k][l]) for l in range(len(tasks[j][k]))]
                     for k in range(len(tasks[j]))] for j in range(len(tasks))]
                   for i in range(num_r)]
            # 添加非命题
            f = And([X[i][j][k][l] == res[i][j][k][l] for i in range(num_r)
                     for j in range(len(tasks)) for k in range(len(tasks[j]))
                     for l in range(len(tasks[j][k]))])
            neg_f = Not(f)
            s.add(neg_f)
            ## 判断是否要计入对比
            res_list = [res[i][j][k][l].as_long() for i in range(num_r) for j in range(len(tasks))
                        for k in range(len(tasks[j])) for l in range(len(tasks[j][k]))]
            if last_res != None:
                if is_worse_smt(res_list, last_res):  # 新的解没意义
                    count -= 1
                    continue
                last_res = remove_useless_res(res_list, last_res) # remove old useless assignments
            print("*********************")
            print(f"Iteration {count}:")
            print("*********************")
            ## 根据当前分配结果，进行指标求解与对比
            last_res.append(res_list)
            alloc = cooperation_task(tasks, res_list) # list[list]. alloc contains each robot's task assignment

            coor_formula = construct_formula(alloc)
            ## solve local formual + global formula piece
            init_pos = [(3, 0), (22, 0), (3, 27), (24, 28)]
            # TODO: change ts from public to a private list
            path_cost, pa_dict = iteration(num_r, local_formula, coor_formula, ts, init_pos) # search optimal path for each robot          
            pa_path_list, ts_path_list = [], [] # contains optimal path for each robot 
            for min_cost, real_path, pa_path in path_cost:
                ts_path_list.append(real_path)
                pa_path_list.append(pa_path)
                
            ## animation
            # new_task_cap = task_cap.copy()
            # animate_path(m, n, ts_path_list, alloc, new_task_cap, obs, tasks_collection, coor_tasks) 
            ## compute end time

            cur_time = time.time()
            cost = [pc[0] for pc in path_cost]
            task_order = [t[0] for seg in tasks for t in seg] # all tasks listed in pre-defined execution order
            twist_task = {}
            for seg in tasks:
                for t in seg:
                    if len(t) > 1:
                        twist_task[t[0]] = t[1:]
            sums_time, end_time = optimise_time(pa_path_list, pa_dict, ts, cost, \
                                                alloc, task_order, twist_task, \
                                                s_pos, servs)           
            print(f"End of Iteration {count}. Program running {cur_time - T_start}s.")
        else:
            print("fail to solve")
    
    

## iteration Function record
```py
def iteration(MAS):
    """calculate optimal plan for each robot in MAS"""
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
        ts_path = extract_ts_path(pa, ts, pa_path)
        robot.set_pa_path(pa_path)
        robot.set_ts_path(ts_path)
        robot.set_path_cost(min_cost)    
        path_label = [] # print labels along the path
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
    return

```


```py
def concurrent_plan(robot):
    TIME_start = time.time()
    ts = robot.ts
    ba, init_nodes, accept_nodes = ltl_formula_to_ba(robot.f)
    print(robot.id, f"BA. Time: {time.time() - TIME_start}s. #node {len(ba)}: ")
    pa, pa_init, pa_accept = product_automaton(
        ts, ba, init_nodes, accept_nodes, init_pos=robot.pos)
    robot.set_pa(pa, pa_init, pa_accept)
    print(robot.id,
        f"PA. Time: {time.time() - TIME_start}s. #init: {len(pa_init)}. #accept: {len(pa_accept)}. #node: {len(pa)}.")
    source = []
    for init in pa_init:
        ts_node = pa.nodes[init]["ts"]
        if ts.nodes[ts_node]["pos"] == robot.pos:
            source.append(init)
    # TODO: 这里可以对pa_accept进行缩减，只保留task的位置对应的accept状态
    min_cost, pa_path = optimal_path_tree(
        pa, source, pa_accept)  # path contains pa states
    ts_path = extract_ts_path(pa, ts, pa_path)
    robot.set_pa_path(pa_path)
    robot.set_ts_path(ts_path)
    robot.set_path_cost(min_cost)    
    # path_label = [] # print labels along the path
    # for node in pa_path:
    #     ts_node = pa.nodes[node]["ts"]
    #     if ts.nodes[ts_node]["label"]:
    #         path_label.append(ts.nodes[ts_node]["label"])
    # print(robot.f)
    # print(path_label)
    print(f"Robot {robot.id} Time Used: {time.time()-TIME_start}s.")
    # animation: real_path is a list of real_path
    # grid_map(env, real_path, save_name = "min-cost-path")

def iteration(MAS):
    """calculate optimal plan for each robot in MAS"""
    start = time.time()
    my_thread = {}
    for i, robot in MAS.items():
        my_thread[i] = threading.Thread(target=concurrent_plan, args=(robot,))
    for i, t in my_thread.items():
        t.start()
    for i, t in my_thread.items():
        t.join()
    print("Finish PA construction.")
    print(f"Time for all PA: {time.time() - start}s.")
    return

```
## server classical planning results
```bash
TS list finish. 0.001308441162109375s.
PTS finish. #nodes: 13824. 6.402230262756348s.
ba #states: 108. 7.35369610786438s.
341 1492992
1122 1492992
4451 1492992
17838 1492992
53511 1492992
109334 1492992
173208 1492992
255582 1492992
374256 1492992
519962 1492992
683706 1492992
853183 1492992
1003088 1492992
1104657 1492992
1164239 1492992
1198478 1492992
1202574 1492992
1206670 1492992
pa #states: 1206670. 4653.323351383209s.
Search end. 4758.2435557842255s.
29
[(0, 0), (0, 0), (0, 0)]
[(0, 1), (0, 1), (0, 1)]
[(0, 1), (0, 2), (0, 2)]
[(0, 2), (1, 2), (1, 2)]
[(1, 2), (2, 2), (2, 2)]
[(2, 2), (3, 2), (3, 2)]
[(2, 3), (3, 3), (3, 3)]
[(3, 3), (3, 4), (2, 3)]
[(3, 2), (3, 3), (2, 4)]
[(3, 1), (3, 2), (1, 4)]
[(4, 1), (3, 1), (0, 4)]
***********************************
30

```
