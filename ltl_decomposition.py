# -*- coding: utf-8 -*-label_require

import sys
from planning import *
from z3 import *
from collections import deque, defaultdict
import time
from gridmap_plot import *

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
    for i in range(num_r):
        print(f"Robot {i}:")
        TIME_start = time.time()
        robot_pos = init_pos[i]  # initial pos of robot 
        new_f = local_formula[i] + " && " + coor_formula[i]
        ba, init_nodes, accept_nodes = ltl_formula_to_ba(new_f)
        print(f"BA. Time: {time.time() - TIME_start}s. #node {len(ba)}: ")
        pa, pa_init, pa_accept = product_automaton(ts, ba, init_nodes, accept_nodes, init_pos = robot_pos)
        print(f"PA. Time: {time.time() - TIME_start}s. #init: {len(pa_init)}. #accept: {len(pa_accept)}. #node: {len(pa)}.")
            
        source = []
        for init in pa_init:
            ts_node = pa.nodes[init]["ts"]
            if ts.nodes[ts_node]["pos"] == robot_pos:
                source.append(init)    
        # TODO: 这里可以对pa_accept进行缩减，只保留task的位置对应的accept状态
        min_cost, path = optimal_path_tree(pa, source, pa_accept) # path contains pa states
        # print(min_cost)
        real_path, path_label = [], []
        for node in path:
            ts_node = pa.nodes[node]["ts"]
            real_path.append(ts.nodes[ts_node]["pos"]) # consisting of position coordination
            if ts.nodes[ts_node]["label"]:
                path_label.append(ts.nodes[ts_node]["label"])
        path_cost.append([min_cost, real_path])                                                                                         
        print(new_f)
        print(path_label)
        print(f"Time Used: {time.time()-TIME_start}s.")   
        print("---------------------------------------------")
        # animation: real_path is a list of real_path
        # grid_map(m, n, real_path, obs, tasks_collection, coor_tasks, save_name = "min-cost-path")
    return path_cost


def compute_finishing_time(path_cost: list, alloc: list, coor_tasks: list, task_order: list)-> list:
    """ 
    task- robot: dic(list)
    task- time: dic(list)
    robot - task - time: list(dic)
    """
    n = len(path_cost)
    res = []
    addtion = [0 for _ in range(n)] # addtion of time maintained by each robot
    task_r = defaultdict(list)  # task to robot dic. allocation relation
    for i, tasks in enumerate(alloc):
        for t in tasks:
            task_r[t].append(i)
    coor_dic = {}
    for x, y, label in coor_tasks:
        coor_dic[(x, y)] = label
    
    r_tsk_tm = []
    for i, c_p in enumerate(path_cost): # path contains pos coordination
        cost, path = c_p
        r_tsk_tm.append({})
        res.append(cost)
        cur_cost = 0
        for j, pos in enumerate(path):
            if j != 0:
                cur_cost += 1         
            if pos in coor_dic:
                r_tsk_tm[i][coor_dic[pos]] = cur_cost
    for ct in task_order:
        latest = 0
        for r in task_r[ct]:
            latest = max(latest, r_tsk_tm[r][ct] + addtion[r]) # actual latest time
        for r in task_r[ct]:
            addtion[r] = max(addtion[r], latest - r_tsk_tm[r][ct])
    for i in range(n):
        res[i] += addtion[i]
    return res

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
    
    c_formula = '(F s1) && (F s2) && (F s4) && (!s3 U s2) && (G(s4 -> (F s3))) '
    
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
        task_order = [t[0] for seg in tasks for t in seg]
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
            # 判断是否要计入对比
            res_list = [res[i][j][k][l].as_long() for i in range(num_r) for j in range(len(tasks))
                        for k in range(len(tasks[j])) for l in range(len(tasks[j][k]))]
            if last_res != None:
                if is_worse_smt(res_list, last_res):  # 新的解没意义
                    count -= 1
                    continue
                last_res = remove_useless_res(res_list, last_res)
            print("*********************")
            print(f"Iteration {count}:")
            print("*********************")
            # 根据当前分配结果，进行指标求解与对比
            last_res.append(res_list)
            alloc = cooperation_task(tasks, res_list) # list[list]. alloc contains each robot's task assignment

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
            ## solve local formual + global formula piece
            # init_pos = [(3, 0) for _ in range(num_r)]
            init_pos = [(3, 0), (22, 0), (3, 27), (24, 28),]
            path_cost = iteration(num_r, local_formula, coor_formula, ts, init_pos) # search optimal path for each robot          
            # plot path
            path_list = []
            for min_cost, real_path in path_cost:
                path_list.append(real_path)
            new_task_cap = task_cap.copy()
            animate_path(m, n, path_list, alloc, new_task_cap, obs, tasks_collection, coor_tasks)     
            end_list = compute_finishing_time(path_cost, alloc, coor_tasks, task_order)
            if best_end == None or best_end > min(end_list):
                best_end = min(end_list)
                best_alloc_path = path_cost
            print(best_end)
            finish_time.append(end_list) 
            cur_time = time.time()
            print(f"End of Iteration {count}. Program running {cur_time - T_start}s.")
        else:
            print("fail to solve")
    ## plot finish time
    # animation
    
    


