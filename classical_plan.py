# -*- coding: utf-8 -*-
from collections import defaultdict

from planning import *
from ltl_decomposition import Environment, optimal_path_tree


def product_ts(ts_list, envs, init=None):

    def combinations(idx, arr, res, visited):
        if idx == num_r:
            if str(arr) not in visited:
                res.append(arr[:])
            return
        for pos in next_qs[idx]:
            arr.append(pos)
            combinations(idx+1, arr, res, visited)
            arr.pop()
        return

    # how to tranverse all combinations
    num_r = len(ts_list)
    pts = nx.DiGraph()
    visited = set()
    queue = [[(0, 0) for _ in range(num_r)]] if not init else [init]
    direct = [(-1, 0), [1, 0], [0, -1], [0, 1]]
    while queue:
        new_queue = []
        while queue:
            nodes = queue.pop()
            q_name = str(nodes)
            if q_name in visited:
                continue
            visited.add(q_name)
            labels = set()
            label_cnt = defaultdict(int)
            for i in range(num_r):
                x, y = nodes[i]
                for l in envs[i].grid[x][y]:
                    labels.add(l)
                    label_cnt[l] += 1
            pts.add_node(q_name,
                         name=q_name,
                         label=list(labels),
                         label_cnt=label_cnt,
                         pos=nodes)
            pts.add_edge(q_name, q_name, weight=0)
            next_qs = []
            for i, cur in enumerate(nodes):
                x, y = cur
                m, n = envs[i].size
                next_qs.append([])
                for d_x, d_y in direct:
                    cur_x, cur_y = x + d_x, y + d_y
                    if 0 <= cur_x < m and 0 <= cur_y < n and "obs" not in envs[i].grid[cur_x][cur_y]:
                        next_qs[-1].append((cur_x, cur_y))
                next_qs[-1].append((x, y))
            res = []
            combinations(0, [], res, visited)
            for new_q in res:
                weight = 0
                for k in range(num_r):
                    if nodes[k] != new_q[k]:
                        weight += 1
                pts.add_edge(q_name, str(new_q), weight=weight)
                pts.add_edge(str(new_q), q_name, weight=weight)
            new_queue += res
        queue = new_queue
    return pts


def multi_product_automaton(ts: nx.DiGraph, ba: nx.DiGraph, init_nodes: list,
                            accept_nodes: list, task_cap: dict, init_pos=None):
    """"""
    # preprocessing
    label_require = {}
    for _, _, label in list(ba.edges.data("label")):
        label_require[label] = extract_transition(label)  # dic(dic)

    pa = nx.DiGraph()
    pa_init, pa_accept = [], []
    queue = []
    if init_pos == None:
        for ba_node in init_nodes:
            for ts_node in list(ts.nodes()):
                queue.append([ts_node, ba_node])
    else:
        for ba_node in init_nodes:
            queue.append([str(init_pos), ba_node])
    visited = set()
    nodes_cnt = len(ts) * len(ba)
    prev = 0
    while queue:
        if len(visited) - prev >= 200:
            print(len(visited), nodes_cnt)
            prev = len(visited)
        new_queue = []
        while queue:
            ts_pre, ba_pre = queue.pop()
            cur = ts_pre + "+" + ba_pre
            if cur in visited:
                continue
            visited.add(cur)
            init, accept = ba.nodes[ba_pre]["init"], ba.nodes[ba_pre]["accept"]
            pa.add_node(cur,
                        name=cur,
                        ts=ts_pre,
                        ba=ba_pre,
                        label=ts.nodes[ts_pre]["label"],
                        init=init,
                        accept=accept)
            if init:
                pa_init.append(cur)
            if accept:
                pa_accept.append(cur)

            next_ba = list(ba[ba_pre])
            next_ts = list(ts[ts_pre])
            for ba_suf in next_ba:
                label = ba[ba_pre][ba_suf]["label"]
                for ts_suf in next_ts:
                    success = ts_suf + "+" + ba_suf
                    if multi_can_transit(label,
                                         ts.nodes[ts_suf]["label"],
                                         ts.nodes[ts_suf]["label_cnt"],
                                         task_cap,
                                         pre_require=label_require[label]):
                        if success not in visited:
                            new_queue.append([ts_suf, ba_suf])
                        # considering loop, so success will be add to edge even it is in visited
                        pa.add_edge(cur, success, weight=ts[ts_pre][ts_suf]["weight"],
                                    label=ba[ba_pre][ba_suf]["label"])
        queue = new_queue
    return pa, pa_init, pa_accept


def multi_can_transit(label: str, aps: list, label_cnt: dict, task_cap: dict,
                      pre_require=None) -> bool:
    # can speed up by preprocessing. require: dic of dic
    require = extract_transition(label) if pre_require == None else pre_require
    aps_set = set(aps)
    coor_t = set(list(task_cap.keys()))
    for key in require.keys():
        enable, disable = require[key]
        if aps_set & disable:
            continue
        if enable - aps_set:
            continue
        can_coor = True
        for t in enable & coor_t:
            if label_cnt[t] < task_cap[t]:
                can_coor = False
                break
        if can_coor:
            return True
    return False


if __name__ == "__main__":
    start = time.time()
    # environment setting
    num_r = 3
    # hronzital
    h_obs = [(1, 1)]
    coor_tasks = [[0, 1, "s1"], [2, 2, "s2"]]
    task_cap = {"s1": 3, "s2": 2}
    tasks_collection = [[[2, 3, "t1"], [4, 1, "t2"]],
                        [[3, 4, "t3"], [3, 1, "t4"]],
                        [[3, 3, "t5"], [0, 4, "t6"]],
                        [[9, 4, "t7"], [2, 7, "t8"]]]
    ts_list = []
    envs = {}
    for i in range(num_r):
        envs[i] = Environment(5, 5)
        envs[i].add_obs(h_obs)
        envs[i].add_t(tasks_collection[i])
        envs[i].add_ct(coor_tasks)
        ts_list.append(grid2map(envs[i].grid))
    print(f"TS list finish. {time.time()-start}s.")
    # product ts
    pts = product_ts(ts_list, envs)
    print(f"PTS finish. #nodes: {len(pts)}. {time.time()-start}s.")
    # show_ts(pts)
    local_formula = ["(F(t1 && (F t2)))",
                     "(F(t3 && (F t4)))",
                     "(F(t5 && (F t6)))"]
    c_formula = "(F s1) && (F s2)"
    local_formula.append(c_formula)
    # local task specification
    formula = ' && '.join(local_formula)
    ba, init_nodes, accept_nodes = ltl_formula_to_ba(formula)
    print(f"ba #states: {len(ba)}. {time.time()-start}s.")
    # show_BA(ba)

    # T_ts_ba = time.time()

    # product automaton
    init_pos = [(0, 0) for _ in range(num_r)]
    pa, pa_init, pa_accept = multi_product_automaton(pts, ba, init_nodes,
                                                     accept_nodes, task_cap,
                                                     init_pos=init_pos)
    print(f"pa #states: {len(pa)}. {time.time()-start}s.")
    # show_BA(pa, show=True, title="product automaton")
    source = []
    for init in pa_init:
        ts_node = pa.nodes[init]["ts"]
        if pts.nodes[ts_node]["pos"] == init_pos:
            source.append(init)

    # min-cost path search
    # TODO: 这里可以对pa_accept进行缩减，只保留task的位置对应的accept状态
    min_cost, path = optimal_path_tree(pa, source, pa_accept)

    print(f"Search end. {time.time()-start}s.")

    # plot results
    print(min_cost)
    
    ts_path = []
    for node in path:
        ts_node = pa.nodes[node]["ts"]
        print(ts_node)
        ts_path.append(pts.nodes[ts_node]["pos"])
    print("***********************************")
    overall_time = (len(path) - 1) * num_r
    print(overall_time)
    # # T_pa = time.time()
    
    # how to compute the actual overall time
    # the other robot may wait for the last robot to come to coor place.
    # if last pos not in c_pos, then just count the non-repeated seq
    # if last pos in c_pos, then 

    # ## environment setting
    # num_r = 4
    # envs = {}
    # # hronzital
    # h_obs = [(10, 10+i) for i in range(4)]+[(10, 16+i) for i in range(4)] +\
    #         [(19, 10+i) for i in range(4)]+[(19, 16+i) for i in range(4)]
    # # vertical
    # v_obs = [(11+i, 10) for i in range(3)]+[(16+i, 10) for i in range(3)] +\
    #         [(11+i, 19) for i in range(3)]+[(16+i, 19) for i in range(3)]
    # coor_tasks = [[15, 6, "s1"], [25, 5, "s2"],
    #               [3, 3, "s3"], [16, 0, "s4"]]
    # # coor_tasks = [[17, 15, "s1"], [12, 12, "s2"],
    # #               [15, 15, "s3"], [13, 18, "s4"]]
    # task_cap = {"s1": 1, "s2": 3, "s3": 2, "s4": 2}
    # tasks_collection = [[0, 0, "t1_0"], [1, 7, "t2_0"], [7, 3, "t3_0"], [5, 11, "t4_0"],
    #                 [20, 0, "t5_1"], [21, 7, "t6_1"], [27, 3, "t7_1"], [25, 11, "t8_1"],
    #                 [0, 20, "t9_2"], [1, 27, "t10_2"], [7, 23, "t11_2"], [5, 29, "t12_2"],
    #                 [20, 20, "t13_3"], [21, 27, "t14_3"], [27, 23, "t15_3"], [25, 28, "t16_3"]]
    # ts_list = []
    # for i in range(num_r):
    #     envs[i] = Environment(30, 30)
    #     envs[i].add_obs(h_obs)
    #     envs[i].add_obs(v_obs)
    #     envs[i].add_t(tasks_collection)
    #     envs[i].add_ct(coor_tasks)
    #     ts_list.append(grid2map(envs[i].grid))
    # print("TS list successfully constructed.")

    # ## product ts
