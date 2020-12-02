# translate nx graph to a graphviz graph

import spot
import re
from graphviz import Digraph
import networkx as nx
import heapq
from collections import deque
from gridmap_plot import grid_map


class Graph:
    def __init__(self):
        self.dot = Digraph()

    def title(self, str):
        self.dot.graph_attr.update(label=str)

    def node(self, name, label, init=False, accepting=False):
        num_peripheries = '2' if accepting else '1'
        color = "black"
        if init: 
            color = "blue"
        if accepting:
            color = "red"
        self.dot.node(name, label, shape='circle',
                      color=color, peripheries=num_peripheries)

    def edge(self, src, dst, label):
        self.dot.edge(src, dst, label)

    def show(self, graph_name):
        self.dot.render("./render_graph/"+graph_name, view=True)

    def save_render(self, path="./render_graph/", on_screen=True):
        self.dot.render(path, view=on_screen)

    def save_dot(self, path):
        self.dot.save(path)

    def __str__(self):
        return str(self.dot)


def ltl_formula_to_ba(formula):
    never_claim = spot.translate(formula, 'BA').to_str('spin')
    pat_state = re.compile(r'\w*_\w*')
    pat_transtion = re.compile(r'(?<=::.).*(?=.->)')
    pat_endstate = re.compile(r'(?<=goto.)\w*_\w*')
    start = None
    graph = nx.DiGraph()
    init_nodes = []
    accept_nodes = []
    for line in never_claim.split("\n"):
        line = line.strip(" ")
        if not line or line == "never {" or line == "}"or line[0:2] == "ne" or line[0:2] == "if" or line[0:2] == "fi":
            continue
        elif line == "skip":
            graph.add_edge(start, start, label='(1)')
        elif line[0:2] == "::":  # transition line
            pat2_str = pat_transtion.search(line)
            pat3_str = pat_endstate.search(line)
            graph.add_edge(start, pat3_str.group(), label=pat2_str.group())
        else:  # state line
            pat1_str = pat_state.search(line)
            start = pat1_str.group()
            graph.add_node(start, 
                           name=start, 
                           init=False, 
                           accept=False)
            if start.find("init") != -1:
                graph.nodes[start]["init"] = True
                init_nodes.append(start)
            if start.find("accept") != -1:
                graph.nodes[start]["accept"] = True
                accept_nodes.append(start)
    return graph, init_nodes, accept_nodes





def show_BA(graph, show=True, title="nx_to_grpahviz"):
    ret = Graph()
    ret.title(title)
    for node in list(graph.nodes):
        ret.node(
            node, node, init=graph.nodes[node]["init"], accepting=graph.nodes[node]["accept"])
    for edge in list(graph.edges):
        ret.edge(edge[0], edge[1], graph[edge[0]][edge[1]]["label"][1:-1])
    if show:
        ret.show(title)


def show_ts(graph, show=True, title="transition system"):
    ret = Graph()
    ret.title(title)
    for node in list(graph.nodes):
        ret.node(
            node, node, init=False, accepting=False)
    for edge in list(graph.edges):
        ret.edge(edge[0], edge[1], str(graph[edge[0]][edge[1]]["weight"]))
    if show:
        ret.show(title)





def grid2map(grid: list, x=0, y=0):
    """
    convert 3-dim grid into a grid_map digraph.
    directly remove obstacle cells.
    """
    m, n = len(grid), len(grid[0])
    map = nx.DiGraph()
    queue = [(x, y)]
    visited = [[False for _ in range(n)] for _ in range(m)]
    direct = [(-1, 0), [1, 0], [0, -1], [0, 1]]
    while queue:
        new_queue = []
        while queue:
            x, y = queue.pop()
            if visited[x][y]:
                continue
            visited[x][y] = True
            if "obs" in grid[x][y]:
                continue
            map.add_node(str((x, y)), 
                         name=str((x, y)), 
                         label=grid[x][y],
                         pos = (x, y))
            map.add_edge(str((x, y)), str((x, y)), weight = 0)
            for d_x, d_y in direct:
                cur_x = x + d_x
                cur_y = y + d_y
                if 0 <= cur_x < m and 0 <= cur_y < n and not visited[cur_x][cur_y]:
                    # avoid obstacles
                    if "obs" not in grid[cur_x][cur_y]:
                        new_queue.append((cur_x, cur_y))
                        map.add_edge(str((x, y)), str((cur_x, cur_y)), weight=1)
                        map.add_edge(str((cur_x, cur_y)), str((x, y)), weight=1)
        queue = new_queue
    return map

def extract_transition(label: str):
    pat_ap = re.compile(r"\w+")
    require = {}
    and_formula = label.split(" || ")
    for f in and_formula:
        ap_formula = f.split(" && ")
        enable, disable = set(), set()
        for ap in ap_formula:
            atom = pat_ap.search(ap).group()
            if "!" in ap:
                disable.add(atom)
            else:
                if atom != "1":
                    enable.add(atom)
        require[f] = (enable, disable)
    return require

def can_transit(label: str, aps: list, pre_require = None) -> bool:
    # can speed up by preprocessing
    require = extract_transition(label) if pre_require==None else pre_require
    aps_set = set(aps)
    for key in require.keys():
        enable, disable = require[key]
        if aps_set & disable:
            continue
        if enable - aps_set:
            continue
        return True
    return False
    
    

def product_automaton(ts: nx.DiGraph, ba: nx.DiGraph, init_nodes: list, accept_nodes: list):
    
#    bfs
#    ba_node -> {next_ba}
#    ts_node -> {next_ts}
#    check: |next_ba| * |next_ts|
    
    # preprocessing
    label_require = {}
    for start, end, label in list(ba.edges.data("label")):
        label_require[label] = extract_transition(label)

    pa = nx.DiGraph()
    pa_init, pa_accept = [], []
    queue = []
    for ba_node in init_nodes:
        for ts_node in list(ts.nodes()):
            queue.append([ts_node, ba_node])
    visited = set()
    while queue:
        new_queue = []
        while queue:
            ts_pre, ba_pre = queue.pop()
            cur = ts_pre + "+" + ba_pre
            if cur in visited:
                continue
            visited.add(cur)
            init, accept = ba.nodes[ba_pre]["init"], ba.nodes[ba_pre]["accept"]
            pa.add_node(cur, 
                        name = cur, 
                        ts = ts_pre,
                        ba = ba_pre,
                        label = ts.nodes[ts_pre]["label"], 
                        init = init, 
                        accept = accept)
            if init: pa_init.append(cur)
            if accept: pa_accept.append(cur)
                     
            next_ba = list(ba[ba_pre])
            next_ts = list(ts[ts_pre])
            for ba_suf in next_ba:
                label = ba[ba_pre][ba_suf]["label"]
                for ts_suf in next_ts:
                    success = ts_suf + "+" + ba_suf
#                    if success in visited:
#                        # considering self-loop and graph-loop
#                        pa.add_edge(cur, success, weight = ts[ts_pre][ts_suf]["weight"], 
#                                    label = ba[ba_pre][ba_suf]["label"])
#                        continue
                    if can_transit(label, ts.nodes[ts_suf]["label"], pre_require = label_require[label]):
                        if success not in visited:
                            new_queue.append([ts_suf, ba_suf])
                        pa.add_edge(cur, success, weight = ts[ts_pre][ts_suf]["weight"], 
                                    label = ba[ba_pre][ba_suf]["label"])
        queue = new_queue
    return pa, pa_init, pa_accept

def optimal_path(graph, source: list, target: list) -> tuple:
    min_cost = float("inf")
    path = []
    for init in source:
        for accept in target:
            try:
                cost = nx.dijkstra_path_length(graph, init, accept, weight = "weight")
                if cost < min_cost:
                    min_cost = cost
                    path = nx.dijkstra_path(graph, init, accept, weight = "weight")
            except nx.NetworkXNoPath:
                continue
    return min_cost, path

def multi_obj_optimal_path(pa, ts, ba, init: list, accept: list, alpha: float, belta: float, distance: dict):
    
    def pareto_le(a: tuple, b: tuple) -> bool:
        for i in range(len(a)):
            if a[i] > b[i]:
                return False
        return True
    accept_set = set(accept)
    heap = []
    labels = {}
    useless = set()
    
    for node in init:
        labels[node] = (set(), [])
        ts_node = pa.nodes[node]["ts"]
        dist = distance[ts_node]
        # cost: path_cost, max_dist, sum_dist
        cost = (0, dist, dist)
        # (J: int, cost: list, cur: str, father: str, idx: int)
        J = alpha * cost[0] + (1 - alpha) * (belta * cost[1] + (1 - belta) * cost[2])
        J = round(J, 3)
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
#            print(ret)
            break
        for post in list(pa[cur]):
            if post == cur: continue
            # 如果ts是原地停留，这里要控制dist不能增加
            new_cost = [cost[0], cost[1], cost[2]]
            new_cost[0] += pa[cur][post]["weight"] # edge
            post_ts = pa.nodes[post]["ts"]
            cur_ts = pa.nodes[cur]["ts"]
            if post_ts != cur_ts:
                new_cost[1] = max(new_cost[1], distance[post_ts])
                new_cost[2] += distance[post_ts]
            new_cost = tuple(new_cost)
            new_J = alpha * new_cost[0] + \
                    (1 - alpha) * (belta * new_cost[1] + (1 - belta) * new_cost[2])
            new_J = round(new_J, 3)
            new_temp = (new_J, new_cost, post, cur, len(labels[cur][1])-1)
            
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
            if not useful: continue
            # temp
            to_remove = []
            for old in labels[post][0]:
                if pareto_le(old[1], new_cost):
                    useful = False
                    break
                elif pareto_le(new_cost, old[1]):
                    to_remove.append(old)
            if not useful: continue 
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
#    print(ret)
#    for key, value in labels.items():
#        print(key)
#        print(value)
    return path

    
    
    
                        
if __name__ == "__main__":
    ## parameter setting
    # user defined max distance from service loc for nodes that can not reach service loc 
    MAX_DIST = 100
    # J = alpha * cost + (1-alpha) * (belta * dist + (1-belta) * sum_dist)
    alpha = 0.5 # weight of path cost versus potential response dist
    belta = 0.5 # weight of response dist versus sum of reponse dist

    ## environment setting
    m, n = 10, 12
    obs = [(2, 3), (3, 3), (4, 3), (5, 3)]
    tasks = [[3, 2, "t11"], [3, 11, "t12"]]
    servs = [[6, 7, "s1"], [6, 8, "s2"], [5, 6, "s3"]]
    grid = [[[] for _ in range(n)] for _ in range(m)]  
    for x, y in obs:
        grid[x][y].append("obs")
    for x, y, ap in tasks:
        grid[x][y].append(ap)
    for x, y, ap in servs:
        grid[x][y].append(ap)
    
    ## environment transition system
    ts = grid2map(grid)
#    show_ts(ts)

    ## local task specification
    formula = '(F(t11 && (F t12)))'
    ba, init_nodes, accept_nodes = ltl_formula_to_ba(formula)
#    show_BA(ba)
    
    ## product automaton
    pa, pa_init, pa_accept = product_automaton(ts, ba, init_nodes, accept_nodes)
#    show_BA(pa, show=True, title="product automaton")
    
    ## preprocessing: calcute distance to service locs
    service_loc = []
    for x, y, label in servs:
        service_loc.append((x, y))
    distance = {}
    for node in list(ts.nodes()):
        dist = 0
        cur_x, cur_y = ts.nodes[node]["pos"]
        cost = 0
        for ser_x, ser_y in service_loc:
            try:
                cost += nx.dijkstra_path_length(ts, str((cur_x, cur_y)), str((ser_x, ser_y)), weight = "weight")
            except nx.NetworkXNoPath:
                print("Service Can Not Reach")
                cost = MAX_DIST
        if len(service_loc) != 0:
            dist = cost/len(service_loc)
        round(dist, 2)
        distance[node] = dist
        
    ## multi-obj task and motion planning
    # J = alpha * cost + (1-alpha) * (belta * dist + (1-belta) * sum_dist)
    robot_pos = (3, 0)
    source = []
    for init in pa_init:
        ts_node = pa.nodes[init]["ts"]
        if ts.nodes[ts_node]["pos"] == robot_pos:
            source.append(init)
        
    ## min-cost path search
    min_cost, path = optimal_path(pa, source, pa_accept)
    # plot results
    print(min_cost)
    real_path1 = []
    for node in path:
        ts_node = pa.nodes[node]["ts"]
        print(ts_node)
        real_path1.append(ts.nodes[ts_node]["pos"])
    print("***********************************")
        
    ## multi-obj path search
    multi_obj_path = multi_obj_optimal_path(pa, 
                                            ts, 
                                            ba, 
                                            source, 
                                            pa_accept, 
                                            alpha, 
                                            belta, 
                                            distance)
    # plot results 
    real_path2= []
    for node in multi_obj_path:
        ts_node = pa.nodes[node]["ts"]
        print(ts_node)
        real_path2.append(ts.nodes[ts_node]["pos"])
        
    # animation
    grid_map(m, n, real_path1, obs, tasks, servs, save_name = "min-cost-path")
    grid_map(m, n, real_path2, obs, tasks, servs, save_name = "multi-obj-path")
    
    
        
    
        
    
        
    
    
            
        
    
