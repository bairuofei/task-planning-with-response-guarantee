# -*- coding: utf-8 -*-

# tasks: m * n, m for m task segments, n for sequential task in each segments

# for every robot i, hava a variable x_i_m_n representing whether he has to execute tasks[m][n] :
# or not

from z3 import *

task_cap = {"a":2, "b":3, "c":3, "d":3, "e":3}
tasks = [["a", "b"], ["c", "d", "e"]]
m= len(tasks)

num_r = 3

X = [[[Int("x_%i_%i_%i" % (i,j,k)) for k in range(len(tasks[j]))] \
       for j in range(m)] for i in range(num_r)]

val_c = [And(0<= X[i][j][k], X[i][j][k] <= 1) for i in range(num_r) \
         for j in range(m) for k in range(len(tasks[j]))]

task_c = [sum([X[i][j][k] for i in range(num_r)]) >= task_cap[tasks[j][k]] \
          for j in range(m) for k in range(len(tasks[j]))]
        
# 同一个segment下，相邻的两个任务分配有交叉
connect_c = [Or([Implies(X[i][j][k] == 1, X[i][j][k+1] == 1) for i in range(num_r)])\
             for j in range(m) for k in range(len(tasks[j]) - 1)]
        
s = Solver()
s.add(val_c + task_c + connect_c)
count = 0
while s.check() == sat:
    print("Solution %i" % count)
    count += 1
    sm = s.model()
    res = [[[sm.evaluate(X[i][j][k]) for k in range(len(tasks[j]))] \
             for j in range(m)] for i in range(num_r)]
    
    print(res)
    f = And([X[i][j][k] == res[i][j][k] for i in range(num_r) \
             for j in range(m) for k in range(len(tasks[j]))])
    neg_f = Not(f)
    s.add(neg_f)
else:
    print("fail to solve")


