# -*- coding: utf-8 -*-

# tasks: m * n, m for m task segments, n for sequential task in each segments

# for every robot i, hava a variable x_i_m_n representing whether he has to execute tasks[m][n] :
# or not

from z3 import *



task_cap = {"a":2, "b":3, "c":5, "d":5, "e":5}
tasks = [[["a", "b"]], [["c"], ["d"], ["e"]]]
m= len(tasks)

num_r = 5

X = [[[[Int("x_%i_%i_%i_%i" % (i, j, k, l)) for l in range(len(tasks[j][k]))] \
        for k in range(len(tasks[j]))] \
        for j in range(len(tasks))] \
        for i in range(num_r)]

val_c = [And(0 <=X[i][j][k][l], X[i][j][k][l] <= 1) for i in range(num_r)\
         for j in range(len(tasks)) for k in range(len(tasks[j]))\
         for l in range(len(tasks[j][k]))]

task_c = [sum([X[i][j][k][l] for i in range(num_r)]) >= task_cap[tasks[j][k][l]]\
          for j in range(len(tasks)) for k in range(len(tasks[j]))\
          for l in range(len(tasks[j][k]))]

mutual_c = [sum([X[i][j][k][l] for l in range(len(tasks[j][k]))]) <= 1 for i in range(num_r)\
            for j in range(len(tasks)) for k in range(len(tasks[j]))]
        
connect_c = [Or([Implies(Or([X[i][j][k][l] == 1 for l in range(len(tasks[j][k]))]), \
                         Or([X[i][j][k+1][l] == 1 for l in range(len(tasks[j][k+1]))]))\
                for i in range(num_r)]) for j in range(len(tasks)) for k in range(len(tasks[j])-1)]
        
s = Solver()
s.add(val_c + task_c + mutual_c + connect_c)
count = 0
while s.check() == sat:
    print("Solution %i" % count)
    count += 1
    sm = s.model()
    res = [[[[sm.evaluate(X[i][j][k][l]) for l in range(len(tasks[j][k]))]\
            for k in range(len(tasks[j]))] for j in range(len(tasks))]\
            for i in range(num_r)]
    print(res)
    f = And([X[i][j][k][l] == res[i][j][k][l] for i in range(num_r)\
             for j in range(len(tasks)) for k in range(len(tasks[j]))\
             for l in range(len(tasks[j][k]))])
    neg_f = Not(f)
    s.add(neg_f)
else:
    print("fail to solve")


