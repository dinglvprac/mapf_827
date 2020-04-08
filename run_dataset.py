#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost

map_file = 'mapf-map/den520d.map'
scen_file = 'scen-even/den520d-even-1.scen'
Mysolver = 'CBS'
disjoint = False
map_f = Path(map_file)
scen_f = Path(scen_file)
if not map_f.is_file():
    raise BaseException(map_file + " does not exist.")
map_f = open(map_file, 'r')
# first line: #rows #columns
line = map_f.readline()
line = map_f.readline()
rows = int(line.split(' ')[1])
line = map_f.readline()
columns = int(line.split(' ')[1])
# #rows lines with the map
my_map = []
line = map_f.readline()
for r in range(rows):
    line = map_f.readline()
    my_map.append([])
    for cell in line:
        if cell == '@' or cell == 'T':
            my_map[-1].append(True)
        elif cell == '.':
            my_map[-1].append(False)
map_f.close()
if not scen_f.is_file():
    raise BaseException(scen_file + " does not exist.")
scen_f = open(scen_file, 'r')
line = scen_f.readline()
starts = []
goals = []
skips = []
while len(starts) != 25:
    line = scen_f.readline()
    x1 = int(line.split('\t')[5])
    y1 = int(line.split('\t')[4])
    x2 = int(line.split('\t')[7])
    y2 = int(line.split('\t')[6])
    """
    if my_map[x1][y1]:
        continue
    if my_map[x2][y2]:
        continue
    flag = 0
    for skip in skips:
        if (x1, y1) == skip:
            flag = 1
            break
    if flag == 1:
        continue
    """
    starts.append((x1, y1))
    goals.append((x2, y2))
scen_f.close()

if Mysolver == "CBS":
   print("***Run CBS***")
   cbs = CBSSolver(my_map, starts, goals)
   paths = cbs.find_solution(disjoint)
elif Mysolver == "Independent":
   print("***Run Independent***")
   solver = IndependentSolver(my_map, starts, goals)
   paths = solver.find_solution()
elif Mysolver == "Prioritized":
   print("***Run Prioritized***")
   solver = PrioritizedPlanningSolver(my_map, starts, goals)
   paths = solver.find_solution()
else:
   raise RuntimeError("Unknown solver!")

cost = get_sum_of_cost(paths)

