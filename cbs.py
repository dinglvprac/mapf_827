import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
#from buildMDD import MDD
import copy


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    max_time = max(len(path1), len(path2))
    collision_list = []
    for time in range(max_time):
        if get_location(path1, time) == get_location(path2, time):
            collision_list.append({'loc': [get_location(path1, time)], 'timestep': time})
        if time == 0:
            continue
        else:
            if [get_location(path1, time-1), get_location(path1, time)] == [get_location(path2, time), get_location(path2, time-1)]:
                collision_list.append({'loc': [get_location(path1, time-1), get_location(path1, time)], 'timestep': time})
    return collision_list




def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collision_1st = []
    for i in range(len(paths)):
        for j in range(i+1, len(paths)):
            collision_list = detect_collision(paths[i], paths[j])
            for collision in collision_list:
                collision_1st.append({'a1': i, 'a2': j, 'loc': collision['loc'], 'timestep': collision['timestep']})
    return collision_1st



def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    if len(collision['loc']) ==1:
        split = [{'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']},
                 {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep']}]
    else:
        split = [{'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']},
                 {'agent': collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]], 'timestep': collision['timestep']}]
    return split


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    if random.randint(0, 1) == 0:
        if len(collision['loc']) == 1:
            split = [{'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': 0},
                     {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': 1}]
        else:
            split = [{'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': 0},
                     {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': 1}]
    else:
        if len(collision['loc']) == 1:
            split = [{'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': 0},
                     {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': 1}]
        else:
            split = [{'agent': collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]],
                      'timestep': collision['timestep'], 'positive': 0},
                     {'agent': collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]],
                      'timestep': collision['timestep'], 'positive': 1}]
    return split


def paths_violate_constraint(paths, constraint):
    t = constraint['timestep']
    agentsId = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        else:
            if constraint['loc'] == [get_location(paths[i], t)]:
                agentsId.append(i)
                continue
            if t == 0:
                continue
            if constraint['loc'] == [get_location(paths[i], t), get_location(paths[i], t-1)]:
                agentsId.append(i)
    return agentsId


def new_constraints(constraint, num):
    newc = []
    for i in range(num):
        if constraint['agent'] == i:
            continue
        if len(constraint['loc']) == 1:
            newc.append({'agent': i, 'loc': constraint['loc'], 'timestep': constraint['timestep'], 'positive': 0})
        else:
            newc.append({'agent': i, 'loc': [constraint['loc'][1], constraint['loc'][0]], 'timestep': constraint['timestep'],
                         'positive': 0})
    return newc


def check(constraints, nextp, map, edge, agent, timestep):
    if map[nextp[0]][nextp[1]]:
        return False
    for cst in constraints:
        if cst['agent'] == agent:
            if cst['timestep'] == timestep:
                if cst['loc'] == nextp or cst['loc'] == edge:
                    return False
    return True


def MDD(agent, cost, constraints, map, start, goal):
    layers = [[start]]
    parents = []
    for time in range(cost):
        layer = []
        parent = []
        for pos in layers[-1]:
            p1 = (pos[0], pos[1] - 1)
            p2 = (pos[0], pos[1] + 1)
            p3 = (pos[0] - 1, pos[1])
            p4 = (pos[0] + 1, pos[1])
            p = [pos, p1, p2, p3, p4]
            for nextp in p:
                if check(constraints, nextp, map, (pos, nextp), agent, time+1):
                    layer.append(nextp)
                    parent.append(pos)
        lrtemp = []
        prtemp = []
        for i in range(len(layer)):
            flag = 0
            for j in range(len(lrtemp)):
                if lrtemp[j] == layer[i]:
                    prtemp[j].append(parent[i])
                    flag = 1
                    break
            if flag == 0:
                lrtemp.append(layer[i])
                prtemp.append([parent[i]])
        layers.append(lrtemp)
        parents.append(prtemp)
    k = cost
    mdd_ls = [[goal]]
    index = [[layers[cost].index(goal)]]
    mdd_ps = []
    while k != 0:
        idx = index[cost-k]
        parent = parents[k-1]
        new_ps = []
        new_idx = []
        new_ls = []
        for i in idx:
            new_ps.append(parent[i])
            for prt in parent[i]:
                if layers[k-1].index(prt) in new_idx:
                    continue
                new_idx.append(layers[k-1].index(prt))
                new_ls.append(prt)
        mdd_ls.append(new_ls)
        mdd_ps.append(new_ps)
        index.append(new_idx)
        k = k-1
    mdd_ls.reverse()
    mdd_ps.reverse()
    return {'layers': mdd_ls, 'parents': mdd_ps}


def ICBS(collisions, paths, constraints, self):
    if len(collisions) == 1:
        return collisions[0]
    trees = []
    for i in range(self.num_of_agents):
        cost = len(paths[i])-1
        tree = MDD(i, cost, constraints, self.my_map, self.starts[i], self.goals[i])
        trees.append(tree)
    for collision in collisions:
        a1 = collision['a1']
        a2 = collision['a2']
        t = collision['timestep']
        tree1 = trees[a1]
        tree2 = trees[a2]
        if len(tree1['layers'][t]) == 1 and len(tree2['layers'][t]) == 1:
            return collision
    return collisions[0]




class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions for agent ' + str(i))

            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while len(self.open_list) > 0:
            curr = self.pop_node()
            if len(curr['collisions']) == 0:
                self.print_results(curr)
                return curr['paths']
            #collision = curr['collisions'][0]
            collision = ICBS(curr['collisions'], curr['paths'], curr['constraints'], self)
            if disjoint:
                constraints = disjoint_splitting(collision)
            else:
                constraints = standard_splitting(collision)
            for constraint in constraints:
                Q = {'cost': 0,
                     'constraints': [],
                     'paths': [],
                     'collisions': []}
                Q['constraints'] = copy.deepcopy(curr['constraints'])
                Q['constraints'].append(constraint)
                Q['paths'] = copy.deepcopy(curr['paths'])
                if disjoint:
                    if constraint['positive'] == 0:
                        ai = constraint['agent']
                        path = a_star(self.my_map, self.starts[ai], self.goals[ai], self.heuristics[ai], ai, Q['constraints'])
                        if path is None:
                            continue
                        Q['paths'][ai] = path
                    else:
                        newc = new_constraints(constraint, self.num_of_agents)
                        for c in newc:
                            Q['constraints'].append(c)
                        agentsId = paths_violate_constraint(curr['paths'], constraint)
                        ai = constraint['agent']
                        path = a_star(self.my_map, self.starts[ai], self.goals[ai], self.heuristics[ai], ai, Q['constraints'])
                        if path is None:
                            continue
                        Q['paths'][ai] = path
                        no_path = 0
                        for agent in agentsId:
                            path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent,
                                          Q['constraints'])
                            if path is None:
                                no_path = 1
                                break
                            Q['paths'][agent] = path
                        if no_path == 1:
                            continue
                else:
                    ai = constraint['agent']
                    path = a_star(self.my_map, self.starts[ai], self.goals[ai], self.heuristics[ai], ai, Q['constraints'])
                    if path is None:
                        continue
                    Q['paths'][ai] = path

                Q['collisions'] = detect_collisions(Q['paths'])
                Q['cost'] = get_sum_of_cost(Q['paths'])
                self.push_node(Q)
        raise BaseException('No solutions')

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
