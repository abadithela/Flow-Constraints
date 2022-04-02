import sys
sys.path.append('..')
import numpy as np
from ipdb import set_trace as st
from collections import OrderedDict as od
import _pickle as pickle
import os
import networkx as nx
import gurobipy as gp
from gurobipy import GRB

def create_network_from_file(mazefile):
    map = od()
    f = open(mazefile, 'r')
    lines = f.readlines()
    len_y = len(lines)
    for i,line in enumerate(lines):
        for j,item in enumerate(line):
            if item != '\n' and item != '|':
                map[i,j] = item
                len_x = j
    return map, len_x, len_y


class MazeNetwork:
    def __init__(self, mazefile):
        self.source = None
        self.goal = None
        self.map, self.len_x, self.len_y = create_network_from_file(mazefile)
        self.gamegraph, self.states, self.next_state_dict = self.get_gamegraph()

    def get_gamegraph(self):
        self.print_maze()
        states = []
        # create states
        for ii in range(0, self.len_y + 1):
            for jj in range(0, self.len_x + 1):
                if (ii,jj) in self.map:
                    if self.map[(ii,jj)] != '*':
                        states.append((ii,jj))
                    if self.map[(ii,jj)] == 'S':
                        self.source = (ii,jj)
                    elif self.map[(ii,jj)] == 'G':
                        self.goal = (ii,jj)
        # create edges
        next_state_dict = dict()
        for node in states:
            next_states = [(node[0] ,node[1])]
            for a in [-1,1]:
                if (node[0] + a ,node[1]) in states:
                    next_states.append((node[0]+a,node[1]))
            for b in [-1,1]:
                if (node[0],node[1] + b) in states:
                    next_states.append((node[0],node[1]+b))
            next_state_dict.update({node: next_states})
        # make networkx graph
        G = nx.DiGraph()
        for key in next_state_dict.keys():
            for item in next_state_dict[key]:
                G.add_edge(key,item)
        return G, states, next_state_dict


    def print_maze(self):
        key_y_old = []
        printline = ""
        for key in self.map:
            key_y_new = key[0]
            if key_y_new == key_y_old:
                printline += self.map[key]
            else:
                print(printline)
                printline = self.map[key]
            key_y_old = key_y_new
        print(printline)
        printline = self.map[key]

    def transition_specs(self):
        dynamics_spec = set()
        for ii in range(0,self.len_y):
            for jj in range(0,self.len_x):
                if not self.map[(ii,jj)] == '*':
                    next_steps_string = ''
                    for item in self.next_state_dict[(ii,jj)]:
                        next_steps_string = next_steps_string + '|| (x = '+str(item[0])+' && y = '+str(item[1])+')'
                    dynamics_spec |= {'(x = '+str(ii)+' && y = '+str(jj)+') -> X(('+ next_steps_string +'))'}
                    print('(x = '+str(ii)+' && y = '+str(jj)+') -> X(('+ next_steps_string +'))')
        return dynamics_spec

class AugmentedNetwork(MazeNetwork):
    def __init__(self, mazefile, comp_wpt_i):
        super().__init__(mazefile)
        self.comp_wpt_i = comp_wpt_i
        self.partition = od()


    def find_critical_path_combination(self):
        # p_si_set = nx.all_simple_paths(self.gamegraph, self.source, self.comp_wpt_i)
        # p_ig_set = nx.all_simple_paths(self.gamegraph, self.comp_wpt_i, self.goal)
        # st()
        path_si = nx.shortest_path(self.gamegraph, self.source, self.comp_wpt_i)
        path_ig = nx.shortest_path(self.gamegraph, self.comp_wpt_i, self.goal)
        # st()
        # pick a combination
        H_crit = self.gamegraph.subgraph(path_si + path_ig)
        # st()
        # check for no shared edges in the subgraph
        # H2 = nx.Graph(H_crit)
        # H2.remove_node(self.comp_wpt_i)
        # if nx.has_path(H2, self.source, self.goal):
        #     print('This is the wrong path combination')
        #     st()
        # H_aug = nx.Graph(self.gamegraph)
        # H_aug.remove_node(self.comp_wpt_i)
        # cut_nodes = nx.minimum_node_cut(H_aug, self.source, self.comp_wpt_i)

        self.minimum_node_cut_with_protected_critical_path(H_crit)

    def minimum_node_cut_with_protected_critical_path(self, H_crit):
        # create model
        # st()
        k = 2 # partition into two segments
        G = nx.Graph(self.gamegraph)
        G.remove_node(self.comp_wpt_i)
        m = gp.Model()

        # create variables
        x = m.addVars(G.nodes, k, vtype=GRB.BINARY)
        y = m.addVars(G.edges, vtype=GRB.BINARY)  # y[u,v] equals one when edge {u,v} is cut
        m.setObjective(gp.quicksum(y[u,v] for u,v in G.edges), GRB.MINIMIZE) # minimize the number of cut edges
        # add constraints
        # st()
        # source and goal need to be in different partitions
        m.addConstr(x[self.source[0],self.source[1],0] == 1)
        m.addConstr(x[self.source[0],self.source[1],1] == 0)
        m.addConstr(x[self.goal[0],self.goal[1],1] == 1)
        m.addConstr(x[self.goal[0],self.goal[1],0] == 0)
        # do not delete self loop edges
        m.addConstrs(y[u,v] == 0 for u,v in G.edges if u == v)
        # other constraints
        m.addConstrs(gp.quicksum(x[i[0],i[1],j] for j in range(k)) == 1 for i in G.nodes) # each node is only part of one partition
        m.addConstrs(x[i[0],i[1],v] - x[j[0],j[1],v] <= y[i,j] for i,j in G.edges for v in range(k)) # cut the edge between partitions
        m.addConstrs(y[u,v] == 0 for u,v in G.edges if u in H_crit.nodes and v in H_crit.nodes) # do not cut in protected area

        m.update()
        # solve IP model
        m.optimize()
        st()

        for item in x:
            if x[item].x > 0.5:
                print('node {0},{1} belongs to {2}'.format(item[0],item[1],item[2]))
                self.partition.update({(item[0],item[1]) : item[2]})

        for item in y:
            if y[item].x > 0.5:
                print('We cut the edge between {0} and {1}'.format(item[0],item[1]))

        self.print_maze()
        self.print_partition()
        # st()

    def print_partition(self):
        # st()
        key_y_old = []
        printline = ""
        for key in sorted(self.map.keys(),key = lambda i:(i[0],i[1])):
            key_y_new = key[0]
            if key_y_new == key_y_old:
                if key not in self.partition:
                    if key == self.comp_wpt_i:
                        printline += 'I'
                    else:
                        printline += '*'
                else:
                    printline += str(self.partition[key])
            else:
                print(printline)
                if key not in self.partition:
                    if key == self.comp_wpt_i:
                        printline = 'I'
                    else:
                        printline = '*'
                else:
                    printline = str(self.partition[key])
            key_y_old = key_y_new
        print(printline)
        printline = str(self.partition[key])


if __name__ == '__main__':
    mazefile = 'maze.txt'
    maze = MazeNetwork(mazefile)
    augmented_maze = AugmentedNetwork(mazefile, (2,2))
    augmented_maze.find_critical_path_combination()
    # maze.print_maze()
    # spec = maze.transition_specs()
    # st()
