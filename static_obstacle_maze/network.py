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

    def minimum_node_cut_with_protected_critical_path(self, H_crit_in):
         # create model
        k = 3 # partition into two segments
        G = nx.Graph(self.gamegraph)
        G.remove_node(self.comp_wpt_i)
        H_crit = nx.Graph(H_crit_in)
        H_crit.remove_node(self.comp_wpt_i)
        m = gp.Model()

        # create variables
        x = m.addVars(G.nodes, k, vtype=GRB.BINARY)
        m.setObjective(gp.quicksum(x[u[0],u[1],2] for u in G.nodes), GRB.MINIMIZE) # minimize the number of nodes in partition 2

        # add constraints
        # source and goal need to be in different partitions
        m.addConstr(x[self.source[0],self.source[1],0] == 1)
        m.addConstr(x[self.source[0],self.source[1],1] == 0)
        m.addConstr(x[self.goal[0],self.goal[1],1] == 1)
        m.addConstr(x[self.goal[0],self.goal[1],0] == 0)
        # other constraints
        m.addConstrs(gp.quicksum(x[i[0],i[1],j] for j in range(k)) == 1 for i in G.nodes) # each node is only part of one partition

        # make sure the direct neighbors are in the same partition or in partition 2
        # st()
        for node in G.nodes():
            neighbors = []
            for j in [-1,1]:
                if ((node[0]+j,node[1]) in G.nodes()):
                    neighbors.append((node[0]+j,node[1]))
            for k in [-1,1]:
                if ((node[0],node[1]+k) in G.nodes()):
                    neighbors.append((node[0],node[1]+k))
            # print('Node {0} and neighbors are {1}'.format(node, neighbors))
            for neighbor in neighbors:
                m.addConstr((x[node[0],node[1],0] == 1) >> (x[neighbor[0],neighbor[1],1] == 0))
                m.addConstr((x[node[0],node[1],1] == 1) >> (x[neighbor[0],neighbor[1],0] == 0))

        m.addConstrs(x[node[0],node[1],2] == 0 for node in H_crit.nodes) # do not cut in protected area

        m.update()
        # solve IP model
        m.optimize()
        # st()

        for item in x:
            if x[item].x > 0.5:
                print('node {0},{1} belongs to {2}'.format(item[0],item[1],item[2]))
                self.partition.update({(item[0],item[1]) : item[2]})

        self.print_maze()
        self.print_partition()

    def minimum_edge_cut_with_protected_critical_path(self, H_crit):
         # create model
        k = 2 # partition into two segments
        G = nx.Graph(self.gamegraph)
        G.remove_node(self.comp_wpt_i)
        m = gp.Model()

        # create variables
        x = m.addVars(G.nodes, k, vtype=GRB.BINARY)
        y = m.addVars(G.edges, vtype=GRB.BINARY)  # y[u,v] equals one when edge {u,v} is cut
        m.setObjective(gp.quicksum(y[u,v] for u,v in G.edges), GRB.MINIMIZE) # minimize the number of cut edges
        # add constraints
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

    def minimum_node_cut_with_protected_critical_path_v2(self, H_crit):
        # create model
        k = 2 # partition into two segments
        G_a = nx.create_empty_copy(self.gamegraph)
        G_a.remove_node(self.comp_wpt_i)
        G_a = nx.relabel_nodes(G_a, lambda x: str(x)+'_a')

        G_b = nx.create_empty_copy(self.gamegraph)
        G_b.remove_node(self.comp_wpt_i)
        G_b = nx.relabel_nodes(G_b, lambda x: str(x)+'_b')

        G = self.gamegraph
        G.remove_node(self.comp_wpt_i)

        G_c = nx.compose(G_a,G_b)

        for edge in G.edges():
            if edge[0] != edge[1]:
                G_c.add_edge(str(edge[0])+'_b', str(edge[1])+'_a')

        for node in G.nodes():
            G_c.add_edge(str(node)+'_a', str(node)+'_b')

        # protected nodes
        protected = []
        for node in H_crit.nodes():
            protected.append(str(node)+'_a')
            protected.append(str(node)+'_b')
        m = gp.Model()

        # create variables
        x = m.addVars(G_c.nodes, k, vtype=GRB.BINARY)
        y = m.addVars(G_c.edges, vtype=GRB.BINARY)  # y[u,v] equals one when edge {u,v} is cut
        m.setObjective(gp.quicksum(y[u,v] for u,v in G_c.edges), GRB.MINIMIZE) # minimize the number of cut edges

        # add constraints
        # source and goal need to be in different partitions
        m.addConstr(x[str(self.source)+'_a',0] == 1)
        m.addConstr(x[str(self.source)+'_a',1] == 0)
        m.addConstr(x[str(self.source)+'_b',0] == 1)
        m.addConstr(x[str(self.source)+'_b',1] == 0)
        m.addConstr(x[str(self.goal)+'_a',1] == 1)
        m.addConstr(x[str(self.goal)+'_a',0] == 0)
        m.addConstr(x[str(self.goal)+'_b',1] == 1)
        m.addConstr(x[str(self.goal)+'_b',0] == 0)

        # do not delete normal edges
        # m.addConstrs(y[u,v] == 0 for u,v in G_c.edges if u in G_b.nodes and v in G_a.nodes)

        # other constraints
        m.addConstrs(gp.quicksum(x[i,j] for j in range(k)) == 1 for i in G_c.nodes) # each node is only part of one partition
        m.addConstrs(x[i,v] - x[j,v] <= y[i,j] for i,j in G_c.edges for v in range(k)) # cut the edge between partitions

        # do not cut in protected area
        m.addConstrs(y[u,v] == 0 for u,v in G_c.edges if u in protected and v in protected)

        m.update()
        # solve IP model
        m.optimize()
        # print (m.display())
        # m.write("file.lp")
        st()

        for item in x:
            if x[item].x > 0.5:
                print('node {0} belongs to {1}'.format(item[0],item[1]))
                self.partition.update({(item[0]) : item[1]})

        for item in y:
            if y[item].x > 0.5:
                print('We cut the edge between {0} and {1}'.format(item[0],item[1]))

        st()
        self.print_maze()
        self.print_partition()

    def minimum_node_cut_with_protected_critical_path_v3(self, H_crit_in):
        # create model
        # st()
        k = 3 # partition into 3 segments
        G = nx.Graph(self.gamegraph)
        G.remove_node(self.comp_wpt_i)
        H_crit = nx.Graph(H_crit_in)
        H_crit.remove_node(self.comp_wpt_i)
        m = gp.Model()

        # create variables
        x = m.addVars(G.nodes, k, vtype=GRB.BINARY)
        y = m.addVars(G.edges, vtype=GRB.BINARY)  # y[u,v] equals one when edge {u,v} is cut
        z = m.addVars(G.nodes, vtype=GRB.BINARY)  # y[u,v] equals one when node is cut
        m.setObjective(gp.quicksum(z[u] for u in G.nodes), GRB.MINIMIZE) # minimize the number of cut nodes
        # add constraints
        # st()
        # source and goal need to be in different partitions
        m.addConstr(x[self.source[0],self.source[1],0] == 1)
        m.addConstr(x[self.source[0],self.source[1],1] == 0)
        m.addConstr(x[self.source[0],self.source[1],2] == 0)
        m.addConstr(x[self.goal[0],self.goal[1],2] == 1)
        m.addConstr(x[self.goal[0],self.goal[1],1] == 0)
        m.addConstr(x[self.goal[0],self.goal[1],0] == 0)

        # do not delete self loop edges
        m.addConstrs(y[u,v] == 0 for u,v in G.edges if u == v)
        m.addConstrs(z[u] == 0 for u in H_crit.nodes) # do not cut nodes in protected area
        m.addConstrs(y[u,v] == 0 for u,v in G.edges if u in H_crit.nodes and v in H_crit.nodes) # do not cut in protected area


        m.addConstrs(gp.quicksum(x[i[0],i[1],j] for j in range(k)) == 1 for i in G.nodes) # each node is only part of one partition

        m.addConstrs(x[i[0],i[1],v] - x[j[0],j[1],v] <= y[i,j] for i,j in G.edges for v in range(k)) # cut the edge between partitions

        m.addConstrs(y[i,j] == gp.abs_(z[i] - z[j]) for i,j in G.edges for v in range(k)) # cut the edge between partitions
        #
        # m.addConstrs( y[i,j] == gp.abs_(z[i] - z[j]) for i,j in G.edges for v in range(k)) # cut the edge between partition
        #
        # m.addConstr(y[i,j] == gp.abs_(x[i[0],i[1],v] - x[j[0],j[1],v]) for i,j in G.edges for v in range(k))
        m.update()
        # solve IP model
        m.optimize()
        st()

        for item in y:
            if y[item].x > 0.5:
                print('node {0} was cut'.format((item[0],item[1])))

        for item in x:
            if x[item].x > 0.5:
                print('node {0},{1} belongs to {2}'.format(item[0],item[1],item[2]))
                self.partition.update({(item[0],item[1]) : item[2]})

        # for item in y:
        #     if y[item].x > 0.5:
        #         print('We cut the edge between {0} and {1}'.format(item[0],item[1]))

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
