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
    len_x += 1
    return map, len_x, len_y

class CustomGrid: # For robot navigation example
    def __init__(self, states, transitions):
        self.states = states
        self.transitions = transitions
        self.original_transitions = transitions
        self.map , self.inv_map= self.make_state_map()
        self.next_state_dict = self.make_next_state_dict()
        self.mapping = {'init': (0,1), 'd1': (1,0), 'd2': (1,1), 'd3': (1,2), 'goal': (2,1)} # to be modified for actual hardware implementation
        self.gamegraph = None
        self.cuts = []

    def make_state_map(self):
        map = {}
        inv_map = {}
        for i,state in enumerate(self.states):
            map.update({i : state})
            inv_map.update({state : i})
        return map, inv_map

    def make_next_state_dict(self):
        # st()
        next_state_dict = {}
        for state in self.states:
            next_states = [state]
            for transition in self.transitions:
                if transition[0] == state:
                    next_states.append(transition[1])
            next_state_dict.update({state: next_states})
        return next_state_dict

    def transition_specs(self, state_str):
        dynamics_spec = set()
        for state_nr in self.map.keys():
            next_steps_string = '('+state_str+' = '+str(state_nr)+')'
            # st()
            for item in self.next_state_dict[self.map[state_nr]][1:]:
                next_steps_string += '|| ('+state_str+' = '+str(self.inv_map[item])+')'
            dynamics_spec |= {'('+state_str+' = '+str(state_nr)+') -> X('+ next_steps_string +')'}
        return dynamics_spec

    def add_cut(self,cut):
        # st()
        self.cuts.append(cut)
        if cut in self.transitions:
            self.transitions.remove(cut)
        self.next_state_dict = self.make_next_state_dict()

    def remove_all_cuts(self):
        self.cuts = []
        self.transitions = self.original_transitions
        self.make_next_state_dict()




class Network:
    def __init__(self, mazefile):
        self.source = None
        self.goal = None
        self.intermediate = None
        self.next_state_dict = None
        self.map, self.len_x, self.len_y = create_network_from_file(mazefile)

    def transition_specs(self, x_str, y_str):
        # st()
        dynamics_spec = set()
        for ii in range(0,self.len_y):
            for jj in range(0,self.len_x):
                if not self.map[(ii,jj)] == '*':
                    start = self.next_state_dict[(ii,jj)][0]
                    next_steps_string = '('+y_str+' = '+str(start[0])+' && '+x_str+' = '+str(start[1])+')'
                    for item in self.next_state_dict[(ii,jj)][1:]:
                        next_steps_string = next_steps_string + '|| ('+y_str+' = '+str(item[0])+' && '+x_str+' = '+str(item[1])+')'
                    dynamics_spec |= {'('+y_str+' = '+str(ii)+' && '+x_str+' = '+str(jj)+') -> X(('+ next_steps_string +'))'}
                    # print('('+y_str+' = '+str(ii)+' && '+x_str+' = '+str(jj)+') -> X(('+ next_steps_string +'))')
        return dynamics_spec

    def print_network(self):
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

class MazeNetwork(Network):
    def __init__(self, mazefile):
        super().__init__(mazefile)
        self.gamegraph, self.states, self.next_state_dict = self.get_gamegraph()

    def get_gamegraph(self):
        self.print_network()
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

class CorridorNetwork(MazeNetwork):
    def __init__(self, mazefile, key1, key2):
        super().__init__(mazefile)
        self.gamegraph, self.states, self.next_state_dict = self.get_gamegraph()
        self.goal1_unlocked = False
        self.goal2_unlocked = False
        self.key1 = key1
        self.key2 = key2



class RoadNetwork(Network):
    def __init__(self, mazefile):
        super().__init__(mazefile)
        self.next_state_dict = self.find_next_state_dict()
        self.gamegraph, self.states = self.get_gamegraph()

    def get_gamegraph(self):
        # self.print_network()
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
        # make networkx graph
        G = nx.DiGraph()
        for key in self.next_state_dict.keys():
            for item in self.next_state_dict[key]:
                G.add_edge(key,item)
        return G, states

    def find_next_state_dict(self):
        next_state_dict = dict()
        for state in self.map:
            if self.map[state] != '*':
                new_states = []
                new_states.append(state)
                direction_x = self.map[state]
                direction_y = self.map[state]
                if self.map[state] == '+':
                    for i in range(0,self.len_y-1):
                        direction_y = self.map[(i,state[1])]
                        if direction_y != '+' and direction_y != '*':
                            break
                    for j in range(0,self.len_x-1):
                        direction_x = self.map[(state[0],j)]
                        if direction_x != '+' and direction_x != '*':
                            break
                # st()
                if direction_x == '←' and state[1] > 0:
                    new_state = (state[0], state[1]-1)
                    new_states.append(new_state)
                if direction_x == '→' and state[1] < self.len_x-1:
                    new_state = (state[0], state[1]+1)
                    new_states.append(new_state)
                if direction_y == '↓' and self.len_y-1 > state[0]:
                    new_state = (state[0]+1, state[1])
                    new_states.append(new_state)
                if direction_y == '↑' and state[0] > 0:
                    new_state = (state[0]-1, state[1])
                    new_states.append(new_state)
                next_state_dict.update({state: new_states})
        # st()
        return next_state_dict

    def SpecialRobotNavNetwork(Network):
        def __init__(self, mazefile):
            super().__init__(mazefile)
            self.next_state_dict = self.find_next_state_dict()
            self.gamegraph, self.states = self.get_gamegraph()

        def get_gamegraph(self):
            self.print_network()
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



if __name__ == '__main__':
    main_dir = os.getcwd()
    par_dir = os.path.dirname(main_dir)
    networkfile = par_dir + '/large_road_network.txt'
    road_network = RoadNetwork(networkfile)
    road_network.print_network()
    # st()
