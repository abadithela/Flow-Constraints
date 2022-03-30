import sys
sys.path.append('..')
import numpy as np
from ipdb import set_trace as st
from collections import OrderedDict as od
import _pickle as pickle
import os
import networkx as nx

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


if __name__ == '__main__':
    mazefile = 'maze.txt'
    maze = MazeNetwork(mazefile)
    # maze.print_maze()
    spec = maze.transition_specs()
    st()
