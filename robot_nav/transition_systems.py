# Robot navigation example.
# Transition system construction

import tulip
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys
sys.path.append('..')
from static_obstacle_maze.network import MazeNetwork
import networkx as nx
import gurobipy as gp
from gurobipy import GRB
from construct_automata import get_ba_ts, product_automaton, convert_grid_to_FTS
import pdb
# Transition system and Buchi automaton for the robot:
def get_ts_ba_robot(mazefile):
    maze = MazeNetwork(mazefile) # Creates the maze
    G, states, next_state_dict = maze.get_gamegraph() # Get finite transition system format.
    # pdb.set_trace()
    ts, state_map = convert_grid_to_FTS(G, states, next_state_dict, (0,maze.len_y-1), maze.len_x, maze.len_y)
    pdb.set_trace()

    system_spec = 
    # product_aut, preim_acc_states, ts_acc_states = product_automaton(ts, ba_orig)
    return ts

# Transition system for the tester:
def get_ts_ba_tester(mazefile):
    pass

if __name__ == '__main__':
    # Constructing the product automaton with the game graph
    # Convert to Buchi automaton:
    mazefile = "robot_nav.txt"
    ts = get_ts_ba_robot(mazefile)
