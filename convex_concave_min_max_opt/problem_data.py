# Converting network flow formulations into convex-concave min-max optimization
# Vectors: x = [f1e/F, f2e/F, de/F, F/F], y = [f3e/F]
import sys
sys.path.append('..')
import numpy as np
from ipdb import set_trace as st
from collections import OrderedDict as od
import _pickle as pickle
import os
import networkx as nx
from components.road_network import RoadNetwork, create_network_from_file
import matplotlib.pyplot as plt

debug = False

def initialize(maze):
    G = nx.DiGraph(maze.gamegraph)
    # remove self loops
    edges = list(G.edges())
    for i,j in edges:
        if i == j:
            G.remove_edge(i,j)
    nodes = list(G.nodes())
    nodes_keys = {k: v for k,v in enumerate(nodes)}
    edges = list(G.edges())
    edges_keys = {k: e for k,e in enumerate(edges)}

    src = maze.source
    sink = maze.goal
    int = maze.intermediate
    vars_x = ['f1_e', 'f2_e', 'd_e', 'F']
    vars_y = ['f3_e']
    edges_dict = {k: 0 for k in edges} # Edges have flow.
    x = {k: edges_dict for k in vars_x}
    y = {k: edges_dict for k in vars_y}
    return x, y, src, sink, int, G, nodes_keys, edges_keys

# Prefix z: zeros
def objective(x,y,reg, edges_keys):
    ne = len(list(edges_keys.keys())) # number of edges
    zf1 = np.zeros((ne,1))
    zf2 = np.zeros((ne,1))
    zf3 = np.zeros((ne,1))
    zde = np.zeros((ne,1))
    ot = 1.0/ne*np.ones((ne,1))

    c1 = np.vstack((zf1, zf2, zde, ot)) # zf: zero vector for f, and ot: one vector for t
    c2 = zf3.copy()
    # Objective function: c1.T x + c2.T y
    return c1, c2

def feas_constraint(x,y):
    pass

def cut_constraint(x,y):
    pass

def conservation_constraint(x,y):
    pass

def capacity_constraint(x,y):
    pass
    
if __name__ == '__main__':
    # test
    grid = "small"
    main_dir = os.getcwd()
    par_dir = os.path.dirname(main_dir)
    if grid == "large":
        networkfile = par_dir + '/large_road_network.txt'
        src = (8,2)
        sink = (2,8)
        int = (5,5)

    elif grid == "small":
        networkfile = par_dir + '/road_network.txt'
        src = (4,2)
        sink = (2,0)
        int = (2,4)

    maze = RoadNetwork(networkfile)
