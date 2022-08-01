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
from scipy import sparse as sp

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

def feas_constraint(x,y, edges_keys):
    ne = len(list(edges_keys.keys())) # number of edges
    Af1 = np.eye(ne)
    Af2 = np.eye(ne)
    Af3 = np.eye(ne)
    blk_zeros = np.zeros((ne,ne))

    b_feas = np.zeros((3*ne,))
    A_feas_f1= np.hstack((Af1, blk_zeros, blk_zeros, blk_zeros, blk_zeros))
    A_feas_f2= np.hstack((blk_zeros, Af2, blk_zeros, blk_zeros, blk_zeros))
    A_feas_f3= np.hstack((blk_zeros, blk_zeros, blk_zeros, blk_zeros, Af3))
    A_feas = np.vstack((A_feas_f1, A_feas_f2, A_feas_f3))
    return A_feas, b_feas

def cut_constraint(x,y, edges_keys):
    ne = len(list(edges_keys.keys())) # number of edges
    Af1 = -1*np.eye(ne)
    Af2 = -1*np.eye(ne)
    Af3 = -1*np.eye(ne)
    Ade = -1*np.eye(ne)
    Aot = np.eye(ne)
    blk_zeros = np.zeros((ne,ne))

    b_feas = np.zeros((3*ne,))
    A_feas_f1= np.hstack((Af1, blk_zeros, Ade, Aot, blk_zeros))
    A_feas_f2= np.hstack((blk_zeros, Af2, Ade, Aot, blk_zeros))
    A_feas_f3= np.hstack((blk_zeros, blk_zeros, Ade, Aot, Af3))
    A_feas = np.vstack((A_feas_f1, A_feas_f2, A_feas_f3))
    return A_feas, b_feas

def conservation_constraint(x,y, nodes_keys, edges_keys, src, int, sink):
    for k,node in nodes_keys.items():
        if node not in set(src, int, sink):
            # Afeas1: sum_i f^i(u,v) >= sum_i f^i(v,u)
            
            # Afeas2: sum_i f^i(u,v) <= sum_i f^i(v,u)

    Afeas = np.vstack((Afeas1, Afeas2))
    bfeas = np.vstack((bfeas1, bfeas2))
    return Afeas, bfeas

def min_flow_constraint(x,y,edges_keys, src, int,sink):
    ne = len(list(edges_keys.keys())) # number of edges
    out_s1_edge_ind = [k for k, v in edges_keys.items() if v[0]==src]
    out_s2_edge_ind = [k for k, v in edges_keys.items() if v[0]==int]
    out_s3_edge_ind = [k for k, v in edges_keys.items() if v[0]==src]

    zero_row_vec = np.zeros((1,ne))
    af1_ones = np.ones((len(ne),))
    af1 = sp.csr_array(af1_ones, (zero_row_vec, out_s1_edge_ind), shape=(1,ne))
    af2 = sp.csr_array(af2_ones, (zero_row_vec, out_s2_edge_ind), shape=(1,ne))
    af3 = sp.csr_array(af3_ones, (zero_row_vec, out_s3_edge_ind), shape=(1,ne))
    bfeas = np.array([[1],[1],[1]])
    Afeas = np.vstack((af1, af2, af3))
    return Afeas, bfeas

def capacity_constraint(x,y, edges_keys):
    ne = len(list(edges_keys.keys())) # number of edges
    Af1 = -1*np.eye(ne)
    Af2 = -1*np.eye(ne)
    Af3 = -1*np.eye(ne)
    Aot = np.eye(ne)
    blk_zeros = np.zeros((ne,ne))

    b_feas = np.zeros((3*ne,))
    A_feas_f1= np.hstack((Af1, blk_zeros, blk_zeros, Aot, blk_zeros))
    A_feas_f2= np.hstack((blk_zeros, Af2, blk_zeros, Aot, blk_zeros))
    A_feas_f3= np.hstack((blk_zeros, blk_zeros, blk_zeros, Aot, Af3))
    A_feas = np.vstack((A_feas_f1, A_feas_f2, A_feas_f3))
    return A_feas, b_feas

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
