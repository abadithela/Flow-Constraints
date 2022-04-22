import networkx as nx
import numpy as np
import gurobipy as gp
from gurobipy import GRB
import sys
from lp import *
import scipy.sparse as sp
sys.path.append('..')
from static_obstacle_maze.network import MazeNetwork

# Construct matrices:
# x = [--fp--, F, --de--]

G, nodes, source_dict, sink_dict = load_graph()
Edges = G.edges()
Verts = G.nodes()
nV = len(Verts)
Edict = {k: Edges(k) for k in range(len(Edges))}
Edict_rev = {v:k for k,v in Edict.items()}
Vdict = {k: Verts(k) for k in range(len(Verts))}
Vdict_rev = {v:k for k,v in Vdict.items()}
numx = len(E)*3 + 1 + len(E)

def load_graph(m, n):
    G = nx.DiGraph()
    nodes = []
    edges = []
    source_dict = {'s1': "c91", 's2': "c55", 's3': "c91"}
    sink_dict = {'t1': "c55", 't2': "c10", 't3': "c10"}
    for row in range(1,m+1):
        for col in range(1,n+1):
            cell = (row-1)*n + col
            succ_cells = [cell-n, cell+n, cell-1, cell+1]
            if row == 1:
                succ_cells.remove(cell-n)
            if col == 1:
                succ_cells.remove(cell-1)
            if row == m:
                succ_cells.remove(cell+n)
            if col == n:
                succ_cells.remove(cell+1)
            nodes.append("c"+str(cell))
            for scell in succ_cells:
                edges.append(("c"+str(cell), "c"+str(scell)))
    G.add_edges_from(edges) # Add edges
    return G, nodes, source_dict, sink_dict

def source_outgoing_edge_indices(k=1):
    pass

def construct_objective(E):
    '''
    Constructs objective function as F
    '''
    nE = len(E)
    c = np.vstack((np.zeros((3*nE,1)), np.array([[1]]), 0*np.ones((nE,1))))
    return c

def construct_min_constraints(E):
    numx = len(E)*3 + 1 + len(E)
    Fmin = np.zeros((2, numx))
    bmin = np.zeros((2,1))
    v1_indices = source_outgoing_edge_indices(k=1)
    return Fmin, bmin

def positivity_constraints(var, nE):
    numx = len(E)*3 + 1 + len(E)
    Fmin = np.zeros((2, numx))
    bmin = np.zeros((2,1))
    v1_indices = source_outgoing_edge_indices(k=1)
    return Fmin, bmin
