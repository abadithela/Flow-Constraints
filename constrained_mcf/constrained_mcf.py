import networkx as nx
import numpy as np
import gurobipy as gp
from gurobipy import GRB
import sys
from lp import *
import scipy.sparse as sp

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

def add_positivity_constr(m, variables):
    for var in variables:
        lv = len(var)
        m.addConstr(var >= np.zeros((lv,1)))
    return m

def add_capacity_constr(m, edge_vars):
    for var in edge_vars:
        lv = len(var)
        m.addConstr(var <= np.ones((lv,1)))
    return m

def add_cut_constr(m, variables, nE):
    f1_e, f2_e, f3_e, F, d_e = variables
    m.addConstr(f1_e + d_e <= np.ones((nE,1)))
    m.addConstr(f2_e + d_e <= np.ones((nE,1)))
    m.addConstr(f3_e + d_e <= np.ones((nE,1)))
    m.addConstr(d_e <= np.ones((nE,1))) # Max value of d_e
    return m

def add_cons_constr(m, edge_vars, source_dict, sink_dict):
    for var in edge_vars:
        m.addConstr(f1_e + d_e <= np.ones((nE,1)))
        m.addConstr(f2_e + d_e <= np.ones((nE,1)))
        m.addConstr(f3_e + d_e <= np.ones((nE,1)))
        m.addConstr(d_e <= np.ones((nE,1))) # Max value of d_e
    return m

# Function to add variables:
def add_vars(model, nE):
    '''
    Creates and returns the variables for multi-commodity flow problem
    '''
    f1_e = model.addMVar(shape=nE, vtype=GRB.CONTINUOUS, name="f1_e")
    f2_e = model.addMVar(shape=nE, vtype=GRB.CONTINUOUS, name="f2_e")
    f3_e = model.addMVar(shape=nE, vtype=GRB.CONTINUOUS, name="f3_e")
    F = model.addVar(vtype=GRB.CONTINUOUS, name="F")
    d_e = model.addMVar(shape=nE, vtype=GRB.CONTINUOUS, name="d_e")
    variables = [f1_e, f2_e, f3_e, F, d_e]
    return model, variables

# Function to add constraints:
def add_constraints(model, variables, nE):
    f1_e, f2_e, f3_e, F, d_e = variables
    model = add_positivity_constr(model, variables)
    model = add_capacity_constr(model, [f1_e, f2_e, f3_e])
    model = add_cut_constr(model, variables, nE)
    return model

def save_mcf_lp(filename, G):
    '''
    Saves the Multi-Commodity Flow LP in an LP model file.
    '''
    V = G.nodes()
    E = G.edges()
    nE = len(E) # no. of edges
    nV = len(V) # no. of vertices

    m = gp.Model()
    m, variables = add_vars(m, nE)
    m = add_constraints(m, variables)
    lp.save_model(m, filename)

def compute_sol(filename):
    '''
    Solves a linear program model saved in filename, and returns optimal output
    '''
    sol = lp.solve_lp(filename)
    return sol
