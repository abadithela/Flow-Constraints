import networkx as nx
import numpy as np
import gurobipy as gp
from gurobipy import GRB
import sys
from lp import *
import scipy.sparse as sp

class MCF():
    '''
    Class that takes in a graph with sources, sinks, and commodities and returns the multi-commodity flow (modified so commodities do not compete for capacities) model m that can be passed into Gurobi
    '''
    def __init__(self):
        self.graph = nx.DiGraph()
        self.m = gp.Model()
        self.source_dict = None
        self.commodities = None
        self.sink_dict = None
        self.variables = None

    def add_positivity_constr(self):
        for var in self.variables:
            lv = len(var)
            self.m.addConstr(var >= np.zeros((lv,1)))

    def add_capacity_constr(self, edge_vars):
        for var in edge_vars:
            lv = len(var)
            self.m.addConstr(var <= np.ones((lv,1)))

    def add_cut_constr(self, nE):
        f1_e, f2_e, f3_e, F, d_e = self.variables
        self.m.addConstr(f1_e + d_e <= np.ones((nE,1)))
        self.m.addConstr(f2_e + d_e <= np.ones((nE,1)))
        self.m.addConstr(f3_e + d_e <= np.ones((nE,1)))
        self.m.addConstr(d_e <= np.ones((nE,1))) # Max value of d_e

    # Add constraints:
    def add_cons_constr(self, edge_vars):
        '''
        for v in V:
            ev_in <-- incoming edges to v
            ev_out <-- outgoing edges from v

            ck_in.T fk_e_in = ck_out.T fk_e_out
        '''
        for var in edge_vars:
            self.m.addConstr(f1_e + d_e <= np.ones((nE,1)))
            self.m.addConstr(f2_e + d_e <= np.ones((nE,1)))
            self.m.addConstr(f3_e + d_e <= np.ones((nE,1)))
            self.m.addConstr(d_e <= np.ones((nE,1))) # Max value of d_e

    # Function to add variables:
    def add_vars(self, nE):
        '''
        Creates and returns the variables for multi-commodity flow problem
        '''
        f1_e = self.m.addMVar(shape=nE, vtype=GRB.CONTINUOUS, name="f1_e")
        f2_e = self.m.addMVar(shape=nE, vtype=GRB.CONTINUOUS, name="f2_e")
        f3_e = self.m.addMVar(shape=nE, vtype=GRB.CONTINUOUS, name="f3_e")
        F = self.m.addVar(vtype=GRB.CONTINUOUS, name="F")
        d_e = self.m.addMVar(shape=nE, vtype=GRB.CONTINUOUS, name="d_e")
        self.variables = [f1_e, f2_e, f3_e, F, d_e]

    # Function to add constraints:
    def add_constraints(self, nE):
        f1_e, f2_e, f3_e, F, d_e = self.variables
        self.add_positivity_constr()
        self.add_capacity_constr([f1_e, f2_e, f3_e])
        self.add_cut_constr(nE)

    def save_mcf_lp(self, filename, G):
        '''
        Saves the Multi-Commodity Flow LP in an LP model file.
        '''
        V = G.nodes()
        E = G.edges()
        nE = len(E) # no. of edges
        nV = len(V) # no. of vertices

        add_vars(nE)
        add_constraints()
        lp.save_model(filename)

    def compute_sol(self, filename):
        '''
        Solves a linear program model saved in filename, and returns optimal output
        '''
        sol = lp.solve_lp(filename)
        return sol
