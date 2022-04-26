import networkx as nx
import numpy as np
import gurobipy as gp
from gurobipy import GRB
import sys
import os
from lp import *
import scipy.sparse as sp
sys.path.append('..')
from static_obstacle_maze.network import MazeNetwork
import pdb

class MCF():
    '''
    Class that takes in a graph with sources, sinks, and commodities and returns the multi-commodity flow (modified so commodities do not compete for capacities) model m that can be passed into Gurobi.
    '''
    def __init__(self):
        self.graph = nx.DiGraph()
        self.m = gp.Model()
        self.source_dict = None
        self.commodities = None
        self.sink_dict = None
        self.variables = None
        self.source_nodes = None
        self.sink_nodes = None

    def add_graph(self, mazefile):
        '''
        Create game graph from file description.
        Graph's nodes are a list of tuples (i,j) where i and j are rows and columns of the graph
        '''
        maze = MazeNetwork(mazefile)
        self.graph = nx.DiGraph(maze.gamegraph)
        # pdb.set_trace() # Check how nodes of the graph work

    def add_source_dict(self, source_dict):
        '''
        Function to add source_dict to class of MCF!
        source_dict = {'s1': (1,2), 's2':(3,3), 's3': (1,2)}
        '''
        self.source_dict = source_dict
        if not self.commodities:
            self.commodities = ["c"+str(i) for i in range(len(source_dict.keys()))]
        if self.sink_dict:
            assert len(self.source_dict.keys()) == len(self.sink_dict.keys())
        self.source_nodes = [v for k, v in source_dict.items()]

    def add_sink_dict(self, sink_dict):
        '''
        Function to add sink_dict to class of MCF!
        sink_dict = {'t1': (1,2), 't2':(3,3), 't3': (1,2)}
        '''
        self.sink_dict = sink_dict
        if not self.commodities:
            self.commodities = ["c"+str(i) for i in range(len(sink_dict.keys()))]
        if self.source_dict:
            assert len(self.source_dict.keys()) == len(self.sink_dict.keys())
        self.sink_nodes = [v for k, v in sink_dict.items()]

    def add_positivity_constr(self):
        for var in self.variables:
            if isinstance(var, dict):
                for e, fe in var.items():
                    self.m.addConstr(var[e] >= 0)
            else:
                try:
                    self.m.addConstr(var >= 0)
                except:
                    print("Variable type mismatch")
                    pdb.set_trace()

    def add_capacity_constr(self, edge_vars):
        for var in edge_vars:
            assert isinstance(var, dict)
            for e, fe in var.items():
                self.m.addConstr(var[e] <= 1)

    def add_cut_constr(self):
        f1_e, f2_e, f3_e, F, d_e = self.variables
        assert set(f1_e.keys()) == set(d_e.keys())
        assert set(f2_e.keys()) == set(d_e.keys())
        assert set(f3_e.keys()) == set(d_e.keys())

        for e in d_e.keys():
            f1e = f1_e[e]
            f2e = f2_e[e]
            f3e = f3_e[e]
            de = d_e[e]
            self.m.addConstr(f1e + de <= 1)
            self.m.addConstr(f2e + de <= 1)
            self.m.addConstr(f3e + de <= 1)
            self.m.addConstr(de <= 1) # Max value of d_e

    # Add constraints:
    def add_cons_constr(self):
        '''
        for v in V:
            ev_in <-- incoming edges to v
            ev_out <-- outgoing edges from v

            ck_in.T fk_e_in = ck_out.T fk_e_out
        '''
        # conservation constraints
        outgoing = {}
        incoming = {}
        pdb.set_trace()
        for node in self.graph.nodes():
            if node not in self.source_nodes and node not in self.sink_nodes:
                outlist = []
                inlist = []
                for i,j in self.graph.edges():
                    if i == node and not j == node:
                        outlist.append((i,j))
                    elif j == node and not i == node:
                        inlist.append((i,j))
                outgoing.update({node: outlist})
                incoming.update({node: inlist})
        self.m.addConstrs((gp.quicksum(f[i, j] for i, j in outgoing[j]) == gp.quicksum(f[j, k] for j, k in incoming[j]) for j in self.graph.nodes()), "conservation")

    # Function to add variables:
    def add_vars(self):
        '''
        Creates and returns the variables for multi-commodity flow problem
        '''
        f1_e = {} # Flow for commodity 1
        f2_e = {} # Flow for commodity 2
        f3_e = {} # Flow for commodity
        d_e = {} # Cuts in graph
        for (i,j) in self.graph.edges():
            f1_e[i,j] = self.m.addVar(vtype=GRB.CONTINUOUS, name = "c1_flow")
            f2_e[i,j] = self.m.addVar(vtype=GRB.CONTINUOUS, name = "c2_flow")
            f3_e[i,j] = self.m.addVar(vtype=GRB.CONTINUOUS, name = "c3_flow")
            d_e[i,j] = self.m.addVar(vtype=GRB.CONTINUOUS, name = "cut_var")
        F = self.m.addVar(vtype=GRB.CONTINUOUS, name="F")
        self.variables = [f1_e, f2_e, f3_e, F, d_e]

    # Function to add minimizing constraints:
    def add_min_constr(self):
        f1_e, f2_e, f3_e, F, d_e = self.variables
        min_constr1 = (F <= gp.quicksum(f1_e[i,j] for i,j in self.graph.edges if i == self.source_dict['s1']))
        min_constr2 = (F <= gp.quicksum(f2_e[i,j] for i,j in self.graph.edges if i == self.source_dict['s2']))
        self.m.addConstr(min_constr1)
        self.m.addConstr(min_constr2)

    # Function to add constraints:
    def add_constraints(self):
        f1_e, f2_e, f3_e, F, d_e = self.variables
        self.add_min_constr() # F <= sum(f1_e) for e from s1 and F<= sum(f2_e) for e from s2
        try:
            self.add_positivity_constr()
            self.add_capacity_constr([f1_e, f2_e, f3_e])

            print(" == Added positivity, min, and capacity constraints! == ")
        except:
            print(" == Error in adding positivity, min or capacity constraints == ")
            pdb.set_trace()

        try:
            self.add_cut_constr() # Cut constraints: if cut, no flow
            print(" == Added cut constraints! == ")
        except:
            print(" == Error in adding cut constraints! == ")
            pdb.set_trace()

        try:
            self.add_cons_constr() # Conservation at each node
            print(" == Added conservation constraints! == ")
        except:
            print(" == Error in adding conservation constraints! == ")
            pdb.set_trace()

    # Add objective function to the model:
    def add_objective(self):
        f1_e, f2_e, f3_e, F, d_e = self.variables
        self.m.setObjective(F, GRB.MAXIMIZE)

    def save_mcf_lp(self, filename):
        '''
        Saves the Multi-Commodity Flow LP in an LP model file.
        '''
        lp.save_model(filename)

    def compute_sol(self, filename):
        '''
        Solves a linear program model saved in filename, and returns optimal output
        '''
        sol = lp.solve_lp(filename)
        return sol

# Example of how this class should be used:
if __name__ == '__main__':
    mazefile = os.getcwd()+'/maze_new.txt'
    mcf_problem = MCF()

    # Setup
    mcf_problem.add_graph(mazefile) # Game graph
    source_dict = {'s1': (1,1), 's2': (2,2), 's3': (3,3)}
    sink_dict = {'t1': (4,4), 't2': (3,3), 't3': (3,3)}
    mcf_problem.add_source_dict(source_dict) # Source dict
    mcf_problem.add_sink_dict(sink_dict) # Sink dict
    mcf_problem.add_vars() # Create variables

    # Add objective and constraints:
    mcf_problem.add_objective()
    mcf_problem.add_constraints()
    print(" ==== Successfully added objective and constraints! ==== ")
    # Dump model in file:

    # Solve for max flow:

    # Save LP solution:

    # Visualize the plot:
