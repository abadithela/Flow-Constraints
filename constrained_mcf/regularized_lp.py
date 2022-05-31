# Regularized LP instead of the bilevel optimization:
# Apurva Badithela
# Linear Fractional program
import networkx as nx
import numpy as np
import gurobipy as gp
from gurobipy import GRB
import matplotlib.pyplot as plt
import sys
import os
import lp
from matplotlib.patches import Rectangle
import scipy.sparse as sp
sys.path.append('..')
from static_obstacle_maze.network import MazeNetwork
import pdb
import pyomo.environ as pyo
from pao.pyomo import *
from pyomo.opt import SolverFactory
solve_gurobi = False
solve_bilevel = True

class MCF():
    '''
    Class that takes in a graph with sources, sinks, and commodities and returns the multi-commodity flow (modified so commodities do not compete for capacities) self.m m that can be passed into Gurobi.
    '''
    def __init__(self):
        self.graph = nx.DiGraph()
        self.m = pyo.ConcreteModel()
        self.source_dict = None
        self.commodities = None
        self.sink_dict = None
        self.variables = None
        self.source_nodes = None
        self.sink_nodes = None
        self.sol = dict() # Dictionary storing solutions of variables
        self.edge_key_dict = dict() # Dictionary storing solutions of variables

    def add_graph(self, mazefile):
        '''
        Create game graph from file description.
        Graph's nodes are a list of tuples (i,j) where i and j are rows and columns of the graph
        Completed in Pyomo format.
        '''
        maze = MazeNetwork(mazefile)
        self.maze = maze
        self.graph = nx.DiGraph(maze.gamegraph)

        for i in self.graph.nodes:
            if (i,i) in self.graph.edges:
                self.graph.remove_edge(i,i)
        # pdb.set_trace() # Check how nodes of the graph work
        self.m.nodes = list(self.graph.nodes())
        self.m.edges = list(self.graph.edges())
        self.m.vars = ['f1_e', 'f2_e', 'f3_e', 'd_e', 'F']
        
