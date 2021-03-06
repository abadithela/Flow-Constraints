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
from network import MazeNetwork, create_network_from_file
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import pyomo.environ as pyo
from pyomo.opt import SolverFactory

def find_max_flow_pyomo(maze):
    G = nx.DiGraph(maze.gamegraph)
    # remove self loops
    edges = list(G.edges())
    for i,j in edges:
        if i == j:
            G.remove_edge(i,j)

    model = pyo.ConcreteModel()
    # st()
    model.nodes = list(G.nodes())
    model.edges = list(G.edges())

    source = (5,0)
    sink = (0,9)

    model.f = pyo.Var(model.edges, within=pyo.NonNegativeReals)
    # Maximize the flow into the sink
    def flow_sink(model):
        return sum(model.f[i,j] for (i, j) in model.edges if j == sink)

    model.o = pyo.Objective(rule=flow_sink, sense=pyo.maximize)

    # Capacity constraints
    def capacity(model, i, j, k, l):
        return model.f[(i,j),(k,l)] <= 1
    model.cap = pyo.Constraint(model.edges, rule=capacity)

    # Conservation constraints
    def conservation(model, k, l):
        if (k,l) == sink or (k,l) == source:
            return pyo.Constraint.Skip
        incoming  = sum(model.f[i,j] for (i,j) in model.edges if j == (k,l))
        outgoing = sum(model.f[i,j] for (i,j) in model.edges if i == (k,l))
        return incoming == outgoing
    model.con = pyo.Constraint(model.nodes, rule=conservation)

    # nothing enters the source
    def no_in_source(model, i,j,k,l):
        if (k,l) == source:
            return model.f[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.no_in_source = pyo.Constraint(model.edges, rule=no_in_source)
    # nothing leaves sink
    def no_out_sink(model, i,j,k,l):
        if (i,j) == sink:
            return model.f[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.no_out_sink = pyo.Constraint(model.edges, rule=no_out_sink)

    # Create a solver
    opt = SolverFactory('glpk')
    # solve
    results = opt.solve(model)

    flow = dict()
    for (i,j),(k,l) in model.edges:
        flow.update({((i,j),(k,l)): model.f[i,j,k,l].value})

    plot_flow(maze, flow)

def get_max_flow(maze):
     # create model
    G = nx.DiGraph(maze.gamegraph)
    m = gp.Model()
    source = (5,0)
    sink = (0,9)

    # remove self loops
    edges = list(G.edges())
    for i,j in edges:
        if i == j:
            G.remove_edge(i,j)

    # create variables
    f = {}

    for (i,j) in G.edges():
        f[i,j] = m.addVar(vtype=GRB.CONTINUOUS, name = "flow")

    m.setObjective((gp.quicksum(f[i,j] for i,j in G.edges if i == source or j == sink)), GRB.MAXIMIZE)

    # capacity constraints
    m.addConstrs((f[i,j] <= 1 for i, j in G.edges), "capacity")

    # conservation constraints
    outgoing = {}
    incoming = {}
    for node in G.nodes():
        outlist = []
        inlist = []
        if not node == source and not node == sink:
            for i,j in G.edges():
                if i == node and not j == node:
                    outlist.append((i,j))
                elif j == node and not i == node:
                    inlist.append((i,j))
        outgoing.update({node: outlist})
        incoming.update({node: inlist})
    m.addConstrs((gp.quicksum(f[i, j] for i, j in outgoing[j]) == gp.quicksum(f[j, k] for j, k in incoming[j]) for j in G.nodes), "conservation")

    # nothing enters the source
    m.addConstr((gp.quicksum(f[i,j] for i,j in G.edges if j == source) == 0), "source")
    # nothing exits the sink
    m.addConstr((gp.quicksum(f[i,j] for i,j in G.edges if i == sink) == 0),"sink")

    m.update()
    # st()
    # solve IP model
    m.optimize()
    # st()
    flow = dict()
    for (i,j) in G.edges():
        flow.update({(i,j): f[i,j].x})

    plot_flow(maze,flow)
    # plot_maze(maze)
    # for item in x:
    #     if x[item].x > 0.5:
    #         print('node {0},{1} belongs to {2}'.format(item[0],item[1],item[2]))
    #         self.partition.update({(item[0],item[1]) : item[2]})
def plot_maze(maze):
    tilesize = 1
    xs = np.linspace(0, 10*tilesize, 10+1)
    ys = np.linspace(0, 6*tilesize, 6+1)
    ax = plt.gca()
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False)
    # grid "shades" (boxes)
    w, h = xs[1] - xs[0], ys[1] - ys[0]
    for i, x in enumerate(xs[:-1]):
        for j, y in enumerate(ys[:-1]):
            if maze.map[j,i]=='*':
                ax.add_patch(Rectangle((x, y), w, h, fill=True, color='black', alpha=.5))
            elif j == 2 and i == 2 :
                ax.add_patch(Rectangle((x, y), w, h, fill=True, color='blue', alpha=.2))
            elif maze.map[j,i]=='':
                ax.add_patch(Rectangle((x, y), w, h, fill=True, color='black', alpha=.5))
            elif i % 2 == j % 2: # racing flag style
                ax.add_patch(Rectangle((x, y), w, h, fill=True, color='gray', alpha=.1))
    # grid lines
    for x in xs:
        plt.plot([x, x], [ys[0], ys[-1]], color='black', alpha=.33, linestyle=':')
    for y in ys:
        plt.plot([xs[0], xs[-1]], [y, y], color='black', alpha=.33, linestyle=':')

    ax.invert_yaxis()
    plt.show()

def plot_flow(maze, flow):
    tilesize = 1
    xs = np.linspace(0, 10*tilesize, 10+1)
    ys = np.linspace(0, 6*tilesize, 6+1)
    ax = plt.gca()
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False)
    # grid "shades" (boxes)
    w, h = xs[1] - xs[0], ys[1] - ys[0]
    for i, x in enumerate(xs[:-1]):
        for j, y in enumerate(ys[:-1]):
            if maze.map[j,i]=='*': # racing flag style
                ax.add_patch(Rectangle((x, y), w, h, fill=True, color='black', alpha=.5))
            if maze.map[j,i]=='o': # racing flag style
                ax.add_patch(Rectangle((x, y), w, h, fill=True, color='red', alpha=.5))
            elif maze.map[j,i]=='': # racing flag style
                ax.add_patch(Rectangle((x, y), w, h, fill=True, color='black', alpha=.5))
            elif i % 2 == j % 2: # racing flag style
                ax.add_patch(Rectangle((x, y), w, h, fill=True, color='gray', alpha=.1))
    # grid lines
    for x in xs:
        plt.plot([x, x], [ys[0], ys[-1]], color='black', alpha=.33, linestyle=':')
    for y in ys:
        plt.plot([xs[0], xs[-1]], [y, y], color='black', alpha=.33, linestyle=':')

    # for item in flow:
    #     if flow[item].x > 0.5:
    #         startxy = item[0]
    #         endxy = item[1]
    #         plt.plot([startxy[1]+ tilesize/2, endxy[1]+ tilesize/2], [startxy[0]+ tilesize/2, endxy[0]+ tilesize/2], color='red', alpha=.33, linestyle='-')
    for item in flow:
        if flow[item] > 0.5:
            startxy = item[0]
            endxy = item[1]
            plt.plot([startxy[1]+ tilesize/2, endxy[1]+ tilesize/2], [startxy[0]+ tilesize/2, endxy[0]+ tilesize/2], color='red', alpha=.33, linestyle='-')

    ax.invert_yaxis()
    plt.show()


if __name__ == '__main__':
    mazefile = os.getcwd()+'/maze_new.txt'
    maze = MazeNetwork(mazefile)
    # get_max_flow(maze)
    find_max_flow_pyomo(maze)
