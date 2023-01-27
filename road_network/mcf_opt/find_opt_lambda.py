# Script verifying that there exists a lambda that leads to 0/1 optimal cuts
# J. Graebener, A. Badithela
# June 2022
# Constrained Test Environments for GR(1) Specifications
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
from components.road_network import RoadNetwork, create_network_from_file
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from helpers.plotting import plot_flow, plot_maze, plot_mcf
from pao.pyomo import *
import pyomo.environ as pyo
from pyomo.opt import SolverFactory
import pdb
debug = False

def solve_bilevel(maze, lam2=1e6):
    G = nx.DiGraph(maze.gamegraph)
    # remove self loops
    edges = list(G.edges())
    for i,j in edges:
        if i == j:
            G.remove_edge(i,j)

    model = pyo.ConcreteModel()
    model.nodes = list(G.nodes())
    model.edges = list(G.edges())

    src = maze.source
    sink = maze.goal
    int = maze.intermediate
    # st()
    vars = ['f1_e', 'f2_e', 'd_e', 'F']
    model.y = pyo.Var(vars, model.edges, within=pyo.NonNegativeReals)
    model.t = pyo.Var(within=pyo.NonNegativeReals)

    # Introduce SUBMODEL
    # fixed variables defined by the upper level (tester)
    fixed_variables = [model.y['d_e',i,j] for i,j in model.edges] # Cut edges
    fixed_variables.extend([model.y['f1_e',i,j] for i,j in model.edges]) # Flow 1
    fixed_variables.extend([model.y['f2_e',i,j] for i,j in model.edges]) # Flow 2
    fixed_variables.extend([model.y['F',i,j] for i,j in model.edges]) # total flow through i
    fixed_variables.extend([model.t]) # 1/F
    # Submodel - variables defined by the system under test
    model.L = SubModel(fixed=fixed_variables)
    model.L.edges = model.edges
    model.L.nodes = model.nodes
    model.L.f3 = pyo.Var(model.L.edges, within=pyo.NonNegativeReals) # Flow 3 (from s to t not through i)

    # Upper level Objective
    # Objective - minimize 1/F + lambda*f_3/F
    def flow_cut_gap(model):
        lam = 10
        flow_3 = sum(model.L.f3[i,j] for (i, j) in model.L.edges if i == src)
        return sum(model.y['d_e',i,j] for (i,j) in model.edges) + lam * flow_3

    # Objective - minimize 1/F + lambda*f_3/F
    def mcf_flow(model):
        lam = 1
        flow_3 = sum(model.L.f3[i,j] for (i, j) in model.L.edges if i == src)
        return (model.t + lam * flow_3)

    # Objective - minimize 1/F + lambda*f_3/F
    def mcf_flow_new(model):
        lam1 = 1
        flow_3 = sum(model.L.f3[i,j] for (i, j) in model.L.edges if i == src)
        return lam1*model.t + lam2*flow_3

    def flow_3(model):
        flow_3 = sum(model.L.f3[i,j] for (i, j) in model.L.edges if i == src)
        return flow_3

    # Objective - minimize 1/F + lambda*f_3/F
    def mcf_flow_quad(model):
        lam = 1
        flow_3 = sum(model.L.f3[i,j]**2 for (i, j) in model.L.edges if i == src)
        return (model.t**2 + lam * flow_3)

    model.o = pyo.Objective(rule=mcf_flow_new, sense=pyo.minimize)

    # Constraints
    # Maximize the flow into the sink
    def flow_src1(model):
        return 1 <= sum(model.y['f1_e', i,j] for (i, j) in model.edges if i == src)
    def flow_src2(model):
        return 1 <= sum(model.y['f2_e', i,j] for (i, j) in model.edges if i == int)
    model.min_constr1 = pyo.Constraint(rule=flow_src1)
    model.min_constr2 = pyo.Constraint(rule=flow_src2)

    def capacity1(model, i, j, k, l):
        return model.y['f1_e',(i,j),(k,l)] <= model.t
    def capacity2(model, i, j, k, l):
        return model.y['f2_e', (i,j),(k,l)] <= model.t
    model.cap1 = pyo.Constraint(model.edges, rule=capacity1)
    model.cap2 = pyo.Constraint(model.edges, rule=capacity2)

    # conservation constraints
    def conservation1(model, k, l):
        if (k,l) == src or (k,l) == int:
            return pyo.Constraint.Skip
        incoming  = sum(model.y['f1_e', i,j] for (i,j) in model.edges if j == (k,l))
        outgoing = sum(model.y['f1_e',i,j] for (i,j) in model.edges if i == (k,l))
        return incoming == outgoing
    model.cons1 = pyo.Constraint(model.nodes, rule=conservation1)

    def conservation2(model, k, l):
        if (k,l) == int or (k,l) == sink:
            return pyo.Constraint.Skip
        incoming  = sum(model.y['f2_e', i,j] for (i,j) in model.edges if j == (k,l))
        outgoing = sum(model.y['f2_e', i,j] for (i,j) in model.edges if i == (k,l))
        return incoming == outgoing
    model.cons2 = pyo.Constraint(model.nodes, rule=conservation2)

    # no flow into sources and out of sinks
    def no_in_source1(model, i,j,k,l):
        if (k,l) == src:
            return model.y['f1_e',(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.no_in_source1 = pyo.Constraint(model.edges, rule=no_in_source1)
    # nothing leaves sink
    def no_out_sink1(model, i,j,k,l):
        if (i,j) == int:
            return model.y['f1_e', (i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.no_out_sink1 = pyo.Constraint(model.edges, rule=no_out_sink1)
    # =================================================================== #
    def no_in_source2(model, i,j,k,l):
        if (k,l) == int:
            return model.y['f2_e',(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.no_in_source2 = pyo.Constraint(model.edges, rule=no_in_source2)

    # nothing leaves sink
    def no_out_sink2(model, i,j,k,l):
        if (i,j) == sink:
            return model.y['f2_e',(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.no_out_sink2 = pyo.Constraint(model.edges, rule=no_out_sink2)
    # =================================================================== #

    # If the edge is cut -> no flow
    def cut_cons1(model, i, j, k, l):
        return model.y['f1_e',(i,j),(k,l)] + model.y['d_e',(i,j),(k,l)]<= model.t
    model.cut_cons1 = pyo.Constraint(model.edges, rule=cut_cons1)

    def cut_cons2(model, i, j, k, l):
        return model.y['f2_e',(i,j),(k,l)] + model.y['d_e',(i,j),(k,l)]<= model.t
    model.cut_cons2 = pyo.Constraint(model.edges, rule=cut_cons2)

    # set F = 1
    def auxiliary(model ,i, j, k, l):
        return model.y['F',i,j,k,l] == 1
    # Adding other auxiliary constraints to make t finite:
    model.aux_constr = pyo.Constraint(model.edges, rule=auxiliary)

    # Auxiliary edge cut for debugging purposes
    def aux_edge(model, i,j, k,l):
        if (i,j) == (2,2) and (k,l) == (2,3):
            return model.y['d_e', i,j,k,l] == 1
        else:
            return pyo.Constraint.Skip
    # Add an obstacle to check if the
    if debug:
        model.edge_constr = pyo.Constraint(model.edges, rule=aux_edge)

    # SUBMODEL
    # Objective - Maximize the flow into the sink
    def flow_sink(model):
        return sum(model.f3[i,j] for (i, j) in model.edges if j == sink)

    def flow_sink_new(model):
        return sum(model.f3[i,j] for (i, j) in model.edges if j == sink)

    def flow_sink_quad(model):
        return sum(model.f3[i,j]**2 for (i, j) in model.edges if j == sink)

    model.L.o = pyo.Objective(rule=flow_sink, sense=pyo.maximize)

    # Capacity constraints
    def capacity(mdl, i, j, k, l):
        return mdl.f3[i, j, k, l] <= model.t
    model.L.cap3 = pyo.Constraint(model.L.edges, rule=capacity)

    # Conservation constraints
    def conservation(model, k, l):
        if (k,l) == sink or (k,l) == src:
            return pyo.Constraint.Skip
        incoming  = sum(model.f3[i,j] for (i,j) in model.edges if j == (k,l))
        outgoing = sum(model.f3[i,j] for (i,j) in model.edges if i == (k,l))
        return incoming == outgoing
    model.L.con = pyo.Constraint(model.L.nodes, rule=conservation)

    # nothing enters the source
    def no_in_source(model, i,j,k,l):
        if (k,l) == src:
            return model.f3[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.L.no_in_source = pyo.Constraint(model.L.edges, rule=no_in_source)

    # nothing leaves sink
    def no_out_sink(model, i,j,k,l):
        if (i,j) == sink:
            return model.f3[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.L.no_out_sink = pyo.Constraint(model.L.edges, rule=no_out_sink)

    # nothing enters the intermediate or leaves the intermediate
    def no_in_interm(model, i,j,k,l):
        if (k,l) == int:
            return model.f3[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.L.no_in_interm = pyo.Constraint(model.L.edges, rule=no_in_interm)

    def no_out_interm(model, i,j,k,l):
        if (i,j) == int:
            return model.f3[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.L.no_out_interm = pyo.Constraint(model.L.edges, rule=no_out_interm)

    # Cut constraints for flow 3
    def cut_cons(mdl, i, j, k, l):
        return mdl.f3[(i,j),(k,l)] + model.y['d_e',(i,j),(k,l)]<= model.t
    model.L.cut_cons = pyo.Constraint(model.L.edges, rule=cut_cons)

    print(" ==== Successfully added objective and constraints! ==== ")
    if debug:
        model.pprint()

    with Solver('pao.pyomo.REG') as solver:
        results = solver.solve(model, tee=True)

    # model.pprint()
    # st()
    f1_e = dict()
    f2_e = dict()
    f3_e = dict()
    d_e = dict()
    F = 0
    for (i,j),(k,l) in model.edges:
        F = (model.y['F',i,j,k,l].value)/(model.t.value)
        f1_e.update({((i,j),(k,l)): model.y['f1_e', i,j,k,l].value*F})
        f2_e.update({((i,j),(k,l)): model.y['f2_e', i,j,k,l].value*F})
        d_e.update({((i,j),(k,l)): model.y['d_e', i,j,k,l].value*F})

    for (i,j),(k,l) in model.L.edges:
        f3_e.update({((i,j),(k,l)): model.L.f3[i,j,k,l].value*F})

    return f1_e, f2_e, f3_e, d_e, F
    # st()
    # for key in d_e.items():
    #     if d_e[key][-1] >= 0.5:
    #         print('Edge {} cut'.format(key))

def find_opt_reg_param(maze, reg_param_list):
    # setup_problem(maze, source, sink, intermediate)
    Fmax = 0
    lam_max = 0
    for lam2 in reg_param_list:
        try:
            f1_e, f2_e, f3_e, d_e, F = solve_bilevel(maze, lam2)
            if F > Fmax:
                Fmax = F
                lam_max = lam2
                max_flows = [f1_e, f2_e, f3_e, d_e]
        except ValueError:
            continue
    pdb.set_trace()
    eps = 1e-4
    
    return F, lam_max


if __name__ == '__main__':
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

    reg_param_list = np.logspace(-5.0, 5.0, num=10)
    maze = RoadNetwork(networkfile)
    # source = (5,0)
    # sink = (0,9)
    # intermediate = (2,2)

    maze.source = src
    maze.goal = sink
    maze.intermediate = int

    F, lam_max = find_opt_reg_param(maze, reg_param_list)
    print(Fmax)
    print(lam_max)
    st()
    plot_mcf(maze, max_flows[0], max_flows[1], max_flows[2], max_flows[3])