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
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from pao.pyomo import *
import pyomo.environ as pyo
from pyomo.opt import SolverFactory

def solve_bilevel(nodes, edges, init, intermed, goal):
    G = nx.DiGraph()
    G.add_nodes_from(nodes)
    G.add_edges_from(edges)
    # remove self loops
    for i,j in edges:
        if i == j:
            G.remove_edge(i,j)

    model = pyo.ConcreteModel()
    model.name = 'virtual_gg'
    model.nodes = list(G.nodes())
    model.edges = list(G.edges())

    src = init # list
    sink = goal # list
    int = intermed # list
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
    def mcf_flow(model):
        lam = 1000
        flow_3 = sum(model.L.f3[i,j] for (i, j) in model.L.edges if i in src)
        return (model.t + lam * flow_3)
    model.o = pyo.Objective(rule=mcf_flow, sense=pyo.minimize)

    # Constraints
    # Maximize the flow into the sink
    def flow_src1(model):
        return 1 <= sum(model.y['f1_e', i,j] for (i, j) in model.edges if i in src)
    def flow_src2(model):
        return 1 <= sum(model.y['f2_e', i,j] for (i, j) in model.edges if i in int)
    model.min_constr1 = pyo.Constraint(rule=flow_src1)
    model.min_constr2 = pyo.Constraint(rule=flow_src2)

    def capacity1(model, i, j):
        return model.y['f1_e', i, j] <= model.t
    def capacity2(model, i, j):
        return model.y['f2_e', i, j] <= model.t
    model.cap1 = pyo.Constraint(model.edges, rule=capacity1)
    model.cap2 = pyo.Constraint(model.edges, rule=capacity2)

    # conservation constraints
    def conservation1(model, k):
        if k in src or k in int:
            return pyo.Constraint.Skip
        incoming  = sum(model.y['f1_e', i, j] for (i, j) in model.edges if j == k)
        outgoing = sum(model.y['f1_e',i,j] for (i, j) in model.edges if i == k)
        return incoming == outgoing
    model.cons1 = pyo.Constraint(model.nodes, rule=conservation1)

    def conservation2(model, k):
        if k in int or k in sink:
            return pyo.Constraint.Skip
        incoming  = sum(model.y['f2_e', i,j] for (i,j) in model.edges if j == k)
        outgoing = sum(model.y['f2_e', i,j] for (i,j) in model.edges if i == k)
        return incoming == outgoing
    model.cons2 = pyo.Constraint(model.nodes, rule=conservation2)

    # no flow into sources and out of sinks
    def no_in_source1(model, i,k):
        if k in src:
            return model.y['f1_e',i,k] == 0
        else:
            return pyo.Constraint.Skip
    model.no_in_source1 = pyo.Constraint(model.edges, rule=no_in_source1)
    # nothing leaves sink
    def no_out_sink1(model, i,k):
        if i in int:
            return model.y['f1_e', i,k] == 0
        else:
            return pyo.Constraint.Skip
    model.no_out_sink1 = pyo.Constraint(model.edges, rule=no_out_sink1)
    # =================================================================== #
    def no_in_source2(model, i,k):
        if k in int:
            return model.y['f2_e',i,k] == 0
        else:
            return pyo.Constraint.Skip
    model.no_in_source2 = pyo.Constraint(model.edges, rule=no_in_source2)

    # nothing leaves sink
    def no_out_sink2(model, i,k):
        if i in sink:
            return model.y['f2_e',i,k] == 0
        else:
            return pyo.Constraint.Skip
    model.no_out_sink2 = pyo.Constraint(model.edges, rule=no_out_sink2)
    # =================================================================== #

    # If the edge is cut -> no flow
    def cut_cons1(model, i, j):
        return model.y['f1_e',i,j] + model.y['d_e',i,j]<= model.t
    model.cut_cons1 = pyo.Constraint(model.edges, rule=cut_cons1)

    def cut_cons2(model, i, j):
        return model.y['f2_e',i,j] + model.y['d_e',i,j]<= model.t
    model.cut_cons2 = pyo.Constraint(model.edges, rule=cut_cons2)

    # set F = 1
    def auxiliary(model ,i, j):
        return model.y['F',i,j] == 1
    # Adding other auxiliary constraints to make t finite:
    model.aux_constr = pyo.Constraint(model.edges, rule=auxiliary)


    # SUBMODEL
    # Objective - Maximize the flow into the sink
    def flow_sink(model):
        return sum(model.f3[i,j] for (i, j) in model.edges if j in sink)
    model.L.o = pyo.Objective(rule=flow_sink, sense=pyo.maximize)

    # Capacity constraints
    def capacity(mdl, i, j):
        return mdl.f3[i, j] <= model.t
    model.L.cap3 = pyo.Constraint(model.L.edges, rule=capacity)

    # Conservation constraints
    def conservation(model, k):
        if k in sink or k in src:
            return pyo.Constraint.Skip
        incoming  = sum(model.f3[i,j] for (i,j) in model.edges if j == k)
        outgoing = sum(model.f3[i,j] for (i,j) in model.edges if i == k)
        return incoming == outgoing
    model.L.con = pyo.Constraint(model.L.nodes, rule=conservation)

    # nothing enters the source
    def no_in_source(model, i,j):
        if j in src:
            return model.f3[i,j] == 0
        else:
            return pyo.Constraint.Skip
    model.L.no_in_source = pyo.Constraint(model.L.edges, rule=no_in_source)

    # nothing leaves sink
    def no_out_sink(model, i,j):
        if i in sink:
            return model.f3[i,j] == 0
        else:
            return pyo.Constraint.Skip
    model.L.no_out_sink = pyo.Constraint(model.L.edges, rule=no_out_sink)

    # nothing enters the intermediate or leaves the intermediate
    def no_in_interm(model, i,j):
        if j in int:
            return model.f3[i,j] == 0
        else:
            return pyo.Constraint.Skip
    model.L.no_in_interm = pyo.Constraint(model.L.edges, rule=no_in_interm)

    def no_out_interm(model, i,j):
        if j in int:
            return model.f3[i,j] == 0
        else:
            return pyo.Constraint.Skip
    model.L.no_out_interm = pyo.Constraint(model.L.edges, rule=no_out_interm)

    # Cut constraints for flow 3
    def cut_cons(mdl, i, j):
        return mdl.f3[i,j] + model.y['d_e',i,j]<= model.t
    model.L.cut_cons = pyo.Constraint(model.L.edges, rule=cut_cons)

    print(" ==== Successfully added objective and constraints! ==== ")
    # if debug:
    # model.pprint()
    # st()

    solver = Solver('pao.pyomo.REG')
    results = solver.solve(model, tee=True)

    # with Solver('pao.pyomo.REG') as solver:
    #     solver.solve(model, tee=True)

    # model.pprint()
    # st()
    f1_e = dict()
    f2_e = dict()
    f3_e = dict()
    d_e = dict()
    F = 0
    for (i,j) in model.edges:
        F = (model.y['F', i,j].value)/(model.t.value)
        f1_e.update({((i,j)): model.y['f1_e', i,j].value*F})
        f2_e.update({((i,j)): model.y['f2_e', i,j].value*F})
        d_e.update({((i,j)): model.y['d_e', i,j].value*F})
    for (i,j)in model.L.edges:
        f3_e.update({((i,j)): model.L.f3[i,j].value*F})

    # st()
    # for key in d_e.items():
    #     if d_e[key][-1] >= 0.5:
    #         print('Edge {} cut'.format(key))
    print(d_e)
    print(F)
    # st()
    # plot_mcf(maze, f1_e, f2_e, f3_e, d_e)
    return f1_e, f2_e, f3_e, d_e, F
