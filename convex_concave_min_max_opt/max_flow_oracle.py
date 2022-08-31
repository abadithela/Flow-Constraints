# Max flow oracle using pyomo
import numpy as np
from ipdb import set_trace as st
from collections import OrderedDict as od
import _pickle as pickle
import os
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import pyomo.environ as pyo
from pao.pyomo import *
from pyomo.opt import SolverFactory
import pdb
# Pyomo max flow oracle:
def max_flow_oracle(edges_keys, nodes_keys, src, sink, int, x, LAMBDA):
    edges = list(edges_keys.values())
    nodes = list(nodes_keys.values())
    ne = len(list(edges_keys.keys()))
    model = pyo.ConcreteModel()
    # st()
    model.nodes = nodes
    model.edges = edges
    model.t = x[3*ne:3*ne+1][0,0] # pick one t
    d_e = x[2*ne:3*ne]
    model.d_e =dict()
    for k, edge in edges_keys.items():
        model.d_e[edge[0], edge[1]] = d_e[k,0]
    model.f3 = pyo.Var(model.edges, within=pyo.NonNegativeReals)
    model.dual = pyo.Suffix(direction=pyo.Suffix.IMPORT_EXPORT)

    def max_flow(model):
        return model.t + LAMBDA*sum(model.f3[i,j] for (i, j) in model.edges if j == sink)
    model.o = pyo.Objective(rule=max_flow, sense=pyo.maximize)

    # Capacity constraints
    def capacity(model, i, j, k, l):
        return model.f3[i, j, k, l] <= model.t
    model.cap3 = pyo.Constraint(model.edges, rule=capacity)
    # st()
    # Conservation constraints
    def conservation(model, k, l):
        if (k,l) == sink or (k,l) == src:
            return pyo.Constraint.Skip
        incoming  = sum(model.f3[i,j] for (i,j) in model.edges if j == (k,l))
        outgoing = sum(model.f3[i,j] for (i,j) in model.edges if i == (k,l))
        return incoming == outgoing
    model.con = pyo.Constraint(model.nodes, rule=conservation)

    # # nothing enters the source
    def no_in_source(model, i,j,k,l):
        if (k,l) == src:
            return model.f3[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.no_in_source = pyo.Constraint(model.edges, rule=no_in_source)
    # nothing leaves sink
    def no_out_sink(model, i,j,k,l):
        if (i,j) == sink:
            return model.f3[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.no_out_sink = pyo.Constraint(model.edges, rule=no_out_sink)

    # nothing enters the intermediate or leaves the intermediate
    def no_in_interm(model, i,j,k,l):
        if (k,l) == int:
            return model.f3[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.no_in_interm = pyo.Constraint(model.edges, rule=no_in_interm)

    def no_out_interm(model, i,j,k,l):
        if (i,j) == int:
            return model.f3[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.no_out_interm = pyo.Constraint(model.edges, rule=no_out_interm)

    def cut_cons(model, i, j, k, l):
        return model.f3[(i,j),(k,l)] + model.d_e[(i,j),(k,l)]<= model.t
    model.cut_cons = pyo.Constraint(model.edges, rule=cut_cons)

    print(" ==== Successfully added objective and constraints! ==== ")
    model.pprint()

    with Solver('glpk') as solver:
        results = solver.solve(model)

    model.pprint()
    print("Solution found!")
    # st()
    f3_e = dict()
    lamt = dict()
    d_e = dict()
    F = 0
    for (i,j),(k,l) in model.edges:
        F = 1.0/(model.t)
        f3_e.update({((i,j),(k,l)): model.f3[i,j,k,l].value*F})
    pdb.set_trace()
    return f3_e, lamt

# Find Lagrange dual of Linear Program:
# Ignore below; see the code above
# rEF: page 225 of Convex Optimization by Boyd and Vandenberghe
# Lagrange dual of linear program:
# max c^T x
# s.t: Ax <= b
# is given as follows:
# min b'lambda
# s.t: lambda >= 0
#      A'lambda -c >= 0
def lagrange_dual(edges_keys, nodes_keys, src, sink, int, x, LAMBDA):
    edges = list(edges_keys.values())
    nodes = list(nodes_keys.values())
    ne = len(list(edges_keys.keys()))
    model = pyo.ConcreteModel()
    # st()
    model.nodes = nodes
    model.edges = edges
    # Number of constraints: 6*|edges| + |nodes|
    model.constraint_list = ["capacity", "conservation", "no_in_source", "no_out_sink", "no_in_interm", "no_out_interm", "cut_cons"]
    model.constraints = []
    for cname in model.constraint_list:
        for edge in model.edges:
            model.constraints.append((edge[0], edge[1], cname))

    model.t = x[3*ne:3*ne+1][0,0] # pick one t
    d_e = x[2*ne:3*ne]
    model.d_e =dict()
    for k, edge in edges_keys.items():
        model.d_e[edge[0], edge[1]] = d_e[k,0]
    # Dual variables for various paramters:
    model.lamt_cap = pyo.Var(model.edges, within=pyo.NonNegativeReals)
    model.lamt_cons = pyo.Var(model.nodes, within=pyo.NonNegativeReals)
    model.lamt_no_in_src = pyo.Var(model.edges, within=pyo.NonNegativeReals)
    model.lamt_no_out_sink = pyo.Var(model.edges, within=pyo.NonNegativeReals)
    model.lamt_no_in_int = pyo.Var(model.edges, within=pyo.NonNegativeReals)
    model.lamt_no_out_int = pyo.Var(model.edges, within=pyo.NonNegativeReals)
    model.lamt_cut = pyo.Var(model.edges, within=pyo.NonNegativeReals)

    def max_flow(model):
        return model.t + LAMBDA*sum(model.f3[i,j] for (i, j) in model.edges if j == sink)
    model.o = pyo.Objective(rule=max_flow, sense=pyo.maximize)

    # ========= Dual problem constraints: A'lambda - c >= 0 =============== #
    def duality_constraint(model):
        constraint = (cap + cut + cons + no_in_src + no_out_sink + no_in_int + no_out_int >= model.c)
        return constraint
    model.duality_constraint = pyo.Constraint(rule=duality_constraint)
    # ======================================================================#
    def pos_lamt_cut(model, i, j, k, l):
        return model.lamt_cut[i, j, k, l] >= 0
    model.pos_lamt_cut = pyo.Constraint(model.edges, rule=pos_lamt_cut)

    def pos_lamt_cons(model, i, j):
        return model.lamt_cons[i, j] >= 0
    model.pos_lamt_cons = pyo.Constraint(model.nodes, rule=pos_lamt_cons)

    def pos_lamt_cap(model, i, j, k, l):
        return model.lamt_cap[i, j, k, l] >= 0
    model.pos_lamt_cap = pyo.Constraint(model.edges, rule=pos_lamt_cap)

    def pos_lamt_no_in_src(model, i, j, k, l):
        return model.lamt_no_in_src[i, j, k, l] >= 0
    model.pos_lamt_no_in_src = pyo.Constraint(model.edges, rule=pos_lamt_no_in_src)

    def pos_lamt_no_out_sink(model, i, j, k, l):
        return model.lamt_no_out_sink[i, j, k, l] >= 0
    model.pos_lamt_no_out_sink = pyo.Constraint(model.edges, rule=pos_lamt_no_out_sink)

    def pos_lamt_no_in_interm(model, i, j, k, l):
        return model.lamt_no_in_interm[i, j, k, l] >= 0
    model.pos_lamt_no_in_interm = pyo.Constraint(model.edges, rule=pos_lamt_no_in_interm)

    def pos_lamt_no_out_interm(model, i, j, k, l):
        return model.lamt_no_out_interm[i, j, k, l] >= 0
    model.pos_lamt_no_out_interm = pyo.Constraint(model.edges, rule=pos_lamt_no_out_interm)

    print(" ==== Successfully added objective and constraints! ==== ")
    model.pprint()

    with Solver('glpk') as solver:
        results = solver.solve(model)

    model.pprint()
    print("Solution found!")

    # st()
    f3_e = dict()
    d_e = dict()
    F = 0
    for (i,j),(k,l) in model.edges:
        F = 1.0/(model.t)
        f3_e.update({((i,j),(k,l)): model.f3[i,j,k,l].value*F})
    return lamt
