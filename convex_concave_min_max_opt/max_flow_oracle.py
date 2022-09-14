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
    model.cap = pyo.Constraint(model.edges, rule=capacity)
    # st()
    # Conservation constraints
    def conservation(model, i,j, k, l):
        if (k,l) == sink or (i,j) == src:
            return pyo.Constraint.Skip
        incoming  = sum(model.f3[i,j] for (i,j) in model.edges if j == (k,l))
        outgoing = sum(model.f3[i,j] for (i,j) in model.edges if i == (k,l))
        return incoming == outgoing
    model.con = pyo.Constraint(model.edges, rule=conservation)

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
    for key in model.dual.keys():
        constraint_name, edge_info = key.name.split("[")
        edge_info = edge_info.split("]")
        edge_info = edge_info[0].split(",")
        u_edge = (float(edge_info[0]), float(edge_info[1]))
        if constraint_name != "con":
            v_edge = (float(edge_info[2]), float(edge_info[3]))
            lamt.update({((u_edge, v_edge), constraint_name): model.dual[key]})
        else:
            lamt.update({(u_edge, constraint_name): model.dual[key]})
    return f3_e, lamt, pyo.value(model.o)

# Pyomo max flow oracle:
def max_flow_oracle_fullg(edges_keys, nodes_keys, src, sink, int, x, LAMBDA):
    edges = list(edges_keys.values())
    nodes = list(nodes_keys.values())
    ne = len(list(edges_keys.keys()))
    model = pyo.ConcreteModel()
    # st()
    model.nodes = nodes
    model.edges = edges
    model.t = pyo.Var(within=pyo.NonNegativeReals)
    t = x[3*ne:3*ne+1][0,0] # pick one t
    d_e = x[2*ne:3*ne]
    f1 = x[0:ne]
    f2 = x[ne:2*ne]
    model.d_e = pyo.Var(model.edges, within=pyo.NonNegativeReals)
    model.f1 = pyo.Var(model.edges, within=pyo.NonNegativeReals)
    model.f2 = pyo.Var(model.edges, within=pyo.NonNegativeReals)
    for k, edge in edges_keys.items():
        model.d_e[edge[0], edge[1]] = d_e[k,0]
        model.f1[edge[0], edge[1]] = f1[k,0]
        model.f2[edge[0], edge[1]] = f2[k,0]

    model.f3 = pyo.Var(model.edges, within=pyo.NonNegativeReals)
    model.dual = pyo.Suffix(direction=pyo.Suffix.IMPORT_EXPORT)

    # Equality constraints:
    def set_f1(model, i, j, k, l):
        idx = list(edges_keys.keys())[list(edges_keys.values()).index(((i,j),(k,l)))]
        return model.f1[(i, j), (k, l)] == f1[idx,0]
    model.set_f1 = pyo.Constraint(model.edges, rule=set_f1)

    def set_f2(model, i, j, k, l):
        idx = list(edges_keys.keys())[list(edges_keys.values()).index(((i,j),(k,l)))]
        return model.f2[(i, j), (k, l)] == f2[idx,0]
    model.set_f2 = pyo.Constraint(model.edges, rule=set_f2)

    def set_de(model, i, j, k, l):
        idx = list(edges_keys.keys())[list(edges_keys.values()).index(((i,j),(k,l)))]
        return model.d_e[(i, j), (k, l)] == d_e[idx,0]
    model.set_de = pyo.Constraint(model.edges, rule=set_de)

    def set_t(model):
        return model.t == t
    model.set_t = pyo.Constraint(rule=set_t)

    def max_flow(model):
        return model.t + LAMBDA*sum(model.f3[i,j] for (i, j) in model.edges if j == sink)
    model.o = pyo.Objective(rule=max_flow, sense=pyo.maximize)

    # Capacity constraints
    def capacity1(model, i, j, k, l):
        return model.f1[(i, j), (k, l)] <= model.t
    model.cap1 = pyo.Constraint(model.edges, rule=capacity1)

    def capacity2(model, i, j, k, l):
        return model.f2[(i, j), (k, l)] <= model.t
    model.cap2 = pyo.Constraint(model.edges, rule=capacity2)

    def capacity3(model, i, j, k, l):
        return model.f3[(i, j), (k, l)] <= model.t
    model.cap3 = pyo.Constraint(model.edges, rule=capacity3)

    def capacity_de(model, i, j, k, l):
        return model.d_e[(i, j), (k, l)] <= model.t
    model.capd_e = pyo.Constraint(model.edges, rule=capacity_de)

    # Conservation constraints
    def conservation1(model, k, l):
        if (k,l) == int or (k,l) == src:
            return pyo.Constraint.Skip
        incoming  = sum(model.f1[i,j] for (i,j) in model.edges if j == (k,l))
        outgoing = sum(model.f1[i,j] for (i,j) in model.edges if i == (k,l))
        return incoming == outgoing
    model.con1 = pyo.Constraint(model.nodes, rule=conservation1)

    def conservation2(model, k, l):
        if (k,l) == int or (k,l) == sink:
            return pyo.Constraint.Skip
        incoming  = sum(model.f2[i,j] for (i,j) in model.edges if j == (k,l))
        outgoing = sum(model.f2[i,j] for (i,j) in model.edges if i == (k,l))
        return incoming == outgoing
    model.con2 = pyo.Constraint(model.nodes, rule=conservation2)

    def conservation3(model, k, l):
        if (k,l) == src or (k,l) == sink:
            return pyo.Constraint.Skip
        incoming  = sum(model.f3[i,j] for (i,j) in model.edges if j == (k,l))
        outgoing = sum(model.f3[i,j] for (i,j) in model.edges if i == (k,l))
        return incoming == outgoing
    model.con3 = pyo.Constraint(model.nodes, rule=conservation3)

    # # nothing enters the source
    def no_in_source1(model, i,j,k,l):
        if (k,l) == src:
            return model.f1[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.no_in_source1 = pyo.Constraint(model.edges, rule=no_in_source1)

    def no_in_source2(model, i,j,k,l):
        if (k,l) == int:
            return model.f2[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.no_in_source2 = pyo.Constraint(model.edges, rule=no_in_source2)

    def no_in_source3(model, i,j,k,l):
        if (k,l) == src:
            return model.f3[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.no_in_source3 = pyo.Constraint(model.edges, rule=no_in_source3)

    # nothing leaves sink
    def no_out_sink1(model, i,j,k,l):
        if (i,j) == int:
            return model.f1[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.no_out_sink1 = pyo.Constraint(model.edges, rule=no_out_sink1)

    def no_out_sink2(model, i,j,k,l):
        if (i,j) == sink:
            return model.f2[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.no_out_sink2 = pyo.Constraint(model.edges, rule=no_out_sink2)

    def no_out_sink3(model, i,j,k,l):
        if (i,j) == sink:
            return model.f3[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.no_out_sink3 = pyo.Constraint(model.edges, rule=no_out_sink3)

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

    def cut_cons1(model, i, j, k, l):
        return model.f1[(i,j),(k,l)] + model.d_e[(i,j),(k,l)]<= model.t
    model.cut_cons1 = pyo.Constraint(model.edges, rule=cut_cons1)

    def cut_cons2(model, i, j, k, l):
        return model.f2[(i,j),(k,l)] + model.d_e[(i,j),(k,l)]<= model.t
    model.cut_cons2 = pyo.Constraint(model.edges, rule=cut_cons2)

    def cut_cons3(model, i, j, k, l):
        return model.f3[(i,j),(k,l)] + model.d_e[(i,j),(k,l)]<= model.t
    model.cut_cons3 = pyo.Constraint(model.edges, rule=cut_cons3)

    # Add min flow constraint:
    def min_flow1(model):
        flow1 = sum(model.f1[i,j] for (i,j) in model.edges if i == src)
        return flow1 >= 1
    model.min_flow1 = pyo.Constraint(rule=min_flow1)

    def min_flow2(model):
        flow2 = sum(model.f2[i,j] for (i,j) in model.edges if i == int)
        return flow2 >= 1
    model.min_flow2 = pyo.Constraint(rule=min_flow2)

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
        F = 1.0/(model.t.value)
        f3_e.update({((i,j),(k,l)): model.f3[i,j,k,l].value*F})

    # for key in model.dual.keys():
    #     if key.name not in ["set_t", "min_flow1", "min_flow2"]:
    #         try:
    #             constraint_name, edge_info = key.name.split("[")
    #             edge_info = edge_info.split("]")
    #             edge_info = edge_info[0].split(",")
    #         except:
    #             pdb.set_trace()
    #         u_edge = (float(edge_info[0]), float(edge_info[1]))
    #         if constraint_name not in ["con1", "con2", "con3"]:
    #             v_edge = (float(edge_info[2]), float(edge_info[3]))
    #             lamt.update({((u_edge, v_edge), constraint_name): model.dual[key]})
    #         else:
    #             lamt.update({(u_edge, constraint_name): model.dual[key]})
    #     else:
    #         lamt.update({key.name: model.dual[key]})

    for key in model.dual.keys():
        lamt.update({key.name: model.dual[key]})
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
