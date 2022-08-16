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
from pyomo.opt import SolverFactory
import pdb
# Pyomo max flow oracle:
def max_flow_oracle(edges_keys, nodes_keys, src, sink, int, x):
    edges = list(edges_keys.values())
    nodes = list(nodes_keys.values())
    ne = len(list(edges_keys.keys()))
    model = pyo.ConcreteModel()
    # st()
    model.nodes = nodes
    model.edges = edges
    model.t = x[3*ne:3*ne+1] # pick one t
    model.d_e = x[2*ne:3*ne]
    pdb.set_trace()
    model.f3 = pyo.Var(model.edges, within=pyo.NonNegativeReals)

    def max_flow(model):
        return sum(model.f3[i,j] for (i, j) in model.edges if j == sink)
    model.o = pyo.Objective(rule=max_flow, sense=pyo.minimize)

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

    with Solver('pao.pyomo.REG') as solver:
        results = solver.solve(model, tee=True)

    model.pprint()
    # st()
    f3_e = dict()
    d_e = dict()
    F = 0
    for (i,j),(k,l) in model.edges:
        F = 1.0/(model.t.value)
        f3_e.update({((i,j),(k,l)): model.f3[i,j,k,l].value*F})
    return f3_e
