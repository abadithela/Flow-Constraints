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
from automata_construction import create_ts_automata_and_virtual_game_graph, make_graphs_for_optimization
import pdb

# Makes automata specific to corridor examples
def automata():
    ts, prod_ba, virtual, sys_virtual, state_map = create_ts_automata_and_virtual_game_graph()
    # G, B_prod, S are GraphData objects with the corresponding attributes
    G, B_prod, S = make_graphs_for_optimization(prod_ba, virtual, sys_virtual, state_map)
    for i,j in S.edges:
        if i == j:
            S.graph.remove_edge(i,j)
            S.edges = S.graph.edges

    for i, j in G.edges:
        if i == j:
            G.graph.remove_edge(i,j)
            G.edges = G.graph.edges
    return G, B_prod, S

def add_sys_ba_vars(model, S, G, sys_ba_de):
    vars = ['fba_e', 'dba_e']
    model_edges = [] # Store de values for every edge in S for every Bprod
    model_nodes = []
    ba_keys = []
    for k in sys_ba_de.keys():
        k_edges = []
        k_nodes = []
        k1 = k[0]
        k2 = k[1]

        for edge in S.edges:
            if ((k1, k2), edge[0], edge[1]) not in k_edges:
                k_edges.extend([((k1, k2), edge[0], edge[1])])

        for node in S.nodes:
            if (k,node) not in k_nodes:
                k_nodes.extend([(k, node)])
            if k not in ba_keys:
                ba_keys.extend([k])
        model_edges.extend(k_edges)
        model_nodes.extend(k_nodes)

    model.ba_edges = model_edges
    model.ba_nodes = model_nodes
    model.ba_keys = ba_keys
    model.ba_var = pyo.Var(vars, model.ba_edges, within=pyo.NonNegativeReals)
    src = S.init
    sink = S.acc_sys

    # Capacity constraint on flow
    def cap_constraint(model, k1, k2, i, j):
        return model.ba_var['fba_e', k1, k2, i, j] <= 1.0
    model.cap = pyo.Constraint(model.ba_edges, rule=cap_constraint)

    def cut_bound_constraint(model, k1, k2, i, j):
        return model.ba_var['dba_e', k1, k2, i, j] <= 1.0
    model.de_cap = pyo.Constraint(model.ba_edges, rule=cut_bound_constraint)

    def ba_cut_constraint(model, k1, k2, i, j):
        return model.ba_var['dba_e', k1, k2, i, j] + model.ba_var['fba_e', k1, k2, i, j] <= 1.0
    model.de_cut = pyo.Constraint(model.ba_edges, rule=ba_cut_constraint)

    # Conservation constraints:
    def ba_conservation(model, l1, l2, k):

        if k in src or k in sink:
            return pyo.Constraint.Skip
        incoming  = sum(model.ba_var['fba_e', k1,k2, i,j] for ((k1,k2),i,j) in model.ba_edges if (j == k and (l1,l2)==(k1,k2)))
        outgoing = sum(model.ba_var['fba_e', k1, k2, i,j] for ((k1, k2),i,j) in model.ba_edges if (i == k and (l1,l2)==(k1,k2)))
        return incoming == outgoing
    model.ba_cons = pyo.Constraint(model.ba_nodes, rule=ba_conservation)

    # no flow into sources and out of sinks
    def ba_no_in_source(model, key1, key2, i,k):
        if k in src:
            return model.ba_var['fba_e',key1, key2,i,k] == 0
        else:
            return pyo.Constraint.Skip
    model.ba_no_in_source = pyo.Constraint(model.ba_edges, rule=ba_no_in_source)
    # nothing leaves sink
    def ba_no_out_sink(model, key1,key2,i,k):
        if i in sink:
            return model.ba_var['fba_e', key1,key2,i,k] == 0
        else:
            return pyo.Constraint.Skip
    model.ba_no_out_sink = pyo.Constraint(model.ba_edges, rule=ba_no_out_sink)
    return model

def group_cuts(G, B_prod, S):
    map_q_to_de = dict() # M: q -> de. Edges that are active at any given state
    sys_ba_de = dict()

    for qnode in B_prod.nodes:
        q = B_prod.node_dict[qnode]
        map_q_to_de[q] = [] # The edges who cuts are active when the system is in a state where the atomic proposition tuple q is satisfied.
        sys_ba_de[q] = dict()

    for node in G.nodes:
        s, q = G.node_dict[node]
        outgoing_edges = G.graph.out_edges(node)
        map_q_to_de[q].extend(outgoing_edges)
        qtest = q[0]
        qsys = q[1]

    # All transitions are cut:
    for q, outgoing_edges in map_q_to_de.items():
        for (i,j) in outgoing_edges:
            su, qu = G.node_dict[i]
            sv, qv = G.node_dict[j]

            qtest_u, qsys_u = qu
            qtest_v, qsys_v = qv
            if ((su, qsys_u), (sv, qsys_v)) in list(sys_ba_de[q].keys()):
                sys_ba_de[q][((su, qsys_u), (sv, qsys_v))].extend([((su, qu), (sv, qv))]) # Edge mapping from G to sys_ba
            else:
                sys_ba_de[q][((su, qsys_u), (sv, qsys_v))] = [((su, qu), (sv, qv))]

    # Sanity check:
    for q in sys_ba_de.keys():
        for k in sys_ba_de[q].keys():
            try:
                assert (len(sys_ba_de[q][k]) == 1)
            except:
                pdb.set_trace()
    pdb.set_trace()
    print("1-1 mapping of edges")
    return map_q_to_de, sys_ba_de

def add_sys_ba_constraints(model, S, G, sys_ba_de):
    src = S.init
    def edge_match(model, k1,k2, i, j):
        si, qi = S.node_dict[i] # numeric to string
        sj, qj = S.node_dict[j] # Going to names
        relevant_edges = sys_ba_de[(k1,k2)]
        for S_edge, G_edge in relevant_edges.items():
            if si == S_edge[0][0] and sj == S_edge[1][0]:
                vGi = G_edge[0][0]
                vGj = G_edge[0][1]
                imap = G.inv_node_dict[vGi] # Converting string back to integers nodes
                jmap = G.inv_node_dict[vGj]
                assert((imap, jmap) in model.edges)
                return model.ba_var['dba_e', k1,k2,i,j] == model.y['d_e', imap, jmap]
        return pyo.Constraint.Skip
    model.edge_matching = pyo.Constraint(model.ba_edges, rule=edge_match)

    # pdb.set_trace()
    # Positive flow for Buchi automaton in each case
    def ba_flow_src(model, key1, key2):
        # pdb.set_trace()
        sum = 0
        for ((k1,k2),i,j) in model.ba_edges:
            if i in src and (k1,k2) == (key1, key2):
                sum += model.ba_var['fba_e', k1, k2, i,j]
        return 1 <= sum
    model.ba_flow = pyo.Constraint(model.ba_keys, rule=ba_flow_src)

    return model

def add_automata_constraints(model, src, sink, int):
    G, B_prod, S = automata()
    map_q_to_de, sys_ba_de = group_cuts(G, B_prod, S) # Add constraint that at every state, S has a flow of atleast 1.
    model = add_sys_ba_vars(model, S, G, sys_ba_de) # Adds variables pertaining to the Buchi product automaton of the system transition and system spec
    # Group states in G according to cuts d_e that are active at that point
    model = add_sys_ba_constraints(model, S, G, sys_ba_de)
    # pdb.set_trace()
    return model

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

    # Call graphs for optimization:
    model = add_automata_constraints(model, src, sink, int)

    # Upper level Objective
    # Objective - minimize 1/F + lambda*f_3/F
    def mcf_flow(model):
        lam = 1
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
    def flow_sink(mdl):
        lam = 1
        return model.t + lam*sum(mdl.f3[i,j] for (i, j) in mdl.edges if j in sink)
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

if __name__ == '__main__':
    ts, prod_ba, virtual_ba, sys_virtual, state_map = create_ts_automata_and_virtual_game_graph()

    nodes = []
    node_dict = {}
    inv_node_dict = {}
    for i, node in enumerate(virtual_ba.nodes):
        nodes.append(i)
        node_dict.update({i: node})
        inv_node_dict.update({node: i})
    # find initial state
    init = []
    for initial in virtual_ba.states.initial:
        init.append(inv_node_dict[initial])
    # find accepting states for system and tester
    acc_sys = []
    acc_test = []
    for i in node_dict.keys():
        if 'accept' in node_dict[i][1][0]:
            acc_test.append(i)
        if 'accept' in node_dict[i][1][1]:
            acc_sys.append(i)
    # setup edges
    edges = []
    for edge in virtual_ba.edges:
        edges.append((inv_node_dict[edge[0]],inv_node_dict[edge[1]]))

    # Group cuts
    cleaned_intermed = [x for x in acc_test if x not in acc_sys]

    f1_e, f2_e, f3_e, d_e, F = solve_bilevel(nodes,edges, init, cleaned_intermed, acc_sys)
    pdb.set_trace()
