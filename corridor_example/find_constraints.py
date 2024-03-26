import sys
sys.path.append('..')
import numpy as np
from ipdb import set_trace as st
from automata_construction import create_virtual_game_graph
import networkx as nx
import matplotlib.pyplot as plt
from tulip.transys.automata import BuchiAutomaton
from tulip.transys import transys
from solve_pyomo_bilevel import solve_bilevel

def setup_virtual_game_graph():
    virtual_ba, virtual_ts, virtual_ts_acc, maze, state_map = create_virtual_game_graph()
    return virtual_ba, virtual_ts_acc, maze, state_map

def setup_nodes_and_edges(virtual_ba):
    # setup nodes and map
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
    return nodes, edges, node_dict, inv_node_dict, acc_sys, acc_test, init

def call_pyomo(nodes,edges, init, intermed, goal):
    # st()
    cleaned_intermed = [x for x in intermed if x not in goal]
    f1_e, f2_e, f3_e, d_e, F = solve_bilevel(nodes,edges, init, cleaned_intermed, goal)
    # st()
    cuts = [x for x in d_e.keys() if d_e[x] >= 0.9 ]
    flow = F
    print('Cut {} edges in the virtual game graph.'.format(len(cuts)))
    print('The max flow through I is {}'.format(F))
    return cuts, flow

def get_game_graph_and_cuts(cuts, nodes, edges, node_dict, inv_node_dict, state_map, init):
    G = nx.DiGraph()
    G.add_nodes_from(nodes)
    G.add_edges_from(edges)
    # add the missing edges - FIX WHY MISSING???
    G.add_edge(*(inv_node_dict[('s1', ('T1_S3_test', 'T0_init_sys'))],inv_node_dict[('s2', ('T1_S3_test', 'T0_init_sys'))]))
    G.add_edge(*(inv_node_dict[('s7', ('T1_S2_test', 'T0_init_sys'))],inv_node_dict[('s6', ('T1_S2_test', 'T0_init_sys'))]))
    return G, state_map, node_dict, inv_node_dict, cuts


def find_cuts():
    virtual_ba, virtual_ts_acc, maze, state_map = setup_virtual_game_graph()
    nodes, edges, node_dict, inv_node_dict, acc_sys, acc_test, init = setup_nodes_and_edges(virtual_ba)
    cuts, flow = call_pyomo(nodes, edges, init, acc_test, acc_sys)
    G, state_map, node_dict, inv_node_dict, cuts = get_game_graph_and_cuts(cuts, nodes, edges, node_dict, inv_node_dict, state_map, init)
    st()
    return G, state_map, node_dict, inv_node_dict, init, cuts

if __name__ == '__main__':
    find_cuts()
