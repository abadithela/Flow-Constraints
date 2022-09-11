import sys
sys.path.append('..')
import numpy as np
from ipdb import set_trace as st
from automata_construction import create_ts_automata_and_virtual_game_graph
import networkx as nx
import matplotlib.pyplot as plt
from tulip.transys.automata import BuchiAutomaton
from tulip.transys import transys
from solve_pyomo_bilevel import solve_bilevel

# def setup_virtual_game_graph():
#     virtual_ba, virtual_ts, virtual_ts_acc, maze, state_map = create_virtual_game_graph()
#     return virtual_ba, virtual_ts_acc, maze, state_map

def setup_automata():
    ts, prod_ba, virtual, sys_virtual, snr_to_nr, snr_to_label, label_to_snr = create_ts_automata_and_virtual_game_graph()
    return virtual, sys_virtual, snr_to_nr, snr_to_label, label_to_snr

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

def get_graph(nodes, edges):
    G = nx.DiGraph()
    G.add_nodes_from(nodes)
    G.add_edges_from(edges)
    return G


def find_cuts():
    virtual, sys_virtual, snr_to_nr, snr_to_label, label_to_snr = setup_automata()

    nodes, edges, node_dict, inv_node_dict, acc_sys, acc_test, init = setup_nodes_and_edges(virtual)
    cuts = []
    while len(cuts) != 7:
        cuts, flow = call_pyomo(nodes, edges, init, acc_test, acc_sys)

    G = get_graph(nodes, edges) # virtual game graph in networkx graph form
    # st()
    return G, node_dict, inv_node_dict, init, cuts, snr_to_nr, snr_to_label, label_to_snr



if __name__ == '__main__':
    find_cuts()
