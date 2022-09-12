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
from components.network import CustomGrid

# def setup_virtual_game_graph():
#     virtual_ba, virtual_ts, virtual_ts_acc, maze, state_map = create_virtual_game_graph()
#     return virtual_ba, virtual_ts_acc, maze, state_map

def setup_automata(network):
    ts, prod_ba, virtual, sys_virtual, snr_to_nr, snr_to_label, label_to_snr = create_ts_automata_and_virtual_game_graph(network)
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


def find_cuts(network):
    # states = ['init', 'p1', 'p2', 'p3', 'jump1', 'lie2', 'stand3', 'd1', 'd2', 'd3', 'goal']
    # transitions = [('init', 'p1'), ('init', 'p2'), ('init', 'p3'), ('p1', 'jump1'), ('p2', 'lie2'), \
    # ('p3', 'stand3'), ('jump1', 'd1'),('lie2', 'd2'), ('stand3', 'd3'), \
    # ('d2', 'p1'), ('d3', 'p2'), ('d1', 'p2'), ('d2', 'p3'), \
    # ('p2', 'p1'), ('p3', 'p2'), ('p1', 'p2'), ('p2', 'p3'), \
    # ('d1', 'goal'), ('d2', 'goal'), ('d3', 'goal')]
    # network = CustomGrid(states, transitions)

    virtual, sys_virtual, snr_to_nr, snr_to_label, label_to_snr = setup_automata(network)

    nodes, edges, node_dict, inv_node_dict, acc_sys, acc_test, init = setup_nodes_and_edges(virtual)
    cuts = []
    # while len(cuts) != 7:
    cuts, flow = call_pyomo(nodes, edges, init, acc_test, acc_sys)

    G = get_graph(nodes, edges) # virtual game graph in networkx graph form
    st()
    return G, node_dict, inv_node_dict, init, cuts, snr_to_nr, snr_to_label, label_to_snr



if __name__ == '__main__':
    states = ['init', 'p1', 'p2', 'p3', 'jump1', 'stand1', 'stand2', 'stand3', 'lie3',\
    'd1_j', 'd1_s', 'd2_s', 'd3_s', 'd3_l', 'goal']
    transitions = [('init', 'p1'), ('init', 'p2'), ('init', 'p3'), \
    ('p1', 'jump1'), ('p1', 'stand1'), ('p2', 'stand2'), ('p3', 'stand3'), ('p3', 'lie3'), \
    ('jump1', 'd1_j'),('stand1', 'd1_s'), ('stand2', 'd2_s'), ('stand3', 'd3_s'), ('lie3', 'd3_l'), \
    ('d1_s', 'p1'), ('d1_j', 'p1'),\
    ('d2_s', 'p2'), \
    ('d3_s', 'p3'), ('d3_l', 'p3'),\
    ('p2', 'p1'), ('p3', 'p2'), ('p1', 'p2'), ('p2', 'p3'), \
    ('d1_s', 'goal'), ('d1_j', 'goal'), ('d2_s', 'goal'), ('d3_s', 'goal'),  ('d3_l', 'goal')]
    network = CustomGrid(states, transitions)
    
    find_cuts(network)
