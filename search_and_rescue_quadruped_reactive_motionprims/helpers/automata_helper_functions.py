
import tulip
import numpy as np
import networkx as nx
import logging
import sys
sys.path.append('..')
import warnings
from tulip.transys import transys, automata, products, algorithms, OnTheFlyProductAutomaton
from tulip import synth, spec
from tulip.interfaces.omega import _grspec_to_automaton
import tulip.interfaces.omega as omega_intf
from helpers.spec_tools import Spec, make_grspec, check_circular, check_specs
from tulip.spec import GRSpec, LTL
from tulip.transys.automata import BuchiAutomaton
from tulip.interfaces import ltl2ba as ltl2baint
from tulip.transys.labeled_graphs import LabeledDiGraph
from tulip.transys.automata import tuple2ba
from tulip.transys.mathset import PowerSet, MathSet, powerset
from itertools import chain, combinations
from tulip.transys import products
import pdb
from components.network import CorridorNetwork
from ipdb import set_trace as st
import networkx as nx
import matplotlib.pyplot as plt
from copy import deepcopy

parser = ltl2baint.Parser()

class GraphData:
    def __init__(self, nodes, edges, node_dict, inv_node_dict, acc_sys, acc_test, init):
        self.nodes = nodes
        self.edges = edges
        self.node_dict = node_dict
        self.inv_node_dict = inv_node_dict
        self.acc_sys = acc_sys
        self.acc_test = acc_test
        self.init = init
        self.graph = self.setup_graph(nodes, edges)

    def setup_graph(self, nodes, edges):
        G = nx.DiGraph()
        G.add_nodes_from(nodes)
        G.add_edges_from(edges)
        return G

def get_BA(f, orig_guard = None):
    never_claim_f = ltl2baint.call_ltl2ba(f)
    symbols, g, initial, accepting = parser.parse(never_claim_f)
    S = list(g.nodes())
    S0 = [s for s in S if "init" in s]
    Sa = [s for s in S if "accept" in s]
    pos_props = ['('+s+')' for s in symbols.keys()]
    neg_props = ['(not '+s+')' for s in symbols.keys()]
    props = pos_props + neg_props
    trans = []
    trans_orig = []
    for ui,vi,di in g.edges(data=True): # Reading the guarded labels
        transition = di['guard']
        trans.append((ui,vi,transition))

    ba = construct_BA(S, S0, Sa, props, trans)

    return symbols, g, initial, accepting, ba

def async_product_BAs(test_ba, sys_ba):
    # ba_prod = algorithms.sync_prod(test_ba, sys_ba)
    # st()
    S_prod = []
    for test_node in test_ba.states():
        for sys_node in sys_ba.states():
            S_prod.append((str(test_node)+'_test', str(sys_node)+'_sys'))

    S0_prod = [s for s in S_prod if "init" in s[0] and "init" in s[1]]
    Sa_prod = [s for s in S_prod if "accept" in s[0] or "accept" in s[1]]
    # st()
    props_prod = test_ba.atomic_propositions | sys_ba.atomic_propositions

    # Add transitions
    prod_trans = []
    # st()
    for ui,vi,di in test_ba.edges(data=True): # Reading the guarded labels
        for start_state in S_prod:
                for end_state in S_prod:
                    if str(ui)+'_test' == str(start_state[0]) and str(vi)+'_test' == str(end_state[0]):
                        if start_state[1] == end_state[1]:
                            # if di['letter'] != set():
                            prod_trans.append((start_state,end_state,di['letter']))

    for ui,vi,di in sys_ba.edges(data=True): # Reading the guarded labels
        for start_state in S_prod:
                for end_state in S_prod:
                    if str(ui)+'_sys' == str(start_state[1]) and str(vi)+'_sys' == str(end_state[1]):
                        if start_state[0] == end_state[0]:
                            # if di['letter'] != set():
                            prod_trans.append((start_state,end_state,di['letter']))

    # Check if transitions are duplicate - proposition takes precedence
    trans_list = remove_redundant_transitions(prod_trans)


    prod_ba = construct_BA(S_prod, S0_prod, Sa_prod, props_prod, trans_list)
    prod_ba.name = 'B_prod'
    return prod_ba

def construct_product_automaton(ba, ts): # synchronous product
    prod_aut = products.ba_ts_sync_prod(ba, ts)
    return prod_aut

def remove_redundant_empty_sets(trans_dict):
    transition_list = []
    for key in trans_dict.keys():
        # st()
        clean_transitions = []
        clean_transitions = [i for i in trans_dict[key] if i != set()]
        if clean_transitions == []:
            clean_transitions = [set()]
        for item in clean_transitions:
            transition_list.append((key[0], key[1], item))
    return transition_list

def remove_redundant_transitions(trans_list):
    trans_dict = get_trans_dict(trans_list)
    trans_list_clean = remove_redundant_empty_sets(trans_dict)
    return trans_list_clean

def get_trans_dict(trans_list):
    trans_dict = dict()
    # st()
    for k, trans in enumerate(trans_list):
        transitions = []
        transitions.append(trans[2])
        if (trans[0],trans[1]) in trans_dict.keys():
            transitions = transitions + trans_dict[(trans[0],trans[1])]
        trans_dict.update({(trans[0],trans[1]): transitions})
    # st()
    return trans_dict

def construct_BA(S, S0, Sa, props, trans):
    ba = BuchiAutomaton(atomic_proposition_based=True) # BA must be AP based
    ba.states.add_from(S)
    ba.states.initial.add_from(S0)
    ba.states.accepting.add_from(Sa)
    ba.atomic_propositions |= get_powerset_from_trans(props, trans)

    for ui,vi,di in trans:
        try:
            if isinstance(di, str):
                if di == '(1)':
                    ba.transitions.add(ui, vi, letter=set())
                else:
                    ba.transitions.add(ui, vi, letter=set([di]))
            elif isinstance(di, list):
                ba.transitions.add(ui, vi, letter=set(di))
            elif isinstance(di, tuple):
                ba.add_edge(ui, vi, letter=(di,))
            elif di == True:
                ba.transitions.add(ui, vi, letter=set([di]))
            elif isinstance(di,set):
                if di == set():
                    ba.transitions.add(ui, vi, letter=set())
                else:
                    ba.transitions.add(ui, vi, letter={str(deepcopy(di).pop())})
            else:
                ba.transitions.add(ui, vi, letter=di)
        except:
            st()
            pass
    return ba

def get_powerset_from_trans(props, trans):
    # st()
    pwrset = []
    for ui,vi,di in trans:
        if isinstance(di,set):
            if di != set():
                di_clean = str(deepcopy(di).pop())
            else:
                di_clean = set()
            if di_clean not in pwrset:
                pwrset.append(di_clean)
        else:
            if di not in pwrset:
                pwrset.append(di)

    # connector = ' and ' # add or too
    # pwrset = props
    # for prop in props:
    #     for nextprop in props:
    #         if prop != nextprop:
    #             if prop != 'not'+ nextprop[1:-1] and  nextprop != 'not'+ prop[1:-1]:
    #                 pwrset.append('('+prop+connector+nextprop+')')
    #                 pwrset.append('('+nextprop+connector+prop+')')
    # pass
    return pwrset

def make_graphs_for_optimization(prod_ba, virtual, sys_virtual, state_map):
    # virtual game graph info
    G = setup_nodes_and_edges_for_G(virtual)
    # prod ba info - probably not necessary, if yes return what is needed
    B_prod = setup_nodes_and_edges_for_prod_ba(prod_ba)
    # virtual system game graph info
    S = setup_nodes_and_edges_for_S(sys_virtual)
    return G, B_prod, S

def setup_nodes_and_edges_for_prod_ba(prod_ba):
    # setup nodes and map
    nodes = []
    node_dict = {}
    inv_node_dict = {}
    for i, node in enumerate(prod_ba.nodes):
        nodes.append(i)
        node_dict.update({i: node})
        inv_node_dict.update({node: i})
    # find initial state
    init = []
    for initial in prod_ba.states.initial:
        init.append(inv_node_dict[initial])
    # find accepting states for system and tester
    acc_sys = []
    acc_test = []
    for i in node_dict.keys():# states are labeled (tester,system)
        if 'accept' in node_dict[i][0]:
            acc_test.append(i)
        if 'accept' in node_dict[i][1]:
            acc_sys.append(i)
    # setup edges
    edges = []
    for edge in prod_ba.edges:
        edges.append((inv_node_dict[edge[0]],inv_node_dict[edge[1]]))

    B_prod = GraphData(nodes, edges, node_dict, inv_node_dict, acc_sys, acc_test, init)
    return B_prod

def setup_nodes_and_edges_for_G(virtual):
    # setup nodes and map
    nodes = []
    node_dict = {}
    inv_node_dict = {}
    for i, node in enumerate(virtual.nodes):
        nodes.append(i)
        node_dict.update({i: node})
        inv_node_dict.update({node: i})
    # find initial state
    init = []
    for initial in virtual.states.initial:
        init.append(inv_node_dict[initial])
    # find accepting states for system and tester
    acc_sys = []
    acc_test = []
    for i in node_dict.keys():# states are labeled (ts, (tester,system))
        if 'accept' in node_dict[i][1][1]: # check if system goal first, no duplicates
            acc_sys.append(i)
        elif 'accept' in node_dict[i][1][0]:
            acc_test.append(i)
    # setup edges
    edges = []
    for edge in virtual.edges:
        edges.append((inv_node_dict[edge[0]],inv_node_dict[edge[1]]))

    G = GraphData(nodes, edges, node_dict, inv_node_dict, acc_sys, acc_test, init)
    return G

def setup_nodes_and_edges_for_S(sys_virtual):
    # setup nodes and map
    nodes = []
    node_dict = {}
    inv_node_dict = {}
    for i, node in enumerate(sys_virtual.nodes):
        nodes.append(i)
        node_dict.update({i: node})
        inv_node_dict.update({node: i})
    # find initial state
    init = []
    for initial in sys_virtual.states.initial:
        init.append(inv_node_dict[initial])
    # find accepting states for system and tester
    acc_sys = []
    acc_test = []
    for i in node_dict.keys():# states are labeled (ts,system)
        if 'accept' in node_dict[i][1]:
            acc_sys.append(i)
    # setup edges
    edges = []
    for edge in sys_virtual.edges:
        edges.append((inv_node_dict[edge[0]],inv_node_dict[edge[1]]))

    S = GraphData(nodes, edges, node_dict, inv_node_dict, acc_sys, None, init)

    return S
