# Construct Virtual Product Automaton for Constraining Test Environments
# Apurva Badithela, Josefine Graebener
# California Institute of Technology, 2022

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
from spec_tools import Spec, make_grspec, check_circular, check_specs
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
# build parser once only
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


def convert_network_to_FTS_for_system(G, states, next_state_dict, init, lenx, leny):
    # TS T that only includes system APs such that tulip can create the product of B_sys x T
    ts = transys.FiniteTransitionSystem()
    ts_states = ["s"+str(k) for k in range(len(states))]
    init_idx = [k for k in range(len(states)) if states[k]==init]
    state_map = {ts_states[i]: states[i] for i in range(len(states))}
    ts.states.add_from(ts_states) # Add states
    ts.states.initial.add(ts_states[init_idx[0]])
    ts.sys_actions.add_from({'e', 'w', 'stay', ''} )
    ts.actions = ['n', 'e', 's', 'w', 'stay']
    ts.atomic_propositions.add('(goal)')

    for i, si in enumerate(states):
        successors = next_state_dict[si]
        successor_idx = [states.index(succ) for succ in successors]
        tsi = ts_states[i]
        ts_succ = [ts_states[k] for k in successor_idx]
        if si[1] == 0 and si[0] == 0:
            ts.states.add(tsi, ap={'(goal)'})
        elif si[1] == 8 and si[0] == 0:
            ts.states.add(tsi, ap={'(goal)'})#,'(not goal1)'})
        for k, ts_succ_k in enumerate(ts_succ):
            succ = successors[k]
            if succ[0] == si[0] and succ[1] == si[1]+1:
                act = 'e'
            elif succ[0] == si[0] and succ[1] == si[1]-1:
                act = 'w'
            elif succ[0] == si[0] and succ[1] == si[1]:
                act = 'stay'
            ts.transitions.add(tsi, ts_succ_k)#, sys_actions=act)
    ts.save('ts_sys.pdf')
    return ts, state_map

def convert_network_to_FTS(G, states, next_state_dict, init, lenx, leny):
    # TS that includes system and tester APs
    ts = transys.FiniteTransitionSystem()
    ts_states = ["s"+str(k) for k in range(len(states))]
    init_idx = [k for k in range(len(states)) if states[k]==init]
    state_map = {ts_states[i]: states[i] for i in range(len(states))}
    ts.states.add_from(ts_states) # Add states
    ts.states.initial.add(ts_states[init_idx[0]])
    ts.sys_actions.add_from({'e', 'w', 'stay', ''} )
    ts.actions = ['n', 'e', 's', 'w', 'stay']
    ts.atomic_propositions.add('(goal)')
    ts.atomic_propositions.add('(key1)')
    ts.atomic_propositions.add('(key2)')
    for i, si in enumerate(states):
        successors = next_state_dict[si]
        successor_idx = [states.index(succ) for succ in successors]
        tsi = ts_states[i]
        ts_succ = [ts_states[k] for k in successor_idx]
        if si[1] == 0 and si[0] == 0:
            ts.states.add(tsi, ap={'(goal)'})
        elif si[1] == 2 and si[0] == 0:
            ts.states.add(tsi, ap={'(key2)'})
        elif si[1] == 8 and si[0] == 0:
            ts.states.add(tsi, ap={'(goal)'})
        elif si[1] == 6 and si[0] == 0:
            ts.states.add(tsi, ap={'(key1)'})
        for k, ts_succ_k in enumerate(ts_succ):
            succ = successors[k]
            if succ[0] == si[0] and succ[1] == si[1]+1:
                act = 'e'
            elif succ[0] == si[0] and succ[1] == si[1]-1:
                act = 'w'
            elif succ[0] == si[0] and succ[1] == si[1]:
                act = 'stay'
            ts.transitions.add(tsi, ts_succ_k)#, sys_actions=act)
    ts.save('ts.pdf')
    return ts, state_map

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

def prepare_BA(f):
    # f = '[]<>(intermed)'
    # st()
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

    all_props = props
    for ui,vi,di in trans:
        if di not in all_props:
            all_props.append(di)

    return S, S0, Sa, all_props, trans

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

def construct_BA(S, S0, Sa, props, trans):
    # st()
    ba = BuchiAutomaton(atomic_proposition_based=True) # BA must be AP based
    ba.states.add_from(S)
    ba.states.initial.add_from(S0)
    ba.states.accepting.add_from(Sa)
    ba.atomic_propositions |= get_powerset_from_trans(props, trans)
    # ba.alphabet.math_set |= props
    # print(ba.alphabet.math_set)
    # st()
    for ui,vi,di in trans:
        try:
            if isinstance(di, str):
                if di == '(1)':
                    ba.transitions.add(ui, vi, letter=set())
                else:
                    ba.transitions.add(ui, vi, letter=set([di]))
                    # print(set([di]))
            elif isinstance(di, list):
                ba.transitions.add(ui, vi, letter=set(di))
                # print(set(di))
            elif isinstance(di, tuple):
                ba.add_edge(ui, vi, letter=(di,))
                # print(set(di))
            elif di == True:
                ba.transitions.add(ui, vi, letter=set([di]))
                # print(set([di]))
            # elif di == {True}:
            #     ba.transitions.add(ui, vi, letter=di)
            #     print(set([di]))
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
    # st()
    return ba

# def make_ba(S, S0, Sa, Sigma, trans):
#     ba = tuple2ba(S, S0, Sa, Sigma, trans, name='ba', prepend_str=None, atomic_proposition_based=True)
#     return ba

def get_transition_system(mazefile):
    '''
    Convert the given network to a transition system in TuLiP.
    ts : Transition system
    state_map: Dictionary that maps states of the TS to the network nodes
    '''
    maze = CorridorNetwork(mazefile, (0,6), (0,2)) # Creates the maze object
    ts, state_map = convert_network_to_FTS(maze.gamegraph, maze.states, maze.next_state_dict, (0,4), maze.len_x, maze.len_y)
    ts.name = 'TS'
    print('Transition system constucted.')
    return ts, state_map

def get_tester_BA():
    '''
    Find the Büchi Automaton (BA) for the test specification.
    ba : BA using the AP 'intermed' as guard
    ba_orig: BA using the system coordinates as guard
    '''
    # orig_guard = {'(intermed)': ('x=2', 'y=0'), '(1)':'(1)'}
    # f = '[]((!goal1 U key1) || (!goal2 U key2))'
    f = '<>(key1) && <>(key2)'
    symbols, g, initial, accepting, ba = get_BA(f) # BA conversion only for safety and progress
    ba.name = 'B_test'
    ba.save('test_ba.pdf')
    # ba_orig.save('test_ba_orig.pdf')
    print('Tester BA constucted.')
    return ba

def get_system_BA():
    '''
    Find the Büchi Automaton (BA) for the system specification.
    ba : BA using the AP 'goal' as guard
    ba_orig: BA using the system coordinates as guard
    '''
    orig_guard = {'(goal)': ('x=0', 'y=0'), '(1)':'(1)'}
    f = '<>[](goal)'
    symbols, g, initial, accepting, ba = get_BA(f, orig_guard) # BA conversion only for safety and progress
    ba.name = 'B_sys'
    ba.save('sys_ba.pdf')
    # ba_orig.save('sys_ba_orig.pdf')
    print('System BA constucted.')
    return ba

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
    # st()
    # for trans1 in prod_trans:
    #     for trans2 in prod_trans:
    #         if trans1[0] == trans2[0] and trans1[1] == trans2[1]:
    #             if trans1[2] == '(goal)' or trans1[2] == '(intermed)':
    #                 trans_to_keep.append()

    # Check if transitions are duplicate - proposition takes precedence
    trans_list = remove_redundant_transitions(prod_trans)


    prod_ba = construct_BA(S_prod, S0_prod, Sa_prod, props_prod, trans_list)
    prod_ba.name = 'B_prod'
    return prod_ba

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

def construct_virtual_product_automaton(prod_ba, ts):
    st()
    virtual_prod_aut, virtual_accepting = products.ts_ba_sync_prod(ts, prod_ba)
    virtual_prod_aut.save('virtual.pdf')
    st()
    return virtual_prod_aut, virtual_accepting

def construct_product_automaton(ba, ts):
    prod_aut = products.ba_ts_sync_prod(ba, ts)
    return prod_aut

def ba_test(): # From TuLiP source code
    ba = BuchiAutomaton()

    aps = ['p']
    ba.atomic_propositions |= {'p'}
    assert('p' in ba.atomic_propositions)
    assert(ba.atomic_propositions == MathSet(aps) )
    assert(ba.alphabet == PowerSet(aps) )


    ba.states.add_from({'q0', 'q1'})
    assert(set(ba.states) == {'q0', 'q1'})

    ba.states.initial.add('q0')
    assert(set(ba.states.initial) == {'q0'})

    ba.states.accepting.add('q1')
    assert(set(ba.states.accepting) == {'q1'})

    ba.transitions.add('q0', 'q1', letter={'p'})
    ba.transitions.add('q1', 'q1', letter={'p'})
    ba.transitions.add('q1', 'q0', letter=set() )
    ba.transitions.add('q0', 'q0', letter=set() )

    # logger.debug(ba)
    ba.save('ba_tulip.pdf')
    return ba

def ts_test(): # From TuLiP source code
    ts = transys.FiniteTransitionSystem()

    ts.states.add('s0')
    assert('s0' in ts)
    assert('s0' in ts.nodes)
    assert('s0' in ts.states)

    states = {'s0', 's1', 's2', 's3'}
    ts.states.add_from(states)
    assert(set(ts.states) == states)

    ts.transitions.add('s0', 's1')
    ts.transitions.add_from([('s1', 's2'), ('s2', 's3'), ('s3', 's0')])

    ts.states.initial.add('s0')
    ts.states.initial.add_from({'s0', 's1'})

    ts.atomic_propositions.add('p')
    assert(set(ts.atomic_propositions) == {'p'})

    ts.states['s0']['ap'] = {'p'}
    ts.states['s1']['ap'] = set()
    ts.states['s2']['ap'] = set()
    ts.states['s3']['ap'] = set()
    assert(ts.states['s0']['ap'] == {'p'})

    for state in {'s1', 's2', 's3'}:
        assert(ts.states[state]['ap'] == set() )

    # logger.debug(ts)
    ts.save('ts_tulip.pdf')
    return ts

def check_tulip(): # Check by running the TuLiP examples
    ba_tulip = ba_test()
    ts_tulip = ts_test()
    test_prod, acc = products.ts_ba_sync_prod(ts_tulip, ba_tulip)
    test_prod.save('prod_tulip.pdf')

def create_ts_automata_and_virtual_game_graph():
    mazefile = "corridor_networkfile.txt"
    # Find the transition system
    ts, state_map = get_transition_system(mazefile)
    # transition system with just the system labels
    maze = CorridorNetwork(mazefile, (0,6), (0,2)) # Creates the maze object
    testing_ts_for_sys, state_map = convert_network_to_FTS_for_system(maze.gamegraph, maze.states, maze.next_state_dict, (0,4), maze.len_x, maze.len_y)

    # Find the product Buchi Automaton for the system and tester specs
    test_ba = get_tester_BA()
    sys_ba = get_system_BA()

    prod_ba = async_product_BAs(test_ba, sys_ba)
    prod_ba.save('prod_ba.pdf')
    print("Constructed Product automaton")
    virtual = construct_product_automaton(prod_ba, ts) # check if it goes to goal

    # add the transitions that the tulip synchronous product function missed ??
    virtual.transitions.add(('s5', ('T1_S3_test', 'T0_init_sys')), ('s6', ('T1_S3_test', 'T0_init_sys')), letter=set())
    virtual.transitions.add(('s7', ('T1_S3_test', 'T0_init_sys')), ('s6', ('T1_S3_test', 'T0_init_sys')), letter=set())
    virtual.transitions.add(('s1', ('T0_S2_test', 'T0_init_sys')), ('s2', ('T0_S2_test', 'T0_init_sys')), letter=set())
    virtual.transitions.add(('s3', ('T0_S2_test', 'T0_init_sys')), ('s2', ('T0_S2_test', 'T0_init_sys')), letter=set())
    virtual.transitions.add(('s5', ('accept_all_test', 'T0_init_sys')), ('s6', ('accept_all_test', 'T0_init_sys')), letter=set())
    virtual.transitions.add(('s7', ('accept_all_test', 'T0_init_sys')), ('s6', ('accept_all_test', 'T0_init_sys')), letter=set())
    virtual.transitions.add(('s1', ('accept_all_test', 'T0_init_sys')), ('s2', ('accept_all_test', 'T0_init_sys')), letter=set())
    virtual.transitions.add(('s3', ('accept_all_test', 'T0_init_sys')), ('s2', ('accept_all_test', 'T0_init_sys')), letter=set())

    virtual.save('virtual.pdf')
    print("Constructed virtual game graph")
    sys_virtual = construct_product_automaton(sys_ba, testing_ts_for_sys) # check if it goes to goal
    sys_virtual.save('system_virtual.pdf')
    print("Constructed virtual game graph for the system")
    return ts, prod_ba, virtual, sys_virtual, state_map

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


if __name__ == '__main__':
    # will return tulip ts, bas and a map from ts states to actual coordinates in the form ('s0' : (0,0))
    ts, prod_ba, virtual, sys_virtual, state_map = create_ts_automata_and_virtual_game_graph()
    # G, B_prod, S are GraphData objects with the corresponding attributes
    G, B_prod, S = make_graphs_for_optimization(prod_ba, virtual, sys_virtual, state_map)
    st()
