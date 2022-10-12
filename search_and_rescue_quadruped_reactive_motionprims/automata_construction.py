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
from components.network import CustomGrid
from helpers.automata_helper_functions import async_product_BAs, get_BA, construct_product_automaton
# build parser once only
parser = ltl2baint.Parser()

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
    # st()
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

def get_transition_system(mazefile): # from file
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

def setup_transition_systems(network): # for 3 door example
    # st()
    # TS T that includes all APs
    ts = transys.FiniteTransitionSystem()
    # TS that ony includes system APs
    ts_sys = transys.FiniteTransitionSystem()
    ts_states = ["s"+str(k) for k in network.map.keys()]
    init_idx =  [k for k in network.map.keys() if network.map[k]=='init']
    snr_to_nr = {ts_states[i]: int(ts_states[i][1:]) for i in range(len(ts_states))}
    snr_to_label = {k : network.map[snr_to_nr[k]] for k in ts_states}
    label_to_snr = {network.map[snr_to_nr[k]] : k for k in ts_states}
    # st()
    ts.states.add_from(ts_states) # Add states
    ts.states.initial.add(ts_states[init_idx[0]])
    ts.atomic_propositions.add('(goal)')
    ts.atomic_propositions.add('(stand)')
    ts.atomic_propositions.add('(lie)')
    ts.atomic_propositions.add('(jump)')

    ts_sys.states.add_from(ts_states) # Add states
    ts_sys.states.initial.add(ts_states[init_idx[0]])
    ts_sys.atomic_propositions.add('(goal)')

    # st()
    for i, si in enumerate(ts_states):
        successors = network.next_state_dict[snr_to_label[si]]
        succs_in_snr = [label_to_snr[k] for k in successors]
        if si == 's4':
            ts.states.add(si, ap={'(jump)'})
            # ts_sys.states.add(si, ap={'(goal)'})
        elif si == 's5':
            ts.states.add(si, ap={'(stand)'})
        elif si == 's6':
            ts.states.add(si, ap={'(stand)'})
        elif si == 's7':
            ts.states.add(si, ap={'(stand)'})
        elif si == 's8':
            ts.states.add(si, ap={'(lie)'})
        elif si == 's14':
            ts.states.add(si, ap={'(goal)'})
        for successor_snr in succs_in_snr:
            ts.transitions.add(si, successor_snr)#, sys_actions=act)
            ts_sys.transitions.add(si, successor_snr)
    ts.save('ts.pdf')
    ts_sys.save('ts_sys.pdf')
    return ts, ts_sys, snr_to_nr, snr_to_label, label_to_snr

def get_tester_BA(f): # from tester spec
    '''
    Find the Büchi Automaton (BA) for the test specification.
    ba : BA using the AP 'intermed' as guard
    ba_orig: BA using the system coordinates as guard
    '''
    # f = '<>(door1) && <>(door2) && <>(door3)'
    symbols, g, initial, accepting, ba = get_BA(f) # BA conversion only for safety and progress
    ba.name = 'B_test'
    ba.save('test_ba.pdf')
    # ba_orig.save('test_ba_orig.pdf')
    print('Tester BA constucted.')
    return ba

def get_system_BA(f): # from system spec
    '''
    Find the Büchi Automaton (BA) for the system specification.
    ba : BA using the AP 'goal' as guard
    ba_orig: BA using the system coordinates as guard
    '''
    # f = '<>[](goal)'
    symbols, g, initial, accepting, ba = get_BA(f) # BA conversion only for safety and progress
    ba.name = 'B_sys'
    ba.save('sys_ba.pdf')
    print('System BA constucted.')
    return ba

def create_ts_automata_and_virtual_game_graph(network):
    # Find the product Buchi Automaton for the system and tester specs
    f_test = '<>(jump) && <>(lie) && <>(stand)'
    test_ba = get_tester_BA(f_test)
    f_sys = '<>[](goal)'
    sys_ba = get_system_BA(f_sys)
    # Frind transition system
    # states = ['init', 'd1', 'd2', 'd3', 'goal']
    # transitions = [('init', 'd1'), ('init', 'd2'), ('init', 'd3'), ('d1', 'd2'), ('d2', 'd3'), ('d2', 'd1'), ('d3', 'd2'), ('d1', 'goal'), ('d2', 'goal'), ('d3', 'goal')]
    # network = CustomGrid(states, transitions)
    ts, ts_sys, snr_to_nr, snr_to_label, label_to_snr = setup_transition_systems(network)
    # asynchronous product
    prod_ba = async_product_BAs(test_ba, sys_ba)
    prod_ba.save('prod_ba.pdf')
    # virtual game graph
    virtual = construct_product_automaton(prod_ba, ts)
    # st()
    # add the states and transitions that the tulip synchronous product function disregarded
    # virtual.states.add(('s4', ('accept_all_test', 'T0_init_sys')))
    # virtual.states.add(('s9', ('accept_all_test', 'T0_init_sys')))
    # virtual.states.add(('s14', ('accept_all_test', 'accept_S2_sys')))
    # virtual.states.add(('s5', ('T1_S6_test', 'T0_init_sys')))
    # virtual.states.add(('s10', ('T1_S6_test', 'T0_init_sys')))
    # virtual.states.add(('s14', ('T1_S6_test', 'accept_S2_sys')))
    # virtual.states.accepting.add(('s4', ('accept_all_test', 'T0_init_sys')))
    # virtual.states.accepting.add(('s9', ('accept_all_test', 'T0_init_sys')))
    # virtual.states.accepting.add(('s14', ('accept_all_test', 'accept_S2_sys')))
    # virtual.states.accepting.add(('s14', ('T1_S6_test', 'accept_S2_sys')))
    virtual.transitions.add(('s2', ('T1_S6_test', 'T0_init_sys')) ,('s6', ('T1_S6_test', 'T0_init_sys')) ,  letter ={'(stand)'})
    virtual.transitions.add(('s3', ('T0_S3_test', 'T0_init_sys')) ,('s8', ('T0_S3_test', 'T0_init_sys')) ,  letter ={'(lie)'})
    virtual.transitions.add(('s3', ('T1_S6_test', 'T0_init_sys')) ,('s7', ('T1_S6_test', 'T0_init_sys')) ,  letter ={'(stand)'})
    virtual.transitions.add(('s3', ('T2_S7_test', 'T0_init_sys')) ,('s8', ('T2_S7_test', 'T0_init_sys')) ,  letter ={'(lie)'})
    virtual.transitions.add(('s1', ('T1_S6_test', 'T0_init_sys')),('s4', ('T1_S6_test', 'T0_init_sys')), letter ={'(jump)'})
    # virtual.transitions.add(('s3', ('T0_S4_test', 'T0_init_sys')) ,('s7', ('T0_S4_test', 'T0_init_sys')) ,  letter ={'(stand)'})
    virtual.transitions.add(('s3', ('T0_S4_test', 'T0_init_sys')),('s8', ('T0_S4_test', 'T0_init_sys')), letter ={'(lie)'})
    virtual.transitions.add(('s1', ('T0_S4_test', 'T0_init_sys')),('s5', ('T0_S4_test', 'T0_init_sys')), letter ={'(stand)'})
    virtual.transitions.add(('s1', ('T0_S2_test', 'T0_init_sys')),('s5', ('T0_S2_test', 'T0_init_sys')), letter ={'(stand)'})
    virtual.transitions.add(('s2', ('T0_S4_test', 'T0_init_sys')),('s6', ('T0_S4_test', 'T0_init_sys')), letter ={'(stand)'})
    virtual.transitions.add(('s1', ('T2_S7_test', 'T0_init_sys')) ,('s4', ('T2_S7_test', 'T0_init_sys')) ,  letter ={'(jump)'})
    virtual.transitions.add(('s1', ('T1_S5_test', 'T0_init_sys')) ,('s4', ('T1_S5_test', 'T0_init_sys')) ,  letter ={'(jump)'})
    virtual.transitions.add(('s3', ('T0_S4_test', 'T0_init_sys')),('s7', ('T0_S4_test', 'T0_init_sys')), letter ={'(stand)'})
    virtual.transitions.add(('s2', ('T0_S2_test', 'T0_init_sys')),('s6', ('T0_S2_test', 'T0_init_sys')), letter ={'(stand)'})
    virtual.transitions.add(('s3', ('T0_S2_test', 'T0_init_sys')),('s7', ('T0_S2_test', 'T0_init_sys')), letter ={'(stand)'})
    # virtual.transitions.add(('s4', ('accept_all_test', 'T0_init_sys')), ('s9', ('accept_all_test', 'T0_init_sys')) , letter = set())
    # virtual.transitions.add(('s9', ('accept_all_test', 'T0_init_sys')) ,('s14', ('accept_all_test', 'accept_S2_sys')), letter ={'(goal)'})
    virtual.transitions.add(('s1', ('T1_S6_test', 'T0_init_sys')),('s5', ('T1_S6_test', 'T0_init_sys')), letter ={'(stand)'})
    # virtual.transitions.add(('s5', ('T1_S6_test', 'T0_init_sys')),('s10', ('T1_S6_test', 'T0_init_sys')), letter =set())
    # virtual.transitions.add(('s10', ('T1_S6_test', 'T0_init_sys')),('s14', ('T1_S6_test', 'accept_S2_sys')),  letter ={'(goal)'})
    # virtual.transitions.add(('s10', ('T1_S6_test', 'T0_init_sys')),('s1', ('T1_S6_test', 'T0_init_sys')),  letter =set())

    # virtual.transitions.add(('s3', ('T0_S4_test', 'T0_init_sys')), ('s2', ('T0_S4_test', 'T0_init_sys')), letter={'(door2)'})
    # virtual.transitions.add(('s1', ('T2_S7_test', 'T0_init_sys')), ('s2', ('T2_S7_test', 'T0_init_sys')), letter={'(door2)'})
    virtual.save('virtual.pdf')
    print("Constructed virtual game graph")
    # system virtual game graph
    sys_virtual = construct_product_automaton(sys_ba, ts_sys) # check if it goes to goal
    sys_virtual.save('system_virtual.pdf')
    print("Constructed virtual game graph for the system")
    return ts, prod_ba, virtual, sys_virtual, snr_to_nr, snr_to_label, label_to_snr

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


    ts, prod_ba, virtual, sys_virtual, snr_to_nr, snr_to_label, label_to_snr = create_ts_automata_and_virtual_game_graph(network)

    st()
