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

def convert_network_to_FTS(G, states, next_state_dict, init, lenx, leny):
    # st()
    ts = transys.FiniteTransitionSystem()
    ts_states = ["s"+str(k) for k in range(len(states))]
    init_idx = [k for k in range(len(states)) if states[k]==init]
    state_map = {ts_states[i]: states[i] for i in range(len(states))}
    ts.states.add_from(ts_states) # Add states
    ts.states.initial.add(ts_states[init_idx[0]])
    ts.sys_actions.add_from({'e', 'w', 'stay', ''} )
    ts.actions = ['n', 'e', 's', 'w', 'stay']
    # ts.atomic_propositions = []
    ts.atomic_propositions.add('(goal)')
    ts.atomic_propositions.add('(intermed)')
    # for xi in range(lenx):
    #     ts.atomic_propositions.add("x="+str(xi))
    # for yi in range(leny):
    #     ts.atomic_propositions.add("y="+str(yi))
    for i, si in enumerate(states):
        successors = next_state_dict[si]
        successor_idx = [states.index(succ) for succ in successors]
        tsi = ts_states[i]
        ts_succ = [ts_states[k] for k in successor_idx]
        if si[1] == 0 and si[0] == 0:
            # ap_tsi = ("x="+str(si[1]), "y="+str(si[0]), "goal")
            # ts.states[tsi]['ap'] = (ap_tsi,)
            ts.states.add(tsi, ap={'(goal)'})
        elif si[1] == 2 and si[0] == 0:
            # ap_tsi = ("x="+str(si[1]), "y="+str(si[0]), "intermed")
            # ts.states[tsi]['ap'] = (ap_tsi,)
            ts.states.add(tsi, ap={'(intermed)'})
        # else:
        #     ap_tsi = ("x="+str(si[1]), "y="+str(si[0]))
        #     ts.states[tsi]['ap'] = (ap_tsi,)
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
    # st()
    return ts, state_map

def get_BA(f, orig_guard):
    # f = '[]<>(intermed)'
    never_claim_f = ltl2baint.call_ltl2ba(f)
    symbols, g, initial, accepting = parser.parse(never_claim_f)
    S = list(g.nodes())
    S0 = [s for s in S if "init" in s]
    Sa = [s for s in S if "accept" in s]
    props = ['('+s+')' for s in symbols.keys()]
    props_orig = [orig_guard['('+s+')'] for s in symbols.keys()]
    props.append(True)
    props_orig.append(True)
    # Sigma = PowerSet(props)
    trans = []
    trans_orig = []
    for ui,vi,di in g.edges(data=True): # Reading the guarded labels
        # if di['guard'] == '(1)':
        #     di['guard'] = set()
        trans.append((ui,vi,di['guard']))
        # st()
        trans_orig.append((ui,vi,orig_guard[di['guard']]))
    ba = construct_BA(S, S0, Sa, props, trans)
    # print("BA successfully constructed!")
    ba_orig = construct_BA(S, S0, Sa, props_orig, trans_orig)
    # print("BA original successfully constructed!")
    # symbols, g, initial, accepting = parser.parse(ba)
    # print(ba.states.accepting) # Insert checks
    # print(ba_orig.states.accepting)
    assert ba.states.accepting == ba_orig.states.accepting # Verify that these are the same states
    return symbols, g, initial, accepting, ba, ba_orig

def construct_BA(S, S0, Sa, props, trans):
    # st()
    ba = BuchiAutomaton(atomic_proposition_based=True) # BA must be AP based
    ba.states.add_from(S)
    ba.states.initial.add_from(S0)
    ba.states.accepting.add_from(Sa)
    ba.atomic_propositions |= props
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
                    print(set([di]))
            elif isinstance(di, list):
                ba.transitions.add(ui, vi, letter=set(di))
                print(set(di))
            elif isinstance(di, tuple):
                ba.add_edge(ui, vi, letter=(di,))
                print(set(di))
            elif di == True:
                ba.transitions.add(ui, vi, letter=set([di]))
                print(set([di]))
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

def get_transition_system(mazefile):
    '''
    Convert the given network to a transition system in TuLiP.
    ts : Transition system
    state_map: Dictionary that maps states of the TS to the network nodes
    '''
    maze = CorridorNetwork(mazefile, (0,6), (0,2)) # Creates the maze object
    ts, state_map = convert_network_to_FTS(maze.gamegraph, maze.states, maze.next_state_dict, (0,4), maze.len_x, maze.len_y)
    print('Transition system constucted.')
    return ts, state_map

def get_tester_BA():
    '''
    Find the Büchi Automaton (BA) for the test specification.
    ba : BA using the AP 'intermed' as guard
    ba_orig: BA using the system coordinates as guard
    '''
    orig_guard = {'(intermed)': ('x=2', 'y=0'), '(1)':'(1)'}
    f = '[]<>(intermed)'
    symbols, g, initial, accepting, ba, ba_orig = get_BA(f, orig_guard) # BA conversion only for safety and progress
    ba.save('test_ba.pdf')
    ba_orig.save('test_ba_orig.pdf')
    print('Tester BA constucted.')
    return ba, ba_orig

def get_system_BA():
    '''
    Find the Büchi Automaton (BA) for the system specification.
    ba : BA using the AP 'goal' as guard
    ba_orig: BA using the system coordinates as guard
    '''
    orig_guard = {'(goal)': ('x=0', 'y=0'), '(1)':'(1)'}
    f = '[]<>(goal)'
    symbols, g, initial, accepting, ba, ba_orig = get_BA(f, orig_guard) # BA conversion only for safety and progress
    ba.save('sys_ba.pdf')
    ba_orig.save('sys_ba_orig.pdf')
    print('System BA constucted.')
    return ba, ba_orig

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
                            prod_trans.append((start_state,end_state,di['letter']))

    for ui,vi,di in sys_ba.edges(data=True): # Reading the guarded labels
        for start_state in S_prod:
                for end_state in S_prod:
                    if str(ui)+'_sys' == str(start_state[1]) and str(vi)+'_sys' == str(end_state[1]):
                        if start_state[0] == end_state[0]:
                            prod_trans.append((start_state,end_state,di['letter']))
    # st()
    prod_ba = construct_BA(S_prod, S0_prod, Sa_prod, props_prod, prod_trans)
    # st()
    prod_ba.save('product_ba.pdf')
    return prod_ba

def construct_virtual_product_automaton(prod_ba, ts):
    # st()
    virtual_prod_aut, virtual_accepting = products.ts_ba_sync_prod(ts, prod_ba)
    virtual_prod_aut.save('virtual.pdf')
    # st()
    return virtual_prod_aut, virtual_accepting

def construct_product_automaton(ba, ts):
    prod_aut, accepting = products.ts_ba_sync_prod(ts, ba)
    # prod_aut.save('example_prod_aut.pdf')
    # st()
    return prod_aut, accepting

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

if __name__ == '__main__':
    mazefile = "corridor_networkfile.txt"
    # Find the separate automata (ts, test_ba and sys_ba)
    ts, state_map = get_transition_system(mazefile)
    test_ba, test_ba_orig = get_tester_BA()
    sys_ba, sys_ba_orig = get_system_BA()

    # Check by running the TuLiP examples
    ba_tulip = ba_test()
    ts_tulip = ts_test()
    test_prod, acc = products.ts_ba_sync_prod(ts_tulip, ba_tulip)
    test_prod.save('prod_tulip.pdf')

    # Construct the product automata for just the transition system and one BA - just to check
    check_goal, goal_acc = construct_product_automaton(sys_ba, ts) # check if it goes to goal
    check_goal.save('prod_aut_sys_and_ts.pdf')
    st()

    # Find the product Buchi Automaton for the system and tester specs
    prod_ba = async_product_BAs(test_ba, sys_ba)
    st()

    # Construct the virtual priduct automaton synchronous product TS x BA_prod
    virtual_prod_aut, virtual_accepting = construct_virtual_product_automaton(prod_ba, ts)

    # test_virtual = OnTheFlyProductAutomaton(prod_ba, ts) # Should do the same
    st()
    print("Constructed Product automaton")
