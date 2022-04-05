## Apurva Badithela
# Constructing the product of a transition system with the Buchi _grspec_to_automaton
import tulip
import numpy as np
import networkx as nx
import logging
import warnings
from tulip.transys import transys, automata, products, algorithms
from tulip import synth, spec
from tulip.interfaces.omega import _grspec_to_automaton
import tulip.interfaces.omega as omega_intf
from spec_tools import Spec, make_grspec, check_circular, check_specs
from tulip.spec import GRSpec, LTL
from tulip.transys.automata import BuchiAutomaton
from tulip.interfaces import ltl2ba as ltl2baint
from tulip.transys.labeled_graphs import LabeledDiGraph
from tulip.transys.automata import tuple2ba
from itertools import chain, combinations

import pdb
# build parser once only
parser = ltl2baint.Parser()
# gridworld plots:
# Base grid
def base_grid(M, N):
    colors = dict(**mcolors.CSS4_COLORS)

    msz = 12 # Scaling of linear parameters
    lw= 2 # Linewidth
    fsz = 14 # fontsize for text

    fig = plt.figure()
    ax = plt.gca()

    # Setting up grid and static obstacles:
    # Grid matrix has extra row and column to accomodate the 1-indexing of the gridworld
    grid_matrix = np.zeros((M+1, N+1))

    # Setting up gridlines
    ax.set_xlim(0.5,N+0.5)
    ax.set_ylim(M+0.5, 0.5)

    # Labels for major ticks
    ax.set_xticklabels(np.arange(1, N+1, 1), minor='True')
    ax.set_yticklabels(np.flipud(np.arange(1, M+1, 1)), minor='True')

    # Gridlines based on minor ticks
    ygrid_lines = np.flipud(np.arange(1, M+1, 1)) - 0.5
    xgrid_lines = np.arange(1, N+1, 1) - 0.5

    # Major ticks
    ax.set_xticks(xgrid_lines)
    ax.set_yticks(ygrid_lines)

    # Minor ticks
    ax.set_yticks(ygrid_lines+0.5, minor='True')
    ax.set_xticks(xgrid_lines+0.5, minor='True')
    ax.grid(which='major', axis='both', linestyle='-', color=colors['silver'], linewidth=lw)
    plt.setp(ax.get_xmajorticklabels(), visible=False)
    plt.setp(ax.get_ymajorticklabels(), visible=False)
    return fig, ax

# grid specifications:
# new row and new column if stepped in direction dir from the location row, col
def grid_transitions(row, col, dir):
    if dir == "n":
        new_row = row-1
        new_col = col
    if dir == "e":
        new_row = row
        new_col = col+1
    if dir == "w":
        new_row = row
        new_col = col-1
    if dir == "s":
        new_row = row+1
        new_col = col
    return new_row, new_col

def generate_gridworld_transition_specs(M,N):
    pos = lambda irow, icol: N*(irow-1) + icol
    east = lambda irow, icol: N*(irow-1) + icol + 1
    west = lambda irow, icol: N*(irow-1) + icol - 1
    north = lambda irow, icol: N*(irow-2) + icol
    south = lambda irow, icol: N*(irow) + icol
    # dummy:
    safe_specs = set()
    Gf = nx.DiGraph()
    for irow in range(1,M+1):
        for icol in range(1,N+1):
            spec_i = '(x='+str(icol)+' && y='+str(irow)+') -> X('
            state = pos(irow, icol)
            succ_dir = ['n','s','e','w']
            successors = [east(irow, icol), west(irow,icol), north(irow, icol), south(irow,icol)]
            # Taking care of corners:
            if(irow == 1):
                rem_succ = [north(irow, icol)]
                rem_succ_dir = ['n']
                for r in rem_succ:
                    if r in successors:
                        successors.remove(r)
                for r in rem_succ_dir:
                    if r in succ_dir:
                        succ_dir.remove(r)

            if(irow == M):
                rem_succ = [south(irow, icol)]
                rem_succ_dir = ['s']
                for r in rem_succ:
                    if r in successors:
                        successors.remove(r)
                for r in rem_succ_dir:
                    if r in succ_dir:
                        succ_dir.remove(r)

            if(icol == 1):
                rem_succ = [west(irow, icol)]
                rem_succ_dir = ['w']
                for r in rem_succ:
                    if r in successors:
                        successors.remove(r)
                for r in rem_succ_dir:
                    if r in succ_dir:
                        succ_dir.remove(r)

            if(icol == N):
                rem_succ = [east(irow, icol)]
                rem_succ_dir = ['e']
                for r in rem_succ:
                    if r in successors:
                        successors.remove(r)
                for r in rem_succ_dir:
                    if r in succ_dir:
                        succ_dir.remove(r)

            # Make sure there are no negative vertices:
            for s in successors:
                if s <= 0:
                    print(state)
                assert(s>0)
            # Adding successors:
            for si in range(len(successors)):
                s = successors[si]
                sdir = succ_dir[si]
                if si == 0:
                    si_row, si_col = grid_transitions(irow, icol, sdir)
                    spec_i += '(x='+str(si_col)+' && y='+str(si_row)+')'
                else:
                    spec_i += ' || (x='+str(si_col)+' && y='+str(si_row)+')'
                u = "n"+str(state)
                v = "n"+str(s)
                Gf.add_edge(u, v, capacity=1.0)
            spec_i += ')'
            safe_specs |= {spec_i}
    return safe_specs, Gf

def sys_propf_construct():
    sys_vars = {}
    sys_init = set()
    sys_prog = set()
    sys_safe = set()
    M = 3
    N = 3
    sys_init |= {'(x=1 && y=1)'}
    sys_vars['x'] = (1,M)
    sys_vars['y'] = (1,N)
    safe_specs, Gf = generate_gridworld_transition_specs(M,N)
    sys_safe |= safe_specs
    sys_prog |= {'(x = 10 && y=10)'}
    return sys_init, sys_vars, sys_safe, sys_prog, Gf

# Make it go through
def test_propf_construct():
    test_vars = {'park', 'intermed'}
    test_init = {'intermed': False}
    test_safe = {'intermed <-> (x=2 && y=2)'} # Intermediate node
    test_prog = {'!park'}
    test_prog |= {'intermed'}
    # test_prog |= {'[]<>(x = 3 && y=3)'}
    return test_init, test_vars, test_safe, test_prog

# Def LTL2BA:
def construct_BA():
    sys_init, sys_vars, sys_safe, sys_prog, Gf = sys_propf_construct()
    test_init, test_vars, test_safe, test_prog = test_propf_construct()
    sys_spec = GRSpec(sys_vars=sys_vars, sys_init=set(), sys_safety=sys_safe, sys_prog = sys_prog)
    test_spec = GRSpec(env_vars=test_vars, env_init=set(), env_safety=test_safe, env_prog = test_prog)

    f = test_spec.to_canon()
    test_spec_BA = ltl2baint.call_ltl2ba(f)
    symbols, g, initial, accepting = parser.parse(test_spec_BA)
    return symbols, g, initial, accepting

def ltl2ba(formula):
    """Convert LTL formula to Buchi Automaton using ltl2ba.
    @type formula: `str(formula)` must be admissible ltl2ba input
    @return: Buchi automaton whose edges are annotated
        with Boolean formulas as `str`
    @rtype: [`Automaton`]
    """
    ltl2ba_out = ltl2baint.call_ltl2ba(str(formula))
    symbols, g, initial, accepting = parser.parse(ltl2ba_out)
    ba = Automaton('Buchi', alphabet=symbols)
    ba.add_nodes_from(g)
    ba.add_edges_from(g.edges(data=True))
    ba.initial_nodes = initial
    ba.accepting_sets = accepting
    logger.info('Resulting automaton:\n\n{ba}\n'.format(ba=ba))
    return ba

# Defining Powerset:
def powerset(A):
    length = len(A)
    return {
        frozenset({e for e, b in zip(A, f'{i:{length}b}') if b == '1'})
        for i in range(2 ** length)
    }

#Original guard of the function in set variable fashion.
def prog_BA_conversion(orig_guard):
    f = '[]<>(intermed)'
    test_spec_BA = ltl2baint.call_ltl2ba(f)
    symbols, g, initial, accepting = parser.parse(test_spec_BA)
    S = list(g.nodes())
    S0 = [s for s in S if "init" in s]
    Sa = [s for s in S if "accept" in s]
    props = set(['('+s+')' for s in symbols.keys()])
    Sigma = [set(s) for s in powerset(props)]
    trans = []
    trans_orig = []
    for u,v,d in g.edges(data=True): # Reading the guarded labels
        trans.append((u,v,d['guard']))
        trans_orig.append((u,v,orig_guard[d['guard']]))
    tp = [S, S0, Sa,Sigma, trans]
    tp_orig = [S, S0, Sa,Sigma, trans_orig]
    return symbols, g, initial, accepting, tp, tp_orig

# Define product automaton with Buchi automaton and transition system
if __name__ == '__main__':
    orig_guard = {'(intermed)': 'x=2', '(1)': True, '(0)':False}
    symbols, g, initial, accepting, tp, tp_orig = prog_BA_conversion(orig_guard) # BA conversion only for safety and progress psi specs, not others
    # Convert to Buchi automaton:
    pdb.set_trace()
    ba = tuple2ba(tp[0], tp[1], tp[2], tp[3], tp[4])
    ba_orig = tuple2ba(tp_orig[0], tp_orig[1], tp_orig[2], tp_orig[3], tp_orig[4])
    pdb.set_trace()
    symbols, g, initial, accepting = construct_BA()
