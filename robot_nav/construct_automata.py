## Apurva Badithela
# Constructing the product of a transition system with the Buchi _grspec_to_automaton
import tulip
import numpy as np
import networkx as nx
import logging
import sys
sys.path.append('..')
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
from tulip.transys.mathset import PowerSet, MathSet, powerset
from itertools import chain, combinations
from tulip.transys import products
import pdb
from static_obstacle_maze.network import MazeNetwork
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
def GRspec_BA():
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

def construct_BA(S, S0, Sa, props, trans):
    ba = BuchiAutomaton(atomic_proposition_based=True) # BA must be AP based
    ba.states.add_from(S)
    ba.states.initial.add_from(S0)
    ba.states.accepting.add_from(Sa)
    ba.alphabet.math_set |= props
    # pdb.set_trace()
    for ui,vi,di in trans:
        try:
            if isinstance(di, str):
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
            else:
                ba.transitions.add(ui, vi, letter=di)
        except:
            pdb.set_trace()
    return ba

# Construct product automaton:
def product_automaton(ts, ba):
    prod = products.ts_ba_sync_prod(ts, ba)
    preim_acc_states = prod[1] # Set of accepting states of the Buchi automaton projected onto TS * BA.
    prodts = prod[0]
    prod_ba = products.ba_ts_sync_prod(ba, ts)
    ts_acc_states = set()
    for prod_st in preim_acc_states:
        ts_acc_states |= {prod_st[0]}
    return prodts, preim_acc_states, ts_acc_states

# Convert to FTS:
def convert_grid_to_FTS(G, states, next_state_dict, init, lenx, leny):
    ts = transys.FiniteTransitionSystem()
    ts_states = ["s"+str(k) for k in range(len(states))]
    init_idx = [k for k in range(len(states)) if states[k]==init]
    state_map = {ts_states[i]: states[i] for i in range(len(states))}
    ts.states.add_from(ts_states) # Add states
    ts.states.initial.add(ts_states[init_idx[0]])
    ts.actions = ['n', 'e', 's', 'w', 'stay']
    ts.atomic_propositions = []
    for xi in range(lenx):
        ts.atomic_propositions.append("x="+str(xi))
    for yi in range(leny):
        ts.atomic_propositions.append("y="+str(yi))
    for i, si in enumerate(states):
        successors = next_state_dict[si]
        successor_idx = [states.index(succ) for succ in successors]
        tsi = ts_states[i]
        ts_succ = [ts_states[k] for k in successor_idx]
        if si[1] == 2 and si[0] == 2:
            ap_tsi = ("x="+str(si[1]), "y="+str(si[0]))
            ts.states[tsi]['ap'] = (ap_tsi,)
        for k, ts_succ_k in enumerate(ts_succ):
            succ = successors[k]
            if succ[0] == si[0] and succ[1] == si[1]+1:
                act = 'e'
            elif succ[0] == si[0] and succ[1] == si[1]-1:
                act = 'w'
            elif succ[0] == si[0]+1 and succ[1] == si[1]:
                act = 's'
            elif succ[0] == si[0]-1 and succ[1] == si[1]:
                act = 'n'
            elif succ[0] == si[0] and succ[1] == si[1]:
                act = 'stay'
            ts.transitions.add(tsi, ts_succ_k)
            # pdb.set_trace()
    return ts, state_map

# Get transition system and Buchi automaton:
def get_ba_ts(mazefile):
    maze = MazeNetwork(mazefile) # Creates the maze
    G, states, next_state_dict = maze.get_gamegraph() # Get finite transition system format.
    # pdb.set_trace()
    ts, state_map = convert_grid_to_FTS(G, states, next_state_dict, (0,maze.len_y-1), maze.len_x, maze.len_y)
    # pdb.set_trace()
    spec = maze.transition_specs()
    
    # Specification
    orig_guard = {'(intermed)': ('x=2', 'y=2'), True:True}
    symbols, g, initial, accepting, ba, ba_orig = prog_BA_conversion(orig_guard) # BA conversion only for safety and progress
    # pdb.set_trace()
    return ts, ba, ba_orig, state_map

# Write product BA by hand // construct it automatically and map it to known states by hand
# Construct BA for the entire test specification.
#Original guard of the function in set variable fashion.
def prog_BA_conversion(orig_guard):
    f = '[]<>(intermed)'
    test_spec_BA = ltl2baint.call_ltl2ba(f)
    symbols, g, initial, accepting = parser.parse(test_spec_BA)
    S = list(g.nodes())
    S0 = [s for s in S if "init" in s]
    Sa = [s for s in S if "accept" in s]
    props = ['('+s+')' for s in symbols.keys()]
    props_orig = [orig_guard['('+s+')'] for s in symbols.keys()]
    props.append(True)
    props_orig.append(True)
    Sigma = PowerSet(props)
    trans = []
    trans_orig = []
    for ui,vi,di in g.edges(data=True): # Reading the guarded labels
        if di['guard'] == '(1)':
            di['guard'] = True
        trans.append((ui,vi,di['guard']))
        trans_orig.append((ui,vi,orig_guard[di['guard']]))
    #pdb.set_trace()
    ba = construct_BA(S, S0, Sa, props, trans)
    print("BA successfully constructed!")
    ba_orig = construct_BA(S, S0, Sa, props_orig, trans_orig)
    print("BA original successfully constructed!")
    # symbols, g, initial, accepting = parser.parse(ba)
    print(ba.states.accepting) # Insert checks
    print(ba_orig.states.accepting)
    assert ba.states.accepting == ba_orig.states.accepting # Verify that these are the same
    # pdb.set_trace()
    return symbols, g, initial, accepting, ba, ba_orig

# Define product automaton with Buchi automaton and transition system
if __name__ == '__main__':
    # Constructing the product automaton with the game graph
    # Convert to Buchi automaton:
    mazefile = "../static_obstacle_maze/maze2.txt"
    ts, ba, ba_orig, state_map = get_ba_ts(mazefile)
    product_aut, preim_acc_states, ts_acc_states = product_automaton(ts, ba_orig)
    print("Constructed Product automaton")
    pdb.set_trace()
    # Product ts does not have s1_accept in it.
    # symbols, g, initial, accepting = construct_BA()
