# Converting network flow formulations into convex-concave min-max optimization
# Vectors: x = [f1e/F, f2e/F, de/F, F/F], y = [f3e/F]
# TO Do (11/8/22)
# 1. Check if max oracle corresponds to the right Pyomo code
# 2. Verify every step of the max oracle algorithm and see if it checks out.
# Need to fix problem data
import sys
sys.path.append('..')
import numpy as np
from ipdb import set_trace as st
from collections import OrderedDict as od
import _pickle as pickle
import os
import networkx as nx
from road_network.components.road_network import RoadNetwork, create_network_from_file
from static_obstacle_maze.network import MazeNetwork
import matplotlib.pyplot as plt
from scipy import sparse as sp
from optimization import Vout, Vin, Vin_oracle, max_oracle_gd, max_oracle_pyomo, projx, gradient
import pdb

debug = False

def initialize(maze):
    G = nx.DiGraph(maze.gamegraph)
    # remove self loops
    edges = list(G.edges())
    for i,j in edges:
        if i == j:
            G.remove_edge(i,j)
    nodes = list(G.nodes())
    nodes_keys = {k: v for k,v in enumerate(nodes)}
    edges = list(G.edges())
    edges_keys = {k: e for k,e in enumerate(edges)}
    vars_x = ['f1_e', 'f2_e', 'd_e', 'F']
    vars_y = ['f3_e']
    edges_dict = {k: 0 for k in edges} # Edges have flow.
    x = {k: edges_dict for k in vars_x}
    y = {k: edges_dict for k in vars_y}
    return x, y, G, nodes_keys, edges_keys

# Prefix z: zeros
def objective(edges_keys):
    ne = len(list(edges_keys.keys())) # number of edges
    zf1 = np.zeros((ne,1))
    zf2 = np.zeros((ne,1))
    of3 = np.ones((ne,1))
    zde = np.zeros((ne,1))
    ot = 1.0/ne*np.ones((ne,1))

    c1 = np.vstack((zf1, zf2, zde, ot)) # zf: zero vector for f, and ot: one vector for t
    c2 = of3.copy()
    # Objective function: c1.T x + c2.T y
    return c1, c2

def populate_constraint_names(name_list, edges_keys=None, nodes_keys=None):
    constraint_names = []
    if edges_keys is not None:
        for k, edge in edges_keys.items():
            for name in name_list:
                edge_str = str(edge[0][0])+","+str(edge[0][1])+","+str(edge[1][0])+","+str(edge[1][1])
                constraint_names.extend([name+"["+edge_str+"]"])
    if nodes_keys is not None:
        for k, node in nodes_keys.items():
            for name in name_list:
                node_str = str(node[0])+","+str(node[1])
                constraint_names.extend([name+"["+node_str+"]"])
    return constraint_names

def feas_constraint(edges_keys, projection=False):
    feas_names = []
    ne = len(list(edges_keys.keys())) # number of edges
    Af1 = np.eye(ne)
    Af2 = np.eye(ne)
    Af3 = np.eye(ne)
    Ade = np.eye(ne)
    At = np.eye(ne)
    blk_zeros = np.zeros((ne,ne))

    if projection:
        feas_constraints_names = ["feas1", "feas2", "feas_t", "feas_de"]
        b_feas = np.zeros((4*ne,1))
        A_feas_f1= np.hstack((Af1, blk_zeros, blk_zeros, blk_zeros))
        A_feas_f2= np.hstack((blk_zeros, Af2, blk_zeros, blk_zeros))
        A_feas_t = np.hstack((blk_zeros, blk_zeros, blk_zeros, At))
        A_feas_de = np.hstack((blk_zeros, blk_zeros, Ade, blk_zeros))
        A_feas = np.vstack((A_feas_f1, A_feas_f2, A_feas_de, A_feas_t))
        assert A_feas.shape[1] == 4*ne
    else:
        b_feas = np.zeros((5*ne,1))
        feas_constraints_names = ["feas1", "feas2", "feas_t", "feas_de", "feas3"]
        A_feas_f1= np.hstack((Af1, blk_zeros, blk_zeros, blk_zeros, blk_zeros))
        A_feas_f2= np.hstack((blk_zeros, Af2, blk_zeros, blk_zeros, blk_zeros))
        A_feas_f3= np.hstack((blk_zeros, blk_zeros, blk_zeros, blk_zeros, Af3))
        A_feas_t = np.hstack((blk_zeros, blk_zeros, blk_zeros, At, blk_zeros))
        A_feas_de = np.hstack((blk_zeros, blk_zeros, Ade, blk_zeros, blk_zeros))
        A_feas = np.vstack((A_feas_f1, A_feas_f2, A_feas_de, A_feas_t, A_feas_f3))
        assert A_feas.shape[1] == 5*ne
    feas_names = populate_constraint_names(feas_constraints_names, edges_keys=edges_keys)
    assert A_feas.shape[0] == b_feas.shape[0]

    return A_feas, b_feas, feas_names

def cut_constraint(edges_keys, projection=False):
    cut_names = []
    ne = len(list(edges_keys.keys())) # number of edges
    Af1 = -1*np.eye(ne)
    Af2 = -1*np.eye(ne)
    Af3 = -1*np.eye(ne)
    Ade = -1*np.eye(ne)
    Aot = np.eye(ne)
    blk_zeros = np.zeros((ne,ne))

    if projection:
        cut_constraints_names = ["cut_cons1", "cut_cons2"]
        A_feas_f1= np.hstack((Af1, blk_zeros, Ade, Aot))
        A_feas_f2= np.hstack((blk_zeros, Af2, Ade, Aot))
        A_feas = np.vstack((A_feas_f1, A_feas_f2))
        b_feas = np.zeros((2*ne,1))
    else:
        cut_constraints_names = ["cut_cons1", "cut_cons2", "cut_cons3"]
        A_feas_f1= np.hstack((Af1, blk_zeros, Ade, Aot, blk_zeros))
        A_feas_f2= np.hstack((blk_zeros, Af2, Ade, Aot, blk_zeros))
        A_feas_f3= np.hstack((blk_zeros, blk_zeros, Ade, Aot, Af3))
        A_feas = np.vstack((A_feas_f1, A_feas_f2, A_feas_f3))
        b_feas = np.zeros((3*ne,1))
    cut_names = populate_constraint_names(cut_constraints_names, edges_keys=edges_keys)

    assert A_feas.shape[0] == b_feas.shape[0]
    return A_feas, b_feas, cut_names

def conservation_helper_function(edges_keys, nodes_keys, start, target, con_names):
    ne = len(list(edges_keys.keys())) # number of edges
    nv = len(list(nodes_keys.keys())) # number of edges
    module_mtrx = np.zeros((nv,ne)) # One matrix for holding all conservation terms
    Afeas1 = module_mtrx.copy()
    Afeas2 = module_mtrx.copy()

    con_f1 = []
    con_f2 = []
    for k,node in nodes_keys.items():
        if node not in {start, target}:
        # Afeas1: sum_i f^i(u,v) >= sum_i f^i(v,u)
        # Afeas2: sum_i f^i(u,v) <= sum_i f^i(v,u)
            out_node_edge_ind = [k for k, v in edges_keys.items() if v[0]==node]
            in_node_edge_ind = [k for k, v in edges_keys.items() if v[1]==node]
            for out_ind in out_node_edge_ind:
                Afeas1[k][out_ind] = -1
                Afeas2[k][out_ind] = 1
            for in_ind in in_node_edge_ind:
                Afeas1[k][in_ind] = 1
                Afeas2[k][in_ind] = -1
    con_f1 = populate_constraint_names(con_names, edges_keys=None, nodes_keys=nodes_keys)
    con_f2 = populate_constraint_names(con_names, edges_keys=None, nodes_keys=nodes_keys)
    return Afeas1, Afeas2, con_f1, con_f2

def proj_conservation_constraint(nodes_keys, edges_keys, src, int, sink):
    ne = len(list(edges_keys.keys())) # number of edges
    nv = len(list(nodes_keys.keys())) # number of edges
    module_mtrx = np.zeros((nv,ne)) # One matrix for holding all conservation
    module_vec = np.zeros((nv,1))

    Afeas1_f1, Afeas2_f1, con1_f1, con2_f1 = conservation_helper_function(edges_keys, nodes_keys, src, int, ["con1"])
    Afeas1_f2, Afeas2_f2, con1_f2, con2_f2 = conservation_helper_function(edges_keys, nodes_keys, int, sink, ["con2"])

    # Constructing block matrices:
    Acons1_f1 = np.hstack((Afeas1_f1, module_mtrx, module_mtrx, module_mtrx))
    Acons2_f1 = np.hstack((Afeas2_f1, module_mtrx, module_mtrx, module_mtrx))

    Acons1_f2 = np.hstack((module_mtrx, Afeas1_f2, module_mtrx, module_mtrx))
    Acons2_f2 = np.hstack((module_mtrx, Afeas2_f2, module_mtrx, module_mtrx))

    # Final assembly:
    cons_names = []
    cons_names.extend(con1_f1)
    cons_names.extend(con2_f1)
    cons_names.extend(con1_f2)
    cons_names.extend(con2_f2)

    Acons1 = np.vstack((Acons1_f1, Acons1_f2))
    Acons2 = np.vstack((Acons2_f1, Acons2_f2))
    bcons1 = np.vstack((module_vec, module_vec))
    bcons2 = np.vstack((module_vec, module_vec))
    Acons = np.vstack((Acons1, Acons2))
    bcons = np.vstack((bcons1, bcons2))
    assert Acons.shape[0] == bcons.shape[0]
    assert Acons.shape[1] == 4*ne
    return Acons, bcons, cons_names

def conservation_constraint(nodes_keys, edges_keys, src, int, sink):
    cons_names =[]
    ne = len(list(edges_keys.keys())) # number of edges
    nv = len(list(nodes_keys.keys())) # number of edges
    module_mtrx = np.zeros((nv,ne)) # One matrix for holding all conservation
    module_vec = np.zeros((nv,1))

    Afeas1_f1, Afeas2_f1, con1_f1, con2_f1 = conservation_helper_function(edges_keys, nodes_keys, src, int,["con1"])
    Afeas1_f2, Afeas2_f2, con1_f2, con2_f2 = conservation_helper_function(edges_keys, nodes_keys, int, sink, ["con2"])
    Afeas1_f3, Afeas2_f3, con1_f3, con2_f3= conservation_helper_function(edges_keys, nodes_keys, src, sink, ["con3"])

    # Constructing block matrices:
    Acons1_f1 = np.hstack((Afeas1_f1, module_mtrx, module_mtrx, module_mtrx, module_mtrx))
    Acons2_f1 = np.hstack((Afeas2_f1, module_mtrx, module_mtrx, module_mtrx, module_mtrx))

    Acons1_f2 = np.hstack((module_mtrx, Afeas1_f2, module_mtrx, module_mtrx, module_mtrx))
    Acons2_f2 = np.hstack((module_mtrx, Afeas2_f2, module_mtrx, module_mtrx, module_mtrx))

    Acons1_f3 = np.hstack((module_mtrx, module_mtrx, module_mtrx, module_mtrx, Afeas1_f3))
    Acons2_f3 = np.hstack((module_mtrx, module_mtrx, module_mtrx, module_mtrx, Afeas2_f3))

    # Final assembly:
    cons_names.extend(con1_f1)
    cons_names.extend(con2_f1)
    cons_names.extend(con1_f2)
    cons_names.extend(con2_f2)
    cons_names.extend(con1_f3)
    cons_names.extend(con2_f3)

    Acons1 = np.vstack((Acons1_f1, Acons1_f2, Acons1_f3))
    Acons2 = np.vstack((Acons2_f1, Acons2_f2, Acons2_f3))
    bcons1 = np.vstack((module_vec, module_vec, module_vec))
    bcons2 = np.vstack((module_vec, module_vec, module_vec))
    Acons = np.vstack((Acons1, Acons2))
    bcons = np.vstack((bcons1, bcons2))
    assert Acons.shape[0] == bcons.shape[0]
    assert Acons.shape[1] == 5*ne
    return Acons, bcons, cons_names

def min_flow_constraint(edges_keys, src, int,sink, projection=False):
    flow_names = []
    ne = len(list(edges_keys.keys())) # number of edges
    out_s1_edge_ind = [k for k, v in edges_keys.items() if v[0]==src]
    out_s2_edge_ind = [k for k, v in edges_keys.items() if v[0]==int]
    out_s3_edge_ind = [k for k, v in edges_keys.items() if v[0]==src]

    zero_row_vec = np.zeros((1,ne))
    af1 = np.zeros((1, ne))
    af2 = np.zeros((1,ne))
    af3 = np.zeros((1,ne))

    for k in out_s1_edge_ind:
        af1[0,k] = 1
    for k in out_s2_edge_ind:
        af2[0,k] = 1
    for k in out_s3_edge_ind:
        af3[0,k] = 1

    if projection:
        a1 = np.hstack((af1, zero_row_vec, zero_row_vec, zero_row_vec))
        a2 = np.hstack((zero_row_vec, af2, zero_row_vec, zero_row_vec))
        bfeas = np.ones((2,1))
        Afeas = np.vstack((a1, a2))
    else:
        a1 = np.hstack((af1, zero_row_vec, zero_row_vec, zero_row_vec, zero_row_vec))
        a2 = np.hstack((zero_row_vec, af2, zero_row_vec, zero_row_vec, zero_row_vec))
        # a3 = np.hstack((zero_row_vec, zero_row_vec, zero_row_vec, zero_row_vec, af3))
        # bfeas = np.ones((3,1))
        # Afeas = np.vstack((a1, a2, a3))
        bfeas = np.ones((2,1))
        Afeas = np.vstack((a1, a2))
    assert Afeas.shape[0] == bfeas.shape[0]
    return Afeas, bfeas, flow_names

def capacity_constraint(edges_keys, projection=False):
    feas_names = []
    ne = len(list(edges_keys.keys())) # number of edges
    Af1 = -1*np.eye(ne)
    Af2 = -1*np.eye(ne)
    Af3 = -1*np.eye(ne)
    Ade = -1*np.eye(ne)
    Aot = np.eye(ne)
    blk_zeros = np.zeros((ne,ne))

    # x constraints
    if projection:
        b_feas = np.zeros((3*ne,1))
        A_feas_f1= np.hstack((Af1, blk_zeros, blk_zeros, Aot))
        A_feas_f2= np.hstack((blk_zeros, Af2, blk_zeros, Aot))
        A_feas_de= np.hstack((blk_zeros, blk_zeros, Ade, Aot))
        A_feas = np.vstack((A_feas_f1, A_feas_f2, A_feas_de))
    else:
        b_feas = np.zeros((4*ne,1))
        A_feas_f1= np.hstack((Af1, blk_zeros, blk_zeros, Aot, blk_zeros))
        A_feas_f2= np.hstack((blk_zeros, Af2, blk_zeros, Aot, blk_zeros))
        A_feas_f3= np.hstack((blk_zeros, blk_zeros, blk_zeros, Aot, Af3))
        A_feas_de= np.hstack((blk_zeros, blk_zeros, Ade, Aot, blk_zeros))
        A_feas = np.vstack((A_feas_f1, A_feas_f2, A_feas_de, A_feas_f3))
    assert A_feas.shape[0] == b_feas.shape[0]
    return A_feas, b_feas, feas_names

# Equality constraints on 1/t:
def eq_aux_constraint(edges_keys, projection=False):
    ne = len(list(edges_keys.keys())) # number of edges
    eq_names = []
    eq_block = np.array([[1,-1],[-1,1]])
    # Aeq_t = np.zeros((2*ne-2, ne))
    # beq = np.zeros((2*ne-2,1))
    # # Take care of t
    # for k in range(ne-1):
    #     At = np.zeros((2,ne))
    #     At[0:2, k:k+2] = eq_block
    #     Aeq_t[k:k+2] = At.copy()

    Aeq_t = np.zeros((2*ne-2, ne))
    beq = np.zeros((2*ne-2,1))
    # Take care of t
    for k in range(ne-1):
        At = np.zeros((2,ne))
        At[0:2, k:k+2] = eq_block
        Aeq_t[2*k:2*k+2] = At.copy()
    # pdb.set_trace()
    # Organize larger matrices:
    zblock = 0*Aeq_t
    if projection:
        Aeq = np.hstack((zblock, zblock, zblock, Aeq_t))
    else:
        Aeq = np.hstack((zblock, zblock, zblock, Aeq_t, zblock))
    assert Aeq.shape[0] == beq.shape[0]
    return Aeq, beq, eq_names

# Collecting all constraints together as g(x,y) = A[x;y] - b>=0
def all_constraints(edges_keys, nodes_keys, src, int, sink):
    A_feas, b_feas, feas_names = feas_constraint(edges_keys)
    A_cap, b_cap, cap_names = capacity_constraint(edges_keys)
    A_cons, b_cons, cons_names = conservation_constraint(nodes_keys, edges_keys, src, int, sink)
    A_eq, b_eq, eq_names = eq_aux_constraint(edges_keys)
    A_cut, b_cut, cut_names = cut_constraint(edges_keys)
    A_flow, b_flow, flow_names = min_flow_constraint(edges_keys, src, int, sink)
    # A = np.vstack((A_feas, A_cut, A_cap, A_flow, A_cons, A_eq))
    # b = np.vstack((b_feas, b_cut, b_cap, b_flow, b_cons, b_eq))
    # pdb.set_trace()
    A = np.vstack((A_feas, A_cap, A_cons, A_eq, A_cut, A_flow))
    b = np.vstack((b_feas, b_cap, b_cons, b_eq, b_cut, b_flow))
    constraint_names = [feas_names, cap_names, cons_names, eq_names, cut_names, flow_names]
    assert A.shape[0] == b.shape[0]
    return A, b

# Collecting projection constraints together as g(x,y) = A[x;y] - b>=0
def proj_constraints(edges_keys, nodes_keys, src, int, sink):
    A_feas, b_feas, feas_names = feas_constraint(edges_keys, projection=True)
    A_cap, b_cap, cap_names = capacity_constraint(edges_keys, projection=True)
    A_cons, b_cons, cons_names = proj_conservation_constraint(nodes_keys, edges_keys, src, int, sink)
    A_eq, b_eq, eq_names = eq_aux_constraint(edges_keys, projection=True)
    A_cut, b_cut, cut_names = cut_constraint(edges_keys, projection=True)
    A_flow, b_flow, flow_names = min_flow_constraint(edges_keys, src, int, sink, projection=True)
    # A = np.vstack((A_feas, A_cap))
    # b = np.vstack((b_feas, b_cap))
    A = np.vstack((A_feas, A_cap, A_cons, A_eq, A_flow, A_cut))
    b = np.vstack((b_feas, b_cap, b_cons, b_eq, b_flow, b_cut))
    assert A.shape[0] == b.shape[0]
    return A, b

# Matching edges:
def match_edges(edges_keys, ne, flow_dict):
    flow_init = np.zeros((ne,1))
    # for k, v in flow_dict.items():
    for k, edge in edges_keys.items():
        flow_init[k,0] = flow_dict[edge[0]][edge[1]] # Parsing the flow dict
    return flow_init

# Function to get a candidate initial condition:
def get_candidate_flows(G, edges_keys, src, int, sink):
    ne = len(list(edges_keys.keys()))
    Gnx = nx.DiGraph()
    for edge in G.edges():
        Gnx.add_edge(*edge, capacity=1.0)
    f1e_value, f1e_dict = nx.maximum_flow(Gnx, src, int)
    f2e_value, f2e_dict = nx.maximum_flow(Gnx, int, sink)
    f3e_value, f3e_dict = nx.maximum_flow(Gnx, src, sink)
    F_init = min(f1e_value, f2e_value)
    assert F_init > 0.0
    f1e_init = match_edges(edges_keys, ne, f1e_dict) # Match flow values to edges keys consistent with the indices used in our optimization.
    f2e_init = match_edges(edges_keys, ne, f2e_dict)
    f3e_init = match_edges(edges_keys, ne, f3e_dict)
    tfac = 1.0/F_init
    t_init = tfac * np.ones((ne,1))
    zero_cuts = np.zeros((ne,1))
    x0 = np.vstack((f1e_init*tfac, f2e_init*tfac, zero_cuts, t_init))
    return x0, f3e_init

def plot_matrix(M, fn):
    fn = os.getcwd() + "/" + fn
    plt.matshow(M)
    plt.show()
    plt.savefig(fn)

def solve_opt(maze, src, sink, int):
    x, y, G, nodes_keys, edges_keys = initialize(maze)
    x0, y0 = get_candidate_flows(G, edges_keys, src, int, sink)
    Aineq,bineq = all_constraints(edges_keys, nodes_keys, src, int, sink)
    Aproj, bproj = proj_constraints(edges_keys, nodes_keys, src, int, sink)
    assert x0.shape[0] + y0.shape[0] == Aineq.shape[1]
    # pdb.set_trace()
    c1, c2 = objective(edges_keys)
    ne = len(list(edges_keys.keys())) # number of edges

    T = 20
    eta = 0.01
    # Vin_oracle(edges_keys, nodes_keys, src, sink, int, x0) #x0 is the wrong size
    xtraj, ytraj = max_oracle_gd(T, x0, eta, c1, c2, Aineq, bineq, Aproj, bproj, edges_keys, nodes_keys, src, sink, int, maze=maze)
    # xtraj, ytraj = max_oracle_pyomo(T, x0, eta, c1, c2, Aineq, bineq, Aproj, bproj, edges_keys, nodes_keys, src, sink, int, maze=maze)
    # Vin(c1, c2, A, b, x0, edges_keys)


if __name__ == '__main__':
    # test
    grid = "toy"
    main_dir = os.getcwd()
    par_dir = os.path.dirname(main_dir)
    if grid == "large":
        networkfile = par_dir + '/road_network/large_road_network.txt'
        src = (8,2)
        sink = (2,8)
        int = (5,5)
        maze = RoadNetwork(networkfile)

    elif grid == "small":
        networkfile = par_dir + '/road_network/road_network.txt'
        src = (4,2)
        sink = (2,0)
        int = (2,4)
        maze = RoadNetwork(networkfile)

    elif grid == "toy":
        mazefile = par_dir + '/constrained_mcf/small_mazefile.txt'
        src = (0,0)
        sink = (0,2)
        int = (2,1)
        maze = MazeNetwork(mazefile)

    reg = 10
    solve_opt(maze, src, sink, int)