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
from optimization_pyomo import Vout, Vin, Vin_oracle, max_oracle_gd, max_oracle_pyomo, projx, gradient
import pdb

# Initialize:

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
    ot = 1.0/ne*np.ones((1,1))

    c1 = np.vstack((zf1, zf2, zde, ot)) # zf: zero vector for f, and ot: one vector for t
    c2 = of3.copy()
    # Objective function: c1.T x + c2.T y
    return c1, c2

def populate_constraint_names(name_list, edges_keys=None, nodes_keys=None):
    constraint_names = []
    if edges_keys:
        for k, edge in edges_keys.items():
            for name in name_list:
                edge_str = str(edge[0][0])+","+str(edge[0][1])+","+str(edge[1][0])+","+str(edge[1][1])
                constraint_names.append(name+"["+edge_str+"]")
    elif nodes_keys:
        for k, node in nodes_keys.items():
            for name in name_list:
                node_str = str(node[0])+","+str(node[1])
                constraint_names.append(name+"["+node_str+"]")
    return constraint_names

# Constraints here correspond to max_flow_oracle_fullg:
# Capacity constraint
def capacity_constraint(edges_keys, projection=False):
    feas_names = []
    ne = len(list(edges_keys.keys())) # number of edges
    Af1 = -1*np.eye(ne)
    Af2 = -1*np.eye(ne)
    Af3 = -1*np.eye(ne)
    Ade = -1*np.eye(ne)
    Aot = np.ones((ne,1))
    blk_zeros = np.zeros((ne,ne))

    # x constraints
    if projection:
        b_feas = np.zeros((3*ne,1))
        feas_constraints_names = ["cap1", "cap2", "capd_e"]
        A_feas_f1= np.hstack((Af1, blk_zeros, blk_zeros, Aot))
        A_feas_f2= np.hstack((blk_zeros, Af2, blk_zeros, Aot))
        A_feas_de= np.hstack((blk_zeros, blk_zeros, Ade, Aot))
        A_feas = np.vstack((A_feas_f1, A_feas_f2, A_feas_de))
    else:
        b_feas = np.zeros((4*ne,1))
        feas_constraints_names = ["cap1", "cap2", "capd_e", "cap3"]
        A_feas_f1= np.hstack((Af1, blk_zeros, blk_zeros, Aot, blk_zeros))
        A_feas_f2= np.hstack((blk_zeros, Af2, blk_zeros, Aot, blk_zeros))
        A_feas_f3= np.hstack((blk_zeros, blk_zeros, blk_zeros, Aot, Af3))
        A_feas_de= np.hstack((blk_zeros, blk_zeros, Ade, Aot, blk_zeros))
        A_feas = np.vstack((A_feas_f1, A_feas_f2, A_feas_de, A_feas_f3))
    feas_names = populate_constraint_names(feas_constraints_names, edges_keys=edges_keys)
    assert A_feas.shape[0] == b_feas.shape[0]
    return A_feas, b_feas, feas_names

# Conservation constraint
def conservation_helper_function(edges_keys, nodes_keys, start, target, con_names):
    ne = len(list(edges_keys.keys())) # number of edges
    nv = len(list(nodes_keys.keys())) # number of edges
    module_mtrx = np.zeros((nv-2,ne)) # One matrix for holding all conservation terms except for the source and target.
    Afeas = module_mtrx.copy()
    nodes_keys_subset = dict()
    j=0
    for k,node in nodes_keys.items():
        if node not in {start, target}:
            nodes_keys_subset.update({k:node})
            out_node_edge_ind = [k for k, v in edges_keys.items() if v[0]==node]
            in_node_edge_ind = [k for k, v in edges_keys.items() if v[1]==node]
            for out_ind in out_node_edge_ind:
                Afeas[j][out_ind] = -1
            for in_ind in in_node_edge_ind:
                Afeas[j][in_ind] = 1
            j += 1
    con = populate_constraint_names(con_names, edges_keys=None, nodes_keys=nodes_keys_subset)

    return Afeas, con

def proj_conservation_constraint(nodes_keys, edges_keys, src, int, sink):
    cons_names =[]
    ne = len(list(edges_keys.keys())) # number of edges
    nv = len(list(nodes_keys.keys())) # number of edges
    module_mtrx = np.zeros((nv-2,ne)) # One matrix for holding all conservation
    module_vec = np.zeros((nv-2,1))

    Afeas_f1, con_f1 = conservation_helper_function(edges_keys, nodes_keys, src, int,["con1"])
    Afeas_f2,con_f2 = conservation_helper_function(edges_keys, nodes_keys, int, sink, ["con2"])

    # Constructing block matrices:
    Acons_f1 = np.hstack((Afeas_f1, module_mtrx, module_mtrx, module_vec))
    Acons_f2 = np.hstack((module_mtrx, Afeas_f2, module_mtrx, module_vec))
    # Final assembly:
    cons_names.extend(con_f1)
    cons_names.extend(con_f2)

    Acons = np.vstack((Acons_f1, Acons_f2))
    bcons = np.vstack((module_vec, module_vec))

    assert Acons.shape[0] == bcons.shape[0]
    return Acons, bcons, cons_names

def conservation_constraint(nodes_keys, edges_keys, src, int, sink):
    #  Equality cosntraints Acons, bcons
    cons_names =[]
    ne = len(list(edges_keys.keys())) # number of edges
    nv = len(list(nodes_keys.keys())) # number of edges
    module_mtrx = np.zeros((nv-2,ne)) # One matrix for holding all conservation
    module_vec = np.zeros((nv-2,1))

    Afeas_f1, con_f1 = conservation_helper_function(edges_keys, nodes_keys, src, int,["con1"])
    Afeas_f2,con_f2 = conservation_helper_function(edges_keys, nodes_keys, int, sink, ["con2"])
    Afeas_f3,con_f3= conservation_helper_function(edges_keys, nodes_keys, src, sink, ["con3"])

    # Constructing block matrices:
    Acons_f1 = np.hstack((Afeas_f1, module_mtrx, module_mtrx, module_vec, module_mtrx))
    Acons_f2 = np.hstack((module_mtrx, Afeas_f2, module_mtrx, module_vec, module_mtrx))
    Acons_f3 = np.hstack((module_mtrx, module_mtrx, module_mtrx, module_vec, Afeas_f3))

    # Final assembly:
    cons_names.extend(con_f1)
    cons_names.extend(con_f2)
    cons_names.extend(con_f3)

    Acons = np.vstack((Acons_f1, Acons_f2, Acons_f3))
    bcons = np.vstack((module_vec, module_vec, module_vec))
    assert len(cons_names) == Acons.shape[0]
    assert Acons.shape[0] == bcons.shape[0]
    return Acons, bcons, cons_names

# Min flow constraint:
def min_flow_constraint(edges_keys, src, int,sink, projection=False):
    flow_names = ["min_flow1", "min_flow2"]
    ne = len(list(edges_keys.keys())) # number of edges
    out_s1_edge_ind = [k for k, v in edges_keys.items() if v[0]==src]
    out_s2_edge_ind = [k for k, v in edges_keys.items() if v[0]==int]

    zero_row_vec = np.zeros((1,ne))
    af1 = np.zeros((1, ne))
    af2 = np.zeros((1,ne))

    for k in out_s1_edge_ind:
        af1[0,k] = 1
    for k in out_s2_edge_ind:
        af2[0,k] = 1

    if projection:
        a1 = np.hstack((af1, zero_row_vec, zero_row_vec, np.zeros((1,1))))
        a2 = np.hstack((zero_row_vec, af2, zero_row_vec, np.zeros((1,1))))
    else:
        a1 = np.hstack((af1, zero_row_vec, zero_row_vec, np.zeros((1,1)), zero_row_vec))
        a2 = np.hstack((zero_row_vec, af2, zero_row_vec, np.zeros((1,1)), zero_row_vec))

    bfeas = np.ones((2,1))
    Afeas = np.vstack((a1, a2))
    assert Afeas.shape[0] == bfeas.shape[0]
    return Afeas, bfeas, flow_names

# Cut constraint:
def cut_constraint(edges_keys, projection=False):
    cut_names = []
    ne = len(list(edges_keys.keys())) # number of edges
    Af1 = -1*np.eye(ne)
    Af2 = -1*np.eye(ne)
    Af3 = -1*np.eye(ne)
    Ade = -1*np.eye(ne)
    Aot = np.ones((ne,1))
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

# No in source
def no_in_source(edges_keys, src, int, sink, projection=False):
    flow_names = []

    ne = len(list(edges_keys.keys())) # number of edges
    in_s1_edge_ind = [k for k, v in edges_keys.items() if v[1]==src]
    in_s2_edge_ind = [k for k, v in edges_keys.items() if v[1]==int]
    in_s3_edge_ind = [k for k, v in edges_keys.items() if v[1]==src]

    af1 = np.zeros((len(in_s1_edge_ind), ne))
    af2 = np.zeros((len(in_s2_edge_ind), ne))
    af3 = np.zeros((len(in_s3_edge_ind), ne))

    s1_edges = dict()
    s2_edges = dict()
    s3_edges = dict()
    for i, k in enumerate(in_s1_edge_ind):
        s1_edges[k] = edges_keys[k]
        af1[i, k] = 1

    for i, k in enumerate(in_s2_edge_ind):
        s2_edges[k] = edges_keys[k]
        af2[i, k] = 1

    for i, k in enumerate(in_s3_edge_ind):
        s3_edges[k] = edges_keys[k]
        af3[i, k] = 1

    flow_names1 = populate_constraint_names(["no_in_source1"], edges_keys=s1_edges)
    flow_names2 = populate_constraint_names(["no_in_source2"], edges_keys=s2_edges)
    flow_names.extend(flow_names1)
    flow_names.extend(flow_names2)
    if projection:
        a1 = np.hstack((af1, 0*af1, 0*af1, np.zeros((af1.shape[0],1))))
        a2 = np.hstack((0*af2, af2, 0*af2, np.zeros((af2.shape[0],1))))
        Afeas = np.vstack((a1, a2))
    else:
        a1 = np.hstack((af1, 0*af1, 0*af1, np.zeros((af1.shape[0],1)), 0*af1))
        a2 = np.hstack((0*af2, af2, 0*af2, np.zeros((af2.shape[0],1)), 0*af2))
        a3 = np.hstack((0*af3, 0*af3, 0*af3, np.zeros((af3.shape[0],1)), af3))
        flow_names3 = populate_constraint_names(["no_in_source3"], edges_keys=s3_edges)
        flow_names.extend(flow_names3)
        Afeas = np.vstack((a1, a2, a3))

    bfeas = np.zeros((Afeas.shape[0],1))
    assert Afeas.shape[0] == bfeas.shape[0]
    return Afeas, bfeas, flow_names

# No out sink
def no_out_sink(edges_keys, src, int, sink, projection=False):
    flow_names = []

    ne = len(list(edges_keys.keys())) # number of edges
    out_t1_edge_ind = [k for k, v in edges_keys.items() if v[0]==int]
    out_t2_edge_ind = [k for k, v in edges_keys.items() if v[0]==sink]
    out_t3_edge_ind = [k for k, v in edges_keys.items() if v[0]==sink]

    af1 = np.zeros((len(out_t1_edge_ind), ne))
    af2 = np.zeros((len(out_t2_edge_ind), ne))
    af3 = np.zeros((len(out_t3_edge_ind), ne))

    t1_edges = dict()
    t2_edges = dict()
    t3_edges = dict()

    for i, k in enumerate(out_t1_edge_ind):
        t1_edges[k] = edges_keys[k]
        af1[i, k] = 1

    for i, k in enumerate(out_t2_edge_ind):
        t2_edges[k] = edges_keys[k]
        af2[i, k] = 1

    for i, k in enumerate(out_t3_edge_ind):
        t3_edges[k] = edges_keys[k]
        af3[i, k] = 1

    flow_names1 = populate_constraint_names(["no_out_sink1"], edges_keys=t1_edges)
    flow_names2 = populate_constraint_names(["no_out_sink2"], edges_keys=t2_edges)
    flow_names.extend(flow_names1)
    flow_names.extend(flow_names2)
    if projection:
        a1 = np.hstack((af1, 0*af1, 0*af1, np.zeros((af1.shape[0],1))))
        a2 = np.hstack((0*af2, af2, 0*af2, np.zeros((af2.shape[0],1))))
        Afeas = np.vstack((a1, a2))
    else:
        a1 = np.hstack((af1, 0*af1, 0*af1, np.zeros((af1.shape[0],1)), 0*af1))
        a2 = np.hstack((0*af2, af2, 0*af2, np.zeros((af2.shape[0],1)), 0*af2))
        a3 = np.hstack((0*af3, 0*af3, 0*af3, np.zeros((af3.shape[0],1)), af3))
        flow_names3 = populate_constraint_names(["no_out_sink3"], edges_keys=t3_edges)
        flow_names.extend(flow_names3)
        Afeas = np.vstack((a1, a2, a3))

    bfeas = np.zeros((Afeas.shape[0],1))
    assert Afeas.shape[0] == bfeas.shape[0]
    return Afeas, bfeas, flow_names

# No in/out intermediate
def no_in_out_interm(edges_keys, src, int, sink, projection=False):
    flow_names = []

    ne = len(list(edges_keys.keys())) # number of edges
    in_int_edge_ind = [k for k, v in edges_keys.items() if v[1]==int]
    out_int_edge_ind = [k for k, v in edges_keys.items() if v[0]==int]

    af3_in = np.zeros((len(in_int_edge_ind), ne))
    af3_out = np.zeros((len(out_int_edge_ind), ne))

    in_edges = dict()
    out_edges = dict()

    for i, k in enumerate(in_int_edge_ind):
        in_edges[k] = edges_keys[k]
        af3_in[i, k] = 1

    for i, k in enumerate(out_int_edge_ind):
        out_edges[k] = edges_keys[k]
        af3_out[i, k] = 1

    flow_names1 = populate_constraint_names(["no_in_interm"], edges_keys=in_edges)
    flow_names2 = populate_constraint_names(["no_out_interm"], edges_keys=out_edges)
    flow_names.extend(flow_names1)
    flow_names.extend(flow_names2)

    a3_in = np.hstack((0*af3_in, 0*af3_in, 0*af3_in, np.zeros((af3_in.shape[0],1)), af3_in))
    a3_out = np.hstack((0*af3_out, 0*af3_out, 0*af3_out, np.zeros((af3_out.shape[0],1)), af3_out))

    Afeas = np.vstack((a3_in, a3_out))
    bfeas = np.zeros((Afeas.shape[0],1))
    assert Afeas.shape[0] == bfeas.shape[0]
    return Afeas, bfeas, flow_names

# Equality constraint:
def eq_aux_constraint(edges_keys, projection=False):
    ne = len(list(edges_keys.keys())) # number of edges
    eq_names = []
    eq_block = np.array([[1,-1]])

    Aeq_t = np.zeros((ne-1, ne))
    beq = np.zeros((ne-1,1))
    # Take care of t
    for k in range(ne-1):
        At = np.zeros((1,ne))
        At[0,k:k+2] = eq_block
        Aeq_t[k] = At.copy()
    # pdb.set_trace()
    # Organize larger matrices:
    zblock = 0*Aeq_t
    eq_names = populate_constraint_names(["eq"], edges_keys=edges_keys)
    if projection:
        Aeq = np.hstack((zblock, zblock, zblock, Aeq_t))
    else:
        Aeq = np.hstack((zblock, zblock, zblock, Aeq_t, zblock))
    assert Aeq.shape[0] == beq.shape[0]
    return Aeq, beq, eq_names

# Collecting all constraints together as g(x,y) = A[x;y] - b>=0
def all_constraints(edges_keys, nodes_keys, src, int, sink):
    A_cap, b_cap, cap_names = capacity_constraint(edges_keys)
    A_cons, b_cons, cons_names = conservation_constraint(nodes_keys, edges_keys, src, int, sink)
    # A_eq, b_eq, eq_names = eq_aux_constraint(edges_keys)
    A_cut, b_cut, cut_names = cut_constraint(edges_keys)
    A_flow, b_flow, flow_names = min_flow_constraint(edges_keys, src, int, sink)
    A_no_in_src, b_no_in_src, no_in_src_names = no_in_source(edges_keys, src, int, sink)
    A_no_out_sink, b_no_out_sink, no_out_sink_names = no_out_sink(edges_keys, src, int, sink)
    A_no_in_out_interm, b_no_in_out_interm, no_in_out_interm_names = no_in_out_interm(edges_keys, src, int, sink)

    Aeq = np.vstack((A_cons, A_no_in_src, A_no_out_sink, A_no_in_out_interm))
    beq = np.vstack((b_cons, b_no_in_src, b_no_out_sink, b_no_in_out_interm))
    eq_cons_names = [*cons_names, *no_in_src_names, *no_out_sink_names, *no_in_out_interm_names]
    try:
        assert Aeq.shape[0] == len(eq_cons_names)
    except:
        pdb.set_trace()
    Aineq = np.vstack((A_cap, A_cut, A_flow))
    bineq = np.vstack((b_cap, b_cut, b_flow))
    ineq_cons_names = [*cap_names, *cut_names, *flow_names]
    assert Aineq.shape[0] == len(ineq_cons_names)
    return Aeq, beq, eq_cons_names, Aineq, bineq, ineq_cons_names

# Collecting projection constraints together as g(x,y) = A[x;y] - b>=0
def proj_constraints(edges_keys, nodes_keys, src, int, sink):
    A_cap, b_cap, cap_names = capacity_constraint(edges_keys, projection=True)
    A_cons, b_cons, cons_names = proj_conservation_constraint(nodes_keys, edges_keys, src, int, sink)
    # A_eq, b_eq, eq_names = eq_aux_constraint(edges_keys, projection=True)
    A_cut, b_cut, cut_names = cut_constraint(edges_keys, projection=True)
    A_flow, b_flow, flow_names = min_flow_constraint(edges_keys, src, int, sink, projection=True)
    A_no_in_src, b_no_in_src, no_in_src_names = no_in_source(edges_keys, src, int, sink, projection=True)
    A_no_out_sink, b_no_out_sink, no_out_sink_names = no_out_sink(edges_keys, src, int, sink, projection=True)
    # A = np.vstack((A_feas, A_cap))
    # b = np.vstack((b_feas, b_cap))
    Aeq = np.vstack((A_cons, A_no_in_src, A_no_out_sink))
    beq = np.vstack((b_cons, b_no_in_src, b_no_out_sink))
    eq_cons_names = [*cons_names, *no_in_src_names, *no_out_sink_names]

    Aineq = np.vstack((A_cap, A_cut, A_flow))
    bineq = np.vstack((b_cap, b_cut, b_flow))
    ineq_cons_names = [*cap_names, *cut_names, *flow_names]
    return Aeq, beq, eq_cons_names, Aineq, bineq, ineq_cons_names

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
    x0 = np.vstack((f1e_init*tfac, f2e_init*tfac, zero_cuts, tfac))
    return x0, f3e_init

def plot_matrix(M, fn):
    fn = os.getcwd() + "/" + fn
    plt.matshow(M)
    plt.show()
    plt.savefig(fn)

def solve_opt(maze, src, sink, int):
    x, y, G, nodes_keys, edges_keys = initialize(maze)
    x0, y0 = get_candidate_flows(G, edges_keys, src, int, sink)

    Aeq, beq, eq_cons_names, Aineq, bineq, ineq_cons_names = all_constraints(edges_keys, nodes_keys, src, int, sink)

    Aeq_proj, beq_proj, eq_cons_names_proj, Aineq_proj, bineq_proj, ineq_cons_names_proj = proj_constraints(edges_keys, nodes_keys, src, int, sink)

    c1, c2 = objective(edges_keys)
    ne = len(list(edges_keys.keys())) # number of edges

    T = 20
    eta = 0.01
    # pdb.set_trace()
    # Vin_oracle(edges_keys, nodes_keys, src, sink, int, x0) #x0 is the wrong size
    # xtraj, ytraj = max_oracle_gd(T, x0, eta, c1, c2, Aineq, bineq, Aproj, bproj, edges_keys, nodes_keys, src, sink, int, maze=maze)
    xtraj, ytraj = max_oracle_pyomo(T, x0, eta, c1, c2, Aeq, beq, Aineq, bineq, Aeq_proj, beq_proj, Aineq_proj, bineq_proj, eq_cons_names, ineq_cons_names, edges_keys, nodes_keys, src, sink, int, maze=maze)
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
