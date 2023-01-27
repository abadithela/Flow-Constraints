# This script implements the first-order gradient descent algorithm for CMS 248
# Apurva Badithela

import sys
sys.path.append('..')
import numpy as np
from ipdb import set_trace as st
from collections import OrderedDict as od
import _pickle as pickle
import os
import networkx as nx
from road_network.components.road_network import RoadNetwork, create_network_from_file
from road_network.helpers.plotting import plot_mcf as roadnet_plot_mcf
from static_obstacle_maze.plotting import plot_mcf as maze_plot_mcf
from static_obstacle_maze.network import MazeNetwork
import matplotlib.pyplot as plt
from scipy import sparse as sp
from gradient_descent_optimizations import Vout, Vin, Vin_oracle, max_oracle_gd, max_oracle_pyomo, projx, gradient, max_oracle_pyomo_dep
import pdb
from gradient_descent_problem_data import *

# Initialize:
debug = True
ADD_EDGE_CUTS_INIT = False

def solve_opt_pyomo_v2(maze, src, sink, int):
    '''
    Solving optimization using gradient descent using Pyomo as the oracle for the inner maximization.
    '''
    x, y, G, nodes_keys, edges_keys = initialize(maze)
    x0, y0 = get_candidate_flows(G, edges_keys, src, int, sink)

    Aeq, beq, eq_cons_names, Aineq, bineq, ineq_cons_names = all_constraints(edges_keys, nodes_keys, src, int, sink)

    Aeq_proj, beq_proj, eq_cons_names_proj, Aineq_proj, bineq_proj, ineq_cons_names_proj = proj_constraints(edges_keys, nodes_keys, src, int, sink)

    # Aineq_proj, bineq_proj, ineq_cons_names_proj = proj_constraints_box_only(edges_keys, nodes_keys, src, int, sink)
    # Aeq_proj = None
    # beq_proj = None

    c1, c2 = objective(edges_keys)
    ne = len(list(edges_keys.keys())) # number of edges

    T = 5000
    eta = 0.01
    # pdb.set_trace()
    # Vin_oracle(edges_keys, nodes_keys, src, sink, int, x0) #x0 is the wrong size
    # xtraj, ytraj = max_oracle_gd(T, x0, eta, c1, c2, Aineq, bineq, Aproj, bproj, edges_keys, nodes_keys, src, sink, int, maze=maze)
    xtraj, ytraj = max_oracle_pyomo_v2(T, x0, eta, c1, c2, Aeq, beq, Aineq, bineq, Aeq_proj, beq_proj, Aineq_proj, bineq_proj, eq_cons_names, ineq_cons_names, edges_keys, nodes_keys, src, sink, int, maze=maze)
    # Vin(c1, c2, A, b, x0, edges_keys)

    f1_e_hist, f2_e_hist, f3_e_hist, d_e_hist, F_hist = parse_solution(xtraj[-1], ytraj[-1], G, edges_keys, nodes_keys)
    return f1_e_hist, f2_e_hist, f3_e_hist, d_e_hist, F_hist

def add_edge_debug(G, x0, edges_keys, init_edge_cuts):
    '''
    For debugging purposes to see if the optimization converges to a different Stackleberg equilibria, we tweak the initial x0 to have some edge cuts. This is to see if it would converge to a different Stacklerberg equilibria. Playing with the toy examples showed that the algorithm is very sensitive to the initial condition.
    '''
    init_edge_cuts_keys = []
    ne = len(list(edges_keys.keys()))
    for k, e in edges_keys.items():
        if e in init_edge_cuts and k not in init_edge_cuts_keys:
            init_edge_cuts_keys.append(k)

    x0 = adjust_flows_for_cuts(G, init_edge_cuts, src, int, sink, edges_keys)
    # pdb.set_trace()
    for k in init_edge_cuts_keys:
        idx = 2*ne + k
        x0[idx][0] = 0.7

    return x0

def solve_opt_pyomo(maze, src, sink, int, reg):
    '''
    Solving optimization using gradient descent using Pyomo as the oracle for the inner maximization.
    '''
    x, y, G, nodes_keys, edges_keys = initialize(maze)
    x0, y0 = get_candidate_flows(G, edges_keys, src, int, sink)
    init_edge_cuts = [((0,0),(0,1))]
    if ADD_EDGE_CUTS_INIT:
        x0 = add_edge_debug(G, x0, edges_keys, init_edge_cuts)

    Aeq, beq, eq_cons_names, Aineq, bineq, ineq_cons_names = all_constraints(edges_keys, nodes_keys, src, int, sink)

    Aeq_proj, beq_proj, eq_cons_names_proj, Aineq_proj, bineq_proj, ineq_cons_names_proj = proj_constraints(edges_keys, nodes_keys, src, int, sink)

    # Aineq_proj, bineq_proj, ineq_cons_names_proj = proj_constraints_box_only(edges_keys, nodes_keys, src, int, sink)
    # Aeq_proj = None
    # beq_proj = None

    c1, c2 = objective(edges_keys)
    ne = len(list(edges_keys.keys())) # number of edges

    T = 2000
    eta = 0.1
    # pdb.set_trace()
    # Vin_oracle(edges_keys, nodes_keys, src, sink, int, x0) #x0 is the wrong size
    # xtraj, ytraj = max_oracle_gd(T, x0, eta, c1, c2, Aineq, bineq, Aproj, bproj, edges_keys, nodes_keys, src, sink, int, maze=maze)
    xtraj, ytraj = max_oracle_pyomo_dep(T, x0, eta, c1, c2, Aeq, beq, Aineq, bineq, Aeq_proj, beq_proj, Aineq_proj, bineq_proj, eq_cons_names, ineq_cons_names, edges_keys, nodes_keys, src, sink, int, LAMBDA=reg, maze=maze)
    # Vin(c1, c2, A, b, x0, edges_keys)
    f1_e_hist, f2_e_hist, f3_e_hist, d_e_hist, F_hist = [],[],[],[],[]
    try:
        f1_e_hist, f2_e_hist, f3_e_hist, d_e_hist, F_hist = parse_solution(xtraj[-1], ytraj[-1], G, edges_keys, nodes_keys)
    except:
        print("Infeasible solution")
    return f1_e_hist, f2_e_hist, f3_e_hist, d_e_hist, F_hist

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
    elif grid == "med":
        mazefile = par_dir + '/constrained_mcf/med_mazefile.txt'
        src = (0,0)
        sink = (2,2)
        int = (1,2)
        maze = MazeNetwork(mazefile)
        #pdb.set_trace()

    regularizers = np.linspace(10, 10**5, 10)
    for reg in regularizers:
        f1_e_hist, f2_e_hist, f3_e_hist, d_e_hist, F_hist = solve_opt_pyomo(maze, src, sink, int, reg)

    if f1_e_hist!= []:
        if grid == "toy" or "med":
            maze_plot_mcf(maze, f1_e_hist[-1], f2_e_hist[-1], f3_e_hist[-1], d_e_hist[-1])
            # for t in range(len(F_hist)):
            #     print("Max flow: " + str(F_hist[t]))
            #     maze_plot_mcf(maze, f1_e_hist[t], f2_e_hist[t], f3_e_hist[t], d_e_hist[t])

        elif grid == "small" or "large":
            for t in range(len(F_hist)):
                print("Max flow: " + str(F_hist[t]))
                roadnet_plot_mcf(maze, f1_e_hist[t], f2_e_hist[t], f3_e_hist[t], d_e_hist[t])
