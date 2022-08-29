# First-order gradient algorithm that solves the convex-concave min-max optimization with dependent feasible sets

import numpy as np
import cvxpy as cp
import pdb
import sys
sys.path.append('..')
from max_flow_oracle import max_flow_oracle
from static_obstacle_maze.plotting import plot_mcf
pyomo_based = False

# Matrix variables:
def init_vars(edges_keys):
    ne = len(list(edges_keys.keys())) # number of edges
    x = cp.Variable(4*ne)
    y = cp.Variable(ne)
    return x,y

# Definition of projection operator onto non-negative halfplane:
def projx(x0, ne, Aproj, bproj):
    n = len(x0)
    x = cp.Variable((n,1)) # Solves for -x
    l2_norm =  cp.sum_squares(x - x0)
    EPS_FLAG = 10**(-6)*np.ones((n,1))
    constraints = [Aproj @ x >= bproj]
    # constraints = [Aproj @ x >= bproj, x >= EPS_FLAG]
    prob = cp.Problem(cp.Minimize(l2_norm), constraints)
    prob.solve()
    # pdb.set_trace()
    if prob.status == "infeasible" or prob.status == "unbounded":
         pdb.set_trace()
    xproj = x.value.copy()
    # if any(xproj < 0):
        # pdb.set_trace()
    for i in range(len(xproj)):
        if xproj[i][0] < 0:
            xproj[i][0] = 0.0
    # pdb.set_trace()
    assert all(xproj >= np.zeros((n,1)))
    return xproj

# Definition of gradient function:
def gradient(xt, yt, lamt, c1, c2, Aineq, bineq):
    ne = len(xt)
    gradf = np.multiply(c1,np.ones((ne,1)))# Column vector
    gradg = Aineq[:, :len(xt)] # The columns of A corresponding to
    grad = gradf + (gradg.T @ lamt)
    return grad

def run_diagnostics(Aineq_y, bineq_sub, xtraj, ytraj):
    # Check zero rows of A matrix:
    pass

def check_constraint(xt, yt, A, b):
    lhs = A @ np.vstack((xt,yt))
    ne = yt.shape[0]
    eps = 1e-2*np.ones((b.shape[0],1))
    Aineq_x = A[:, 0:4*ne]
    Aineq_y = A[:, 4*ne:]
    constraint = lhs >= b
    violations = [i for i, k in enumerate(constraint) if not k]
    violations_gap = [lhs[k,0]-b[k,0] for k in violations]
    # A[x; y+ygap] >= b. Need to solve for ygap
    ygap = np.linalg.inv(Aineq_y.T @ Aineq_y) @ (Aineq_y.T @ (b+eps-lhs))
    new_lhs = A @ np.vstack((xt,yt + ygap))
    new_const = new_lhs >= b
    new_viol = [i for i, k in enumerate(new_const) if not k]
    new_viol_gap = [new_lhs[k,0]-b[k,0] for k in new_viol]
    if violations != []:
        pdb.set_trace()
        
def max_oracle_gd(T, x0, eta, c1, c2, Aineq, bineq, Aproj, bproj, edges_keys, maze=None):
    # for LAMBDA in np.logspace(,6,20):
    LAMBDA = 1
    ne = len(list(edges_keys.keys()))
    Aineq_x = Aineq[:, 0:4*ne]
    Aineq_y = Aineq[:, 4*ne:]
    plot = False
    xtraj = {i:None for i in range(T)}
    xtraj[0] = x0
    ytraj = {i:None for i in range(T)}
    inner_obj_traj = {i:None for i in range(T-1)}
    for t in range(1,T-1):
        # Plotting to help debug:
        bineq_sub = bineq - Aineq_x.dot(xtraj[t-1])
        # pdb.set_trace()
        if plot and t>1:
            plot_flows(xtraj[t-1], ytraj[t-2], edges_keys, maze)
        if not pyomo_based:
            yt, inner_obj_t, lamt, status = Vin(c1, c2, Aineq, bineq, xtraj[t-1], edges_keys,LAMBDA)
        else:
            yt, inner_obj_t, lamt, status = Vin_oracle()
        if status == "infeasible" or status == "unbounded":
            run_diagnostics(Aineq_y, bineq_sub, xtraj, ytraj)
            pdb.set_trace()
            break
        else:
            if plot and maze is not None:
                plot_flows(xtraj[t-1], yt, edges_keys, maze)
            # Run diagnostics:
            check_constraint(xtraj[t-1], yt, Aineq, bineq)
            # pdb.set_trace()

        ytraj[t-1] = yt.copy()
        inner_obj_traj[t-1] = inner_obj_t
        xstep = xtraj[t-1] - (1.0/t)*gradient(xtraj[t-1], ytraj[t-1], lamt, c1, c2, Aineq, bineq)
        xtraj[t] = projx(xstep, ne, Aproj, bproj)
        check_constraint(xtraj[t], yt, Aineq, bineq)

    yt, inner_obj_t, lam_t, status = Vin(c1, c2, Aineq, bineq, xtraj[T-1], edges_keys, LAMBDA)
    pdb.set_trace()
    # if status == "infeasible" or status == "unbounded":
    #     continue
    ytraj[T-1] = yt.copy()
    inner_obj_traj[T-1] = inner_obj_t
    xtraj[T-1] = projx(xtraj[T-2] - eta*gradient(xtraj[T-2], ytraj[T-2], lamt, c1, c2, Aineq, bineq))
    print("Found a suitable lambda!")
    pdb.set_trace()
    return xtraj, ytraj

def plot_flows(x,y, edges_keys, maze):
    ne = len(edges_keys.keys())
    t = x[3*ne,0]
    print("1/F is: ", str(t))
    assert t>0
    flow1 = dict()
    flow2 = dict()
    flow3 = dict()
    cuts = dict()

    for k, edge in edges_keys.items():
        idx1 = k
        idx2 = ne + k
        idx_cuts = 2*ne + k
        flow1.update({(edge):x[idx1,0]/t})
        flow2.update({(edge):x[idx2,0]/t})
        flow3.update({(edge):y[idx1,0]/t})
        cuts.update({(edge):x[idx_cuts,0]/t})
    plot_mcf(maze, flow1, flow2, flow3, cuts)
# Gradient descent:
def max_oracle_gd(T, x0, eta, c1, c2, Aineq, bineq, Aproj, bproj, edges_keys, maze=None):
    # for LAMBDA in np.logspace(,6,20):
    LAMBDA = 1
    ne = len(list(edges_keys.keys()))
    Aineq_x = Aineq[:, 0:4*ne]
    Aineq_y = Aineq[:, 4*ne:]
    plot = False
    xtraj = {i:None for i in range(T)}
    xtraj[0] = x0
    ytraj = {i:None for i in range(T)}
    inner_obj_traj = {i:None for i in range(T-1)}
    for t in range(1,T-1):
        # Plotting to help debug:
        bineq_sub = bineq - Aineq_x.dot(xtraj[t-1])
        # pdb.set_trace()
        if plot and t>1:
            plot_flows(xtraj[t-1], ytraj[t-2], edges_keys, maze)
        if not pyomo_based:
            yt, inner_obj_t, lamt, status = Vin(c1, c2, Aineq, bineq, xtraj[t-1], edges_keys,LAMBDA)
        else:
            yt, inner_obj_t, lamt, status = Vin_oracle()
        if status == "infeasible" or status == "unbounded":
            run_diagnostics(Aineq_y, bineq_sub, xtraj, ytraj)
            pdb.set_trace()
            break
        else:
            if plot and maze is not None:
                plot_flows(xtraj[t-1], yt, edges_keys, maze)
            # Run diagnostics:
            check_constraint(xtraj[t-1], yt, Aineq, bineq)
            # pdb.set_trace()

        ytraj[t-1] = yt.copy()
        inner_obj_traj[t-1] = inner_obj_t
        xstep = xtraj[t-1] - (1.0/t)*gradient(xtraj[t-1], ytraj[t-1], lamt, c1, c2, Aineq, bineq)
        xtraj[t] = projx(xstep, ne, Aproj, bproj)
        check_constraint(xtraj[t], yt, Aineq, bineq)

    yt, inner_obj_t, lam_t, status = Vin(c1, c2, Aineq, bineq, xtraj[T-1], edges_keys, LAMBDA)
    pdb.set_trace()
    # if status == "infeasible" or status == "unbounded":
    #     continue
    ytraj[T-1] = yt.copy()
    inner_obj_traj[T-1] = inner_obj_t
    xtraj[T-1] = projx(xtraj[T-2] - eta*gradient(xtraj[T-2], ytraj[T-2], lamt, c1, c2, Aineq, bineq))
    print("Found a suitable lambda!")
    pdb.set_trace()
    return xtraj, ytraj

def dual_variable():
    pass

def Vin_oracle(edges_keys, nodes_keys, src, sink, int, x):
    yt, inner_obj_t, status = max_flow_oracle(edges_keys, nodes_keys, src, sink, int, x)
    lamt = dual_variable()
    return x, yt, inner_obj_t, lamt, status

def pseudo_inv(Aineq_y):
    pseudo_inv = Aineq_y.T @ Aineq_y
    if not np.isfinite(np.linalg.cond(pseudo_inv)):
        pdb.set_trace()
    return pseudo_inv

def Vin(c1, c2, Aineq, bineq, x0, edges_keys, LAMBDA):
    ne = len(list(edges_keys.keys())) # number of edges
    # x = cp.Variable((4*ne,1))
    y = cp.Variable((ne,1))
    # gamma = cp.Parameter(nonneg=True) # Regularization parameter
    # inner_obj = c1.T @ x0 + LAMBDA*c2.T @ y
    inner_obj = c2.T @ y
    # pdb.set_trace()
    Aineq_x = Aineq[:, 0:4*ne]
    Aineq_y = Aineq[:, 4*ne:]
    bineq_sub = bineq - Aineq_x.dot(x0)
    constraints = [Aineq_y @ y >= bineq_sub]
    # constraints.append(x == x0)
    inner_max = cp.Problem(cp.Maximize(inner_obj), constraints)
    inner_max.solve(verbose=True)
    # Print result.
    print("status:", inner_max.status)
    # if inner_max.status == "infeasible" or inner_max.status == "unbounded":
    #     pdb.set_trace()
    # print("\nThe optimal value of Vin is", inner_max.value)
    # print("A solution x is")
    # print(x.value)
    # print(y.value)
    # print("A dual solution is")
    # print(inner_max.constraints[0].dual_value)
    # print("Is DPP? ", inner_max.is_dcp(dpp=True))
    # print("Is DCP? ", inner_max.is_dcp(dpp=False))
    return y.value, inner_max.value, inner_max.constraints[0].dual_value, inner_max.status

# TODO: use max flow oracle to do this.
# Challenge: convert between max flow pyomo and matrix forms'

# Value function:
def Vout(c1, c2, Aineq, bineq, y0, edges_keys):
    # inner_max = cp.Problem(inner_obj)
    ne = len(list(edges_keys.keys())) # number of edges
    x = cp.Variable((4*ne,1))
    y = cp.Variable((ne,1))
    # gamma = cp.Parameter(nonneg=True) # Regularization parameter
    outer_obj = c1.T @ x + 10*c2.T @ y
    zineq = 0*bineq # Zero vector
    constraints = [Aineq @ cp.vstack([x,y]) >= bineq]
    constraints.append(y == y0)
    outer_min = cp.Problem(cp.Minimize(outer_obj), constraints)
    outer_min.solve()
    # Print result.
    print("\nThe optimal value of Vout is", outer_min.value)
    print("A solution x is")
    print(x.value)
    print(y.value)
    print("A dual solution is")
    print(outer_min.constraints[0].dual_value)
    print("Is DPP? ", outer_min.is_dcp(dpp=True))
    print("Is DCP? ", outer_min.is_dcp(dpp=False))
    return x.value, y.value, outer_min.value, outer_min.constraints[0].dual_value
