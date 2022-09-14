# First-order gradient algorithm that solves the convex-concave min-max optimization with dependent feasible sets

import numpy as np
import cvxpy as cp
import pdb
import sys
sys.path.append('..')
from max_flow_oracle import max_flow_oracle, max_flow_oracle_fullg
from static_obstacle_maze.plotting import plot_mcf
pyomo_based = True
from matplotlib import pyplot as plt

# Matrix variables:
def init_vars(edges_keys):
    ne = len(list(edges_keys.keys())) # number of edges
    x = cp.Variable(3*ne+1)
    y = cp.Variable(ne)
    return x,y

# Definition of projection operator onto non-negative halfplane:
def projx(x0, ne, Aineq_proj, bineq_proj, Aeq_proj, beq_proj):
    n = len(x0)
    x = cp.Variable((n,1)) # Solves for -x --> ???
    l2_norm =  cp.sum_squares(x - x0)
    EPS_FLAG = 10**(-6)*np.ones((n,1))
    if Aeq_proj is not None:
        constraints = [Aineq_proj @ x >= bineq_proj, Aeq_proj @ x == beq_proj, x >=np.zeros((n,1))]
    else:
        constraints = [Aineq_proj @ x >= bineq_proj, x >=np.zeros((n,1))]
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

def projx_des(xs, x0, ne, Aineq_proj, bineq_proj, Aeq_proj, beq_proj):
    n = len(x0)
    t = cp.Variable() # Solves for -x
    x = t*x0 + (1-t)*xs # x0 is the original x and xs is the descent x.
    l2_norm =  cp.sum_squares(x - xs)
    EPS_FLAG = 10**(-6)*np.ones((n,1))
    constraints = [Aineq_proj @ x >= bineq_proj, Aeq_proj @ x == beq_proj, x >=np.zeros((n,1)), t >= 0, t <= 1]
    # constraints = [Aproj @ x >= bproj, x >= EPS_FLAG]
    prob = cp.Problem(cp.Minimize(l2_norm), constraints)
    prob.solve()
    # pdb.set_trace()
    if prob.status == "infeasible" or prob.status == "unbounded":
         pdb.set_trace()
    xproj = x.value.copy()
    for i in range(len(xproj)):
        if xproj[i][0] < 0:
            xproj[i][0] = 0.0
    pdb.set_trace()
    assert all(xproj >= np.zeros((n,1)))
    return xproj

# Definition of gradient function:
def gradient(xt, yt, lamt, c1, c2, Aineq, bineq, Aeq, beq):
    ne = len(xt)
    gradf = np.multiply(c1,np.ones((ne,1)))# Column vector
    gradg = np.vstack((Aineq[:, :len(xt)], Aeq[:, :len(xt)])) # The columns of A corresponding to
    grad = gradf + (gradg.T @ lamt)
    return grad

def run_diagnostics(Aineq_y, bineq_sub, xtraj, ytraj):
    # Check zero rows of A matrix:
    pass

def check_constraint(xt, yt, A, b):
    lhs = A @ np.vstack((xt,yt)) # A_x' x + A_y' y >= b
    ne = yt.shape[0]
    eps = 1e-2*np.ones((b.shape[0],1))
    Aineq_x = A[:, 0:4*ne]
    Aineq_y = A[:, 4*ne:]
    constraint = lhs >= b
    violations = [i for i, k in enumerate(constraint) if not k]
    violations_gap = [lhs[k,0]-b[k,0] for k in violations]

    # A[x; y+ygap] >= b. Need to solve for ygap
    # y = (A'.A)^-1 A'.b
    lhs_x = Aineq_x @ xt
    bnew = b - lhs_x
    ygap = np.linalg.inv(Aineq_y.T @ Aineq_y) @ (Aineq_y.T @ (bnew))
    # pdb.set_trace()
    new_lhs = A @ np.vstack((xt,yt + ygap))
    new_const = new_lhs >= b
    new_viol = [i for i, k in enumerate(new_const) if not k]
    new_viol_gap = [new_lhs[k,0]-b[k,0] for k in new_viol]
    # if violations != []:
    #     pdb.set_trace()


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

def match_dual_vars(lamt, eq_cons_names, ineq_cons_names):
    # Inequality constraints and then equality constraints:
    ineq_constraints = len(ineq_cons_names)
    eq_constraints = len(eq_cons_names)
    dual_vars = np.zeros((ineq_constraints + eq_constraints, 1))
    for i, c in enumerate(ineq_cons_names):
        dual_vars[i,0] = lamt[c]
    for i, c in enumerate(eq_cons_names):
        try:
            dual_vars[ineq_constraints+i, 0] = lamt[c]
        except:
            pdb.set_trace()
    return dual_vars

def match_dual_vars_v2(lamt, eq_cons_names, ineq_cons_names):
    # Inequality constraints and then equality constraints:
    ineq_constraints = len(ineq_cons_names)
    eq_constraints = len(eq_cons_names)
    dual_vars = np.zeros((ineq_constraints + eq_constraints, 1))
    for i, c in enumerate(ineq_cons_names):
        if c in lamt.keys():
            dual_vars[i,0] = lamt[c]
        else:
            dual_vars[i,0] = 0.0
    for i, c in enumerate(eq_cons_names):
        try:
            dual_vars[ineq_constraints+i, 0] = 0
        except:
            pdb.set_trace()
    return dual_vars

# Max oracle pyomo based:
# This is the Pyomo oracle that works!
def max_oracle_pyomo(T, x0, eta, c1, c2, Aeq, beq, Aineq, bineq, Aeq_proj, beq_proj, Aineq_proj, bineq_proj, eq_cons_names, ineq_cons_names, edges_keys,nodes_keys, src, sink, int, maze=None):
    LAMBDA = 1
    EPS_FLG = 1e-6
    grad_norm = []
    proj = "not_des"
    ne = len(list(edges_keys.keys()))
    xtraj = dict()
    xtraj[0] = x0
    ytraj = dict()
    converged = False
    for t in range(1,T-1):
        try:
            yt, lamt = Vin_oracle(edges_keys, nodes_keys, src, sink, int, xtraj[t-1], LAMBDA)
        except:
            pdb.set_trace()
        dual_vars = match_dual_vars(lamt, eq_cons_names, ineq_cons_names)
        ytraj[t-1] = yt.copy()
        xstep = xtraj[t-1] - (0.1)*gradient(xtraj[t-1], ytraj[t-1], dual_vars, c1, c2, Aineq, bineq, Aeq, beq)
        gt = np.linalg.norm(gradient(xtraj[t-1], ytraj[t-1], dual_vars, c1, c2, Aineq, bineq, Aeq, beq))
        grad_norm.append(gt)
        if proj == "des":
            xtraj[t] = projx_des(xstep, xtraj[t-1], ne, Aineq_proj, bineq_proj, Aeq_proj, beq_proj)
        else:
            xtraj[t] = projx(xstep, ne, Aineq_proj, bineq_proj, Aeq_proj, beq_proj)
        converged = np.linalg.norm(xtraj[t]-xtraj[t-1]) < EPS_FLG
        if t in [20, 100, 250, 500]:
            pdb.set_trace()

        # if converged:
        #     yt, lam_t = Vin_oracle(edges_keys, nodes_keys, src, sink, int, xtraj[t],LAMBDA)
        #     ytraj[t] = yt.copy()
        #     print("Converged!")
        #     break
    yt, lam_t = Vin_oracle(edges_keys, nodes_keys, src, sink, int, xtraj[t],LAMBDA)
    ytraj[t] = yt.copy()
    # if not converged:
    dual_vars = match_dual_vars(lamt, eq_cons_names, ineq_cons_names)
    xstep = xtraj[T-2] - eta*gradient(xtraj[T-2], ytraj[T-2], dual_vars, c1, c2, Aineq, bineq, Aeq, beq)
    xtraj[T-1] = projx(xstep, ne, Aineq_proj, bineq_proj, Aeq_proj, beq_proj)
    print("Found a suitable lambda!")
    return xtraj, ytraj

# Max oracle pyomo based:
# This is the Pyomo oracle that works!
def max_oracle_pyomo_v2(T, x0, eta, c1, c2, Aeq, beq, Aineq, bineq, Aeq_proj, beq_proj, Aineq_proj, bineq_proj, eq_cons_names, ineq_cons_names, edges_keys,nodes_keys, src, sink, int, maze=None):
    LAMBDA = 1e4
    EPS_FLG = 1e-6
    fval = []
    proj = "not_des"
    ne = len(list(edges_keys.keys()))
    xtraj = dict()
    xtraj[0] = x0
    ytraj = dict()
    converged = False
    for t in range(1,T-1):
        yt, lamt, fv = Vin_oracle(edges_keys, nodes_keys, src, sink, int, xtraj[t-1], LAMBDA)
        dual_vars = match_dual_vars_v2(lamt, eq_cons_names, ineq_cons_names)
        ytraj[t-1] = yt.copy()
        xstep = xtraj[t-1] - 0.1/t*gradient(xtraj[t-1], ytraj[t-1], dual_vars, c1, c2, Aineq, bineq, Aeq, beq)
        # xstep = xtraj[t-1] - 0.1/t*np.random.random()*np.ones(xtraj[t-1].shape)
        fval.append(fv)
        if proj == "des":
            xtraj[t] = projx_des(xstep, xtraj[t-1], ne, Aineq_proj, bineq_proj, Aeq_proj, beq_proj)
        else:
            xtraj[t] = projx(xstep, ne, Aineq_proj, bineq_proj, Aeq_proj, beq_proj)
        converged = np.linalg.norm(xtraj[t]-xtraj[t-1]) < EPS_FLG
        if t in [500, 1000]:
            pdb.set_trace()
            break

        # if converged:
        #     yt, lam_t = Vin_oracle(edges_keys, nodes_keys, src, sink, int, xtraj[t],LAMBDA)
        #     ytraj[t] = yt.copy()
        #     print("Converged!")
        #     break
    yt, lam_t, fv = Vin_oracle(edges_keys, nodes_keys, src, sink, int, xtraj[t],LAMBDA)
    ytraj[t] = yt.copy()
    # if not converged:
    # dual_vars = match_dual_vars_v2(lamt, eq_cons_names, ineq_cons_names)
    # xstep = xtraj[T-2] - eta*gradient(xtraj[T-2], ytraj[T-2], dual_vars, c1, c2, Aineq, bineq, Aeq, beq)
    # xstep = xtraj[T-2] - eta*np.random.random()*np.ones(xtraj[T-2].shape)
    # xtraj[T-1] = projx(xstep, ne, Aineq_proj, bineq_proj, Aeq_proj, beq_proj)
    print("Found a suitable lambda!")
    return xtraj, ytraj

# Gradient descent:
def max_oracle_gd(T, x0, eta, c1, c2, Aineq, bineq, Aproj, bproj, edges_keys,nodes_keys, src, sink, int, maze=None):
    # for LAMBDA in np.logspace(,6,20):
    LAMBDA = 1e4
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
        if plot:
            plot_flows(xtraj[t-1], ytraj[t-2], edges_keys, maze)

        if not pyomo_based:
            yt, inner_obj_t, lamt, status = Vin(c1, c2, Aineq, bineq, xtraj[t-1], edges_keys,LAMBDA)
            pdb.set_trace()
        else:
            yt, lamt = Vin_oracle(edges_keys, nodes_keys, src, sink, int, xtraj[t-1],LAMBDA)

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
        xstep = xtraj[t-1] - (1.0/t)*gradient(xtraj[t-1], ytraj[t-1], lamt, c1, c2, Aineq, bineq, Aeq, beq)
        xtraj[t] = projx(xstep, ne, Aineq_proj, bineq_proj, Aeq_proj, beq_proj)
        check_constraint(xtraj[t], yt, Aineq, bineq)

    if not pyomo_based:
        yt, inner_obj_t, lamt, status = Vin(c1, c2, Aineq, bineq, xtraj[t-1], edges_keys,LAMBDA)
        pdb.set_trace()
    else:
        yt, lamt = Vin_oracle(edges_keys, nodes_keys, src, sink, int, xtraj[t-1],LAMBDA)
    pdb.set_trace()
    # if status == "infeasible" or status == "unbounded":
    #     continue
    ytraj[T-1] = yt.copy()
    inner_obj_traj[T-1] = inner_obj_t
    xtraj[T-1] = projx(xtraj[T-2] - eta*gradient(xtraj[T-2], ytraj[T-2], lamt, c1, c2, Aineq, bineq))
    print("Found a suitable lambda!")
    pdb.set_trace()
    return xtraj, ytraj

def plot_grad_norm(gt):
    fig, ax = plt.subplots()
    time_stps = list(range(len(gt)))
    plt.plot(time_stps, gt, 'b-*')
    plt.show()

def Vin_oracle(edges_keys, nodes_keys, src, sink, int, x,LAMBDA):
    yt, lamt, fv = max_flow_oracle(edges_keys, nodes_keys, src, sink, int, x, LAMBDA)
    # yt, lamt = max_flow_oracle_fullg(edges_keys, nodes_keys, src, sink, int, x,LAMBDA)
    return yt, lamt, fv

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
