# First-order gradient algorithm that solves the convex-concave min-max optimization with dependent feasible sets

import numpy as np
import cvxpy as cp
import pdb
import sys
sys.path.append('..')
from max_flow_oracle import max_flow_oracle

# Matrix variables:
def init_vars(edges_keys):
    ne = len(list(edges_keys.keys())) # number of edges
    x = cp.Variable(4*ne)
    y = cp.Variable(ne)
    return x,y

# Definition of projection operator onto non-negative halfplane:
def projx(x0):
    n = len(x0)
    x = cp.Variable((n,1))
    l2_norm =  cp.sum_squares(x - x0)
    EPS_FLAG = 1e-6*np.ones((n,1))
    constraints = [x >= np.zeros((n,1)) + EPS_FLAG]
    prob = cp.Problem(cp.Minimize(l2_norm), constraints)
    prob.solve()
    return x.value

# Definition of gradient function:
def gradient(xt, yt, lamt, c1, c2, Aineq, bineq):
    ne = len(xt)
    gradf = np.multiply(c1,np.ones((ne,1)))# Column vector
    gradg = Aineq[:, :len(xt)] # The columns of A corresponding to
    grad = gradf + (gradg.T @ lamt)
    return grad

# Gradient descent:
def max_oracle_gd(T, x0, eta, c1, c2, Aineq, bineq, edges_keys):
    for LAMBDA in np.logspace(-6,6,20):
        xtraj[0] = x0
        xtraj = {i:None for i in range(T)}
        ytraj = {i:None for i in range(T)}
        inner_obj_traj = {i:None for i in range(T-1)}
        for t in range(1,T-1):
            xt, yt, inner_obj_t, lamt, status = Vin(c1, c2, Aineq, bineq, xtraj[t-1], edges_keys,LAMBDA)
            if status == "infeasible" or status == "unbounded":
                break
            ytraj[t-1] = yt.copy()
            inner_obj_traj[t-1] = inner_obj_t
            xtraj[t] = projx(xtraj[t-1] - eta*gradient(xtraj[t-1], ytraj[t-1], lamt, c1, c2, Aineq, bineq))
        xt, yt, inner_obj_t, lam_t, status = Vin(c1, c2, Aineq, bineq, xtraj[T-1], edges_keys, LAMBDA)
        if status == "infeasible" or status == "unbounded":
            continue
        ytraj[T-1] = yt.copy()
        inner_obj_traj[T-1] = inner_obj_t
        xtraj[T-1] = projx(xtraj[T-2] - eta*gradient(xtraj[T-2], ytraj[T-2], lamt, c1, c2, Aineq, bineq))
        print("Found a suitable lambda!")
        pdb.set_trace()
    return xtraj, ytraj

def Vin_oracle(edges_keys, nodes_keys, src, sink, int, x):
    yt = max_flow_oracle(edges_keys, nodes_keys, src, sink, int, x)
    pdb.set_trace()

def Vin(c1, c2, Aineq, bineq, x0, edges_keys, LAMBDA):
    ne = len(list(edges_keys.keys())) # number of edges
    x = cp.Variable((4*ne,1))
    y = cp.Variable((ne,1))
    # gamma = cp.Parameter(nonneg=True) # Regularization parameter
    inner_obj = c1.T @ x + LAMBDA*c2.T @ y
    zineq = 0*bineq # Zero vector
    constraints = [Aineq @ cp.vstack([x,y])  >= bineq]
    # pdb.set_trace()
    constraints.append(x == x0)
    inner_max = cp.Problem(cp.Maximize(inner_obj), constraints)
    inner_max.solve()
    # Print result.
    print("status:", inner_max.status)
    # if inner_max.status == "infeasible" or inner_max.status == "unbounded":
    #     pdb.set_trace()
    print("\nThe optimal value of Vin is", inner_max.value)
    print("A solution x is")
    print(x.value)
    print(y.value)
    print("A dual solution is")
    print(inner_max.constraints[0].dual_value)
    print("Is DPP? ", inner_max.is_dcp(dpp=True))
    print("Is DCP? ", inner_max.is_dcp(dpp=False))
    return x.value, y.value, inner_max.value, inner_max.constraints[0].dual_value, inner_max.status

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
