# First-order gradient algorithm that solves the convex-concave min-max optimization with dependent feasible sets

import numpy as np
import cvxpy as cp
import pdb

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
    constraints = [x >= np.zeros((n,1))]
    prob = cp.Problem(cp.Minimize(l2_norm), constraints)
    prob.solve()
    return x.value

# Definition of gradient function:
def gradient(xt, yt, lamt, c1, c2, Aineq, bineq):
    gradf = c1.copy()
    gradg = Aineq[:, :len(xt)].T # The columns of A corresponding to
    grad = gradf + lamt @ gradg
    return grad

# Gradient descent:
def max_oracle_gd(T, x0, eta, c1, c2, Aineq, bineq, edges_keys):
    xtraj = np.zeros((T,))
    ytraj = np.zeros((T,))
    inner_obj_traj = np.zeros((T-1,))
    xtraj[0] = x0.copy()
    for t in range(T-1):
        xt, yt, inner_obj_t, lam_t = Vin(c1, c2, Aineq, bineq, xtraj[t], edges_keys)
        ytraj[t] = yt.copy()
        inner_obj_traj[t] = inner_obj_t.copy()
        # Gradient step:
        pdb.set_trace()
        xtraj[t+1] = projx(xtraj[t] - eta*gradient(xtraj[t], ytraj[t], lamt, c1, c2, Aineq, bineq))
    xt, yt, inner_obj_t, lam_t = Vin(c1, c2, Aineq, bineq, xtraj[T-1], edges_keys)
    ytraj[T-1] = yt.copy()
    inner_obj_traj[T-1] = inner_obj_t.copy()
    return xtraj, ytraj

def Vin(c1, c2, Aineq, bineq, x0, edges_keys):
    ne = len(list(edges_keys.keys())) # number of edges
    x = cp.Variable((4*ne,1))
    y = cp.Variable((ne,1))
    # gamma = cp.Parameter(nonneg=True) # Regularization parameter
    inner_obj = c1.T @ x + 10*c2.T @ y
    zineq = 0*bineq # Zero vector
    constraints = [Aineq @ cp.vstack([x,y]) - bineq >= zineq]
    constraints.append(x == x0)
    inner_max = cp.Problem(cp.Minimize(inner_obj), constraints)
    inner_max.solve()
    # Print result.
    print("\nThe optimal value of Vin is", inner_max.value)
    print("A solution x is")
    print(x.value)
    print(y.value)
    print("A dual solution is")
    print(inner_max.constraints[0].dual_value)
    print("Is DPP? ", inner_max.is_dcp(dpp=True))
    print("Is DCP? ", inner_max.is_dcp(dpp=False))
    return x.value, y.value, inner_max.value, inner_max.constraints[0].dual_value

# Value function:
def Vout(c1, c2, Aineq, bineq, y0, edges_keys):
    # inner_max = cp.Problem(inner_obj)
    ne = len(list(edges_keys.keys())) # number of edges
    x = cp.Variable((4*ne,1))
    y = cp.Variable((ne,1))
    # gamma = cp.Parameter(nonneg=True) # Regularization parameter
    outer_obj = c1.T @ x + 10*c2.T @ y
    zineq = 0*bineq # Zero vector
    constraints = [Aineq @ cp.vstack([x,y]) - bineq >= zineq]
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
