# This file studies the convergence and importance of Lagrange multipliers in the usefulness of max oracle gradient descent algorithm for a toy min0max stackleberg game with dependent feasible sets.
import numpy as np
import cvxpy as cp
import pdb
import sys
sys.path.append('..')
from max_flow_oracle import max_flow_oracle, max_flow_oracle_fullg, max_flow_oracle_debug
from static_obstacle_maze.plotting import plot_mcf
pyomo_based = True
from matplotlib import pyplot as plt

# Matrix variables:
def init_vars():
    x = cp.Variable()
    y = cp.Variable()
    return x,y

# Definition of gradient function:
def gradient_orig(xt, yt, lamt):
    gradf = 2*xt # Column vector
    gradg = -1 # Gradient with respect to x
    grad = gradf + (gradg * lamt[0])
    return grad

# Definition of gradient function:
def gradient_mod(xt, yt, lamt):
    gradf = 2*xt # Column vector
    gradg = np.array([[-1.0],[1.0]]) # Gradient with respect to x with x's constraint
    grad = gradf + (gradg.T @ lamt)
    return grad

# Plot grad norm
def plot_grad_norm(gt):
    fig, ax = plt.subplots()
    time_stps = list(range(len(gt)))
    plt.plot(time_stps, gt, 'b-*')
    plt.show()

# Projection between [-1,1]
def projx(x0, lb, ub):
    x = cp.Variable() # Solves for -x --> ???
    l2_norm =  cp.sum_squares(x - x0)
    constraints = [x <= ub, x >=lb]

    prob = cp.Problem(cp.Minimize(l2_norm), constraints)
    prob.solve()
    # pdb.set_trace()
    if prob.status == "infeasible" or prob.status == "unbounded":
         pdb.set_trace()
    else:
         return x.value

# Projection between [-1,1] + plus extra constraints on outer player that do not depend on inner player.
def projx_mod(x0, lb, ub):
    x = cp.Variable() # Solves for -x --> ???
    l2_norm =  cp.sum_squares(x - x0)
    constraints = [x <= ub, x >=lb, x <= -0.1] # The last one is a constraint that is independent of the inner player

    prob = cp.Problem(cp.Minimize(l2_norm), constraints)
    prob.solve()
    # pdb.set_trace()
    if prob.status == "infeasible" or prob.status == "unbounded":
         pdb.set_trace()
    else:
         return x.value

# Vin_oracle
def Vin_oracle_orig(xval):
    xp = xval
    yp = cp.Variable()
    prob = cp.Problem(cp.Maximize(xval**2+1+yp), [xval + yp <= 1, yp <= 1, yp >= -1])
    prob.solve()
    yt = yp.value
    lamt = [prob.constraints[0].dual_value]
    fv = prob.value
    return yt, lamt, fv

def get_x_lag(yt, lamt):
    xp = cp.Variable()
    prob = cp.Problem(cp.Minimize(xp**2 + 1 + yt), [xp + yt <= 1,  xp <= -0.1, xp <= 1, xp >= -1])
    prob.solve(verbose=False)
    xt = xp.value
    lamt.append(prob.constraints[1].dual_value)
    return lamt

# Writing this modified function is not easy
def Vin_oracle_mod(xval):
    yp = cp.Variable()
    prob = cp.Problem(cp.Maximize(xval**2 + 1 + yp), [xval + yp <= 1, yp <= 1, yp >= -1])
    prob.solve()
    yt = yp.value
    lamt = [prob.constraints[0].dual_value]
    fv = prob.value
    lamt = get_x_lag(yt, lamt)
    return yt, lamt, fv

def max_oracle_gd_proj_mod(T, x0):
    '''
    First-order gradient descent optimization withthe oracle for max flow. The gradient descent part is done with Lagrange multipliers from the normal Vin oracle and the projection operator is modified to reflect constraints decoupled from the inner player.
    '''
    fval = []
    lb = -1
    ub = 1
    xtraj = dict()
    xtraj[0] = x0
    ytraj = dict()
    lag = []
    converged = False
    for t in range(1,T-1):
        print("----------------------------------")
        print("Iteration: "+str(t))
        yt, lamt, fv = Vin_oracle_orig(xtraj[t-1])
        print("Solve oracle problem for inner max")
        ytraj[t-1] = yt.copy()
        xstep = xtraj[t-1] - 0.25/t*gradient_orig(xtraj[t-1], ytraj[t-1], lamt)
        fval.append(fv)
        lag.append(lamt)
        xtraj[t] = projx_mod(xstep, lb, ub)
        pdb.set_trace()
        if xtraj[t] is not None:
            print("Solved projection")
            print("\n")
        else:
            pdb.set_trace()

    yt, lamt, fv = Vin_oracle_orig(xtraj[T-2])
    ytraj[T-2] = yt.copy()
    xstep = xtraj[T-2] - 0.25/t*gradient_orig(xtraj[T-2], ytraj[T-2], lamt)
    xtraj[T-1] = projx_mod(xstep, lb, ub)
    fval.append(fv)
    lag.append(lamt)
    return xtraj, ytraj, fval, lag

def max_oracle_gd_mod(T, x0):
    '''
    First-order gradient descent optimization withthe oracle for max flow. Solve inner problem to find the Lagrangian, and to find Lagrangian of constraints corresponding to outer problem, the inner value is fixed and the outer problem is solved, and only the Lagrangian is taken from this.
    '''
    fval = []
    lb = -1
    ub = 1
    xtraj = dict()
    xtraj[0] = x0
    ytraj = dict()
    lag = []
    converged = False
    for t in range(1,T-1):
        print("----------------------------------")
        print("Iteration: "+str(t))
        yt, lamt, fv = Vin_oracle_mod(xtraj[t-1])
        print("Solve oracle problem for inner max")
        ytraj[t-1] = yt.copy()
        xstep = xtraj[t-1] - 0.25/t*gradient_mod(xtraj[t-1], ytraj[t-1], lamt)
        fval.append(fv)
        lag.append(lamt)
        xtraj[t] = projx(xstep, lb, ub)
        if xtraj[t] is not None:
            print("Solved projection")
            print("\n")
        else:
            pdb.set_trace()
    yt, lamt, fv = Vin_oracle_mod(xtraj[T-2])
    ytraj[T-2] = yt.copy()
    xstep = xtraj[T-2] - 0.25/t*gradient_mod(xtraj[T-2], ytraj[T-2], lamt)
    xtraj[T-1] = projx(xstep, lb, ub)
    fval.append(fv)
    lag.append(lamt)
    return xtraj, ytraj, fval, lag

def max_oracle_gd(T, x0):
    '''
    First-order gradient descent optimization withthe oracle for max flow. Similar to above except that the Lagrangian for equality constraints corresponding to this function is set to zero.
    '''
    fval = []
    lb = -1
    ub = 1
    xtraj = dict()
    xtraj[0] = x0
    ytraj = dict()
    lag = []
    converged = False
    for t in range(1,T-1):
        # print("----------------------------------")
        # print("Iteration: "+str(t))
        yt, lamt, fv = Vin_oracle_orig(xtraj[t-1])
        print("Solve oracle problem for inner max")
        ytraj[t-1] = yt.copy()
        xstep = xtraj[t-1] - 0.25/t*gradient_orig(xtraj[t-1], ytraj[t-1], lamt)
        fval.append(fv)
        lag.append(lamt)
        xtraj[t] = projx(xstep, lb, ub)
        if xtraj[t] is not None:
            print("Solved projection")
            print("\n")
        else:
            pdb.set_trace()

    yt, lamt, fv = Vin_oracle_orig(xtraj[T-2])
    ytraj[T-2] = yt.copy()
    xstep = xtraj[T-2] - 0.25/t*gradient_orig(xtraj[T-2], ytraj[T-2], lamt)
    xtraj[T-1] = projx(xstep, lb, ub)
    fval.append(fv)
    lag.append(lamt)
    return xtraj, ytraj, fval, lag

def plot_strategies(xtraj, ytraj, lag):
    fig1, ax1 = plt.subplots()
    ax1.plot(list(xtraj.values()), 'b--')
    ax1.set_title("xtraj")

    fig2, ax2 = plt.subplots()
    ax2.plot(list(ytraj.values()), 'k--')
    ax2.set_title("ytraj")

    if len(lag[0]) == 1:
        fig3, ax3 = plt.subplots()
        ax3.plot(lag, 'r--')
        ax3.set_title("lagrangian")
    else:
        fig3, ax3 = plt.subplots(nrows=2, ncols=1)
        lag_first = [l[0] for l in lag]
        ax3[0].plot(lag_first, 'r--')
        ax3[0].set_title("lagrangian (first constraint)")

        lag_second = [l[1] for l in lag]
        ax3[1].plot(lag_second, 'r--')
        ax3[1].set_title("lagrangian (second constraint)")

    plt.show()

def plot_value_function():
    fig1, ax1 = plt.subplots()
    ax1.set_title("Value function for tester")
    ax1.set_xlabel("x")
    ax1.set_ylabel("V(x)")
    lam =1
    lam2 = 50
    x = np.linspace(-1,1,100)
    y1 = []
    y2 = []
    for xi in x:
        y1.append(lam2*xi**2+1 + lam)
        y2.append(lam2*xi**2+1 + lam*(1-xi))
    ax1.plot(x, np.array(y1), 'r')
    ax1.plot(x, np.array(y2), 'b')
    plt.show()

def plot_value_function_reg():
    fig1, ax1 = plt.subplots()
    ax1.set_title("Value function for tester")
    ax1.set_xlabel("x")
    ax1.set_ylabel("V(x)")
    lam = 0.5
    x = np.linspace(-1,1,100)
    y = []
    for xi in x:
        if xi < 0:
            y.append(-2*xi+1 + lam)
        else:
            y.append(-2*xi+1 + lam - lam*xi)
    ax1.plot(x, np.array(y), 'r--')
    plt.show()

if __name__ == '__main__':
    x,y = init_vars()
    x0 = -0.5
    T = 2500
    # xtraj, ytraj, fval, lag = max_oracle_gd(T, x0)
    # plot_strategies(xtraj, ytraj, lag)
    plot_value_function_reg()
    #plot_value_function_reg(lam)
