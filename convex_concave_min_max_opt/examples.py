# Simple examples to debug:
import pdb
import numpy as np
import cvxpy as cp

class ToyProb():
    def __init__(self, x0=None, y0=None, lam0 = None):
        if x0 is None:
            self.x0 = 0
        if y0 is None:
            self.y0 = 0
        self.lam0 = None # Change
        self.T = 1000
        self.K = 10
        self.alpha = 0.5
        self.N_inner = 5
        self.A = None
        self.B = None
        self.c = None
        self.set_init()
        self.set_params()
        self.set_problem_data()

    def set_init(self):
        self.lam0 = None # Change

    def set_params(self):
        self.T = 1000
        self.K = 10
        self.alpha = 0.5
        self.N_inner = 5

    def set_problem_data(self):
        self.A = 1
        self.B = -1
        self.c = 1

    def define(self):
        init = [self.x0, self.y0, self.lam0]
        params = [self.T, self.K, self.alpha, self.N_inner]
        problem_data = [self.A, self.B, self.c]
        return init, params, problem_data

    def gradf_x(self, x, y):
        return x+y

    def gradg_y(self, x, y):
        return x-y

    def projX(self, x):
        lbx = -1
        ubx = 1
        k = len(x)
        xproj = cp.Variable(k)
        l2_norm =  cp.sum_squares(x - xproj)
        constraints = [xproj <= ubx, xproj >=lbx]
        prob = cp.Problem(cp.Minimize(l2_norm), constraints)
        prob.solve()
        # pdb.set_trace()
        if prob.status == "infeasible" or prob.status == "unbounded":
             pdb.set_trace()
        else:
             return xproj.value

    def projY(self, y):
        lby = -2
        uby = 0
        k = len(y)
        yproj = cp.Variable(k)
        l2_norm =  cp.sum_squares(y- yproj)
        constraints = [yproj <= uby, yproj >=lby]
        prob = cp.Problem(cp.Minimize(l2_norm), constraints)
        prob.solve()
        # pdb.set_trace()
        if prob.status == "infeasible" or prob.status == "unbounded":
             pdb.set_trace()
        else:
             return yproj.value

    def proj_pos(self, vec):
        k = len(vec)
        lb = np.zeros((k,1))
        vec_pos = cp.Variable(k) # Solves for -x --> ???
        l2_norm =  cp.sum_squares(vec_pos - vec)
        constraints = [vec_pos >=lb]
        prob = cp.Problem(cp.Minimize(l2_norm), constraints)
        prob.solve()
        # pdb.set_trace()
        if prob.status == "infeasible" or prob.status == "unbounded":
             pdb.set_trace()
        else:
             return vec_pos.value
