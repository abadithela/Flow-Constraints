# Simple examples to debug:
import pdb
import numpy as np
import cvxpy as cp

class ToyProb():
    def __init__(self, x0=None, y0=None, lam0 = None):
        if x0 is None:
            self.x0 = 0
        else:
            self.x0 = x0
        if y0 is None:
            self.y0 = 0
        else:
            self.y0 = y0

        if lam0 is None:
            set_init()
        else:
            self.lam0 = lam0

        self.T = 1000
        self.alpha = 0.5
        self.N_inner = 5
        self.A = None
        self.B = None
        self.c = None
        self.set_params()
        self.set_problem_data()

    def set_init(self):
        self.lam0 = np.array([[0],[0]], dtype=object,) # Change

    def set_params(self):
        self.T = 100
        self.alpha = 0.5
        self.N_inner = 5

    def set_problem_data(self):
        self.A = np.array([[1],[-1]], dtype=object,)
        self.B = np.array([[-1],[1]], dtype=object,)
        self.c = np.array([[1],[-1]], dtype=object,)

    def define(self):
        init = [self.x0, self.y0, self.lam0]
        params = [self.T, self.alpha, self.N_inner]
        problem_data = [self.A, self.B, self.c]
        return init, params, problem_data

    def gradf_x(self, x, y):
        return x+y

    def gradf_y(self, x, y):
        return x-y

    def projX(self, x):
        lbx = -1
        ubx = 1
        if isinstance(x, (list, tuple, np.ndarray)):
            xproj = cp.Variable(x.shape)
        else:
            xproj = cp.Variable()
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
        if isinstance(y, (list, tuple, np.ndarray)):
            yproj = cp.Variable(y.shape)
        else:
            yproj = cp.Variable()
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
        if isinstance(vec, (list, tuple, np.ndarray)):
            lb = np.zeros(vec.shape)
            vec_pos = cp.Variable(vec.shape)
        else:
            vec_pos = cp.Variable()
            lb = 0

        l2_norm =  cp.sum_squares(vec_pos - vec)
        constraints = [vec_pos >=lb]
        prob = cp.Problem(cp.Minimize(l2_norm), constraints)
        prob.solve()
        # pdb.set_trace()
        if prob.status == "infeasible" or prob.status == "unbounded":
             pdb.set_trace()
        else:
             return vec_pos.value
