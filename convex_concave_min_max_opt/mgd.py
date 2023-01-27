# Multiple gradient descent and gradient descent ascent for paper on solving min-max problems with coupled linear constraints
# 1/25/23 Apurva Badithela
import numpy as np
import pdb
from examples import ToyProb

def mgd(problem, init, params, problem_data):
    '''
    Multiplier Gradient Descent Algorithm
    '''
    x0, y0, lam0 = init
    T, K, alpha, N_inner = params
    A, B, c = problem_data
    params_gda = [N_inner, alpha, alpha]
    xh, yh, lamh = [x0], [y0], [lam0]
    for r in range(0, T):
        init_r = [xh[r], yh[r], lamh[r]]
        xgda, ygda, lam_gda = gda(init_r, params_gda, problem_data)
        xn = xgda[-1]
        yn = ygda[-1]
        lamn = problem.proj_pos(lamh[r] - alpha*(-A*xn - B*yn + c))
        xh.append(xn)
        yh.append(yn)
        lamh.append(lamn)
    return xh, yh, lamh

def gda(problem, init, params, problem_data):
    '''
    Gradient Descent Ascent Algorithm. The inputs are:

    '''
    x0, y0, lam0 = init
    T, alpha, beta = params
    A, B, c = problem_data

    xh, yh, lamh = [x0], [y0], [lam0]
    for r in range(0, T):
        yn = problem.projY(yh[r] + beta*(problem.gradf_y(x[r], y[r]) - np.dot(B.T, lamh[r])))
        xn = problem.projX(xh[r] - alpha*(problem.gradf_x(x[r], y[r+1]) + np.dot(A.T, lamh[r])))
        lamn = problem.proj_pos(lamh[r] - alpha*(-A*xh[r] - B*yn + c))
        xh.append(xn)
        yh.append(yn)
        lamh.append(lamn)
    return xh, yh, lamh

if __name__=='__main__':
    problem = ToyProb() # From paper on coupled constraints
    init, params, problem_data = problem.define()
    xh, yh, lamh = mgd(problem, init, params, problem_data)
