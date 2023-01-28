# Multiple gradient descent and gradient descent ascent for paper on solving min-max problems with coupled linear constraints
# 1/25/23 Apurva Badithela
import numpy as np
import pdb
from examples import ToyProb
import matplotlib.pyplot as plt

def mgd(problem, init, params, problem_data):
    '''
    Multiplier Gradient Descent Algorithm
    '''
    x0, y0, lam0 = init
    T, alpha, N_inner = params
    A, B, c = problem_data
    params_gda = [N_inner, alpha, alpha]
    xh, yh, lamh = [x0], [y0], [lam0]
    for r in range(0, T):
        init_r = [xh[r], yh[r], lamh[r]]
        xgda, ygda, lam_gda = gda(problem, init_r, params_gda, problem_data)
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
        if isinstance(B, (np.ndarray)):
            yn = problem.projY(yh[r] + beta*(problem.gradf_y(xh[r], yh[r]) - np.dot(B.T, lamh[r])))
            xn = problem.projX(xh[r] - alpha*(problem.gradf_x(xh[r], yn) + np.dot(A.T, lamh[r])))
        else:
            yn = problem.projY(yh[r] + beta*(problem.gradf_y(xh[r], yh[r]) - B*lamh[r]))
            xn = problem.projX(xh[r] - alpha*(problem.gradf_x(xh[r], yn) + A*lamh[r]))
        lamn = problem.proj_pos(lamh[r] - alpha*(-A*xh[r] - B*yn + c))
        xh.append(xn)
        yh.append(yn)
        lamh.append(lamn)
    return xh, yh, lamh

def plot_conv(xh, yh, lamh):
    fig1, ax1 = plt.subplots(3)
    fig1.suptitle("mgd")
    ax1[0].set_xlabel("iter")
    ax1[0].set_ylabel("x")
    ax1[0].set_ylim([-1,1])
    ax1[0].plot(xh, 'r')

    ax1[1].set_xlabel("iter")
    ax1[1].set_ylabel("y")
    ax1[1].plot(yh, 'r')
    ax1[1].set_ylim([-2,0])

    ax1[2].set_xlabel("iter")
    ax1[2].set_ylabel("lambda")
    lam0 = [lamh[i][0] for i in range(len(lamh))]
    lam1 = [lamh[i][1] for i in range(len(lamh))]
    ax1[2].plot(lam0, 'r', label="lam[0]")
    ax1[2].plot(lam1, 'k', label="lam[1]")
    ax1[2].legend()
    plt.savefig("debugging_imgs/mgd_toy_example.png")
    plt.show()

if __name__=='__main__':
    problem = ToyProb(x0 = 0.75, y0 = -0.25, lam0 = np.array([[0],[0]], dtype=object,)) # From paper on coupled constraints
    init, params, problem_data = problem.define()
    xh, yh, lamh = mgd(problem, init, params, problem_data)
    plot_conv(xh, yh, lamh)
