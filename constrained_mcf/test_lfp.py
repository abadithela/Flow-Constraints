import networkx as nx
import numpy as np
import gurobipy as gp
from gurobipy import GRB
import matplotlib.pyplot as plt
import sys
import os
import lp
from matplotlib.patches import Rectangle
import scipy.sparse as sp
sys.path.append('..')
from static_obstacle_maze.network import MazeNetwork
import pdb
import pyomo.environ as pyo
from pyomo.opt import SolverFactory
solve_gurobi = False


model = pyo.ConcreteModel()

def lfp_prob_data():
    model.n = 4 # dim of x
    model.m = 4 # A \i R^(m*n)
    model.c = np.array([[1] for i in range(model.n)])
    model.alpha = -2
    model.beta = 3
    model.d = np.array([[2] for i in range(model.n)])
    Ac = np.random.rand(model.m,model.m)
    model.A = 0.5*np.identity(model.m) + np.dot(Ac, Ac.transpose())
    model.b = np.array([[1] for i in range(model.m)])

def add_objective(model):
    return (sum(model.c[k,0]*model.y[k] for k in range(model.n)) + model.alpha*model.t)

def add_yineq_cons(model, k):
    return (sum(model.A[k,i]*model.y[i] for i in range(model.n)) - model.b[k]*model.t <= 0)

def add_tineq_cons(model):
    return (model.t >= 0)

def add_eq_cons(model):
    return (sum(model.d[k,0]*model.y[k] for k in range(model.n)) + model.beta * model.t == 1)

def transformed_lfp():
    lfp_prob_data()
    model.xind = list(range(model.n))
    # pdb.set_trace()
    # Create variables:
    model.y = pyo.Var(model.xind, within=pyo.NonNegativeReals)
    model.t = pyo.Var(within=pyo.NonNegativeReals)
    model.o = pyo.Objective(rule=add_objective, sense=pyo.maximize)
    # pdb.set_trace()
    model.yineq = pyo.Constraint(model.xind, rule=add_yineq_cons)
    model.tineq = pyo.Constraint(rule=add_tineq_cons)
    model.eq = pyo.Constraint(rule=add_eq_cons)

def solve_lfp():
    opt = SolverFactory('glpk')
    results = opt.solve(model)
    y = []
    t = model.t.value
    for i in model.xind:
        y.append(model.y[i].value)
    x = np.array(y)/t
    return x, y, t

if __name__ == '__main__':
    transformed_lfp()
    xopt, yopt, topt = solve_lfp()
    print("t: ", str(topt))
    print("y: ")
    print(yopt)
    print("x: ")
    print(xopt)
