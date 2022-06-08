import sys
sys.path.append('..')
import numpy as np
from ipdb import set_trace as st
from collections import OrderedDict as od
import _pickle as pickle
import os
import networkx as nx
import gurobipy as gp
from gurobipy import GRB
from network import MazeNetwork, create_network_from_file
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from plotting import plot_flow, plot_maze
# import pao.pyomo
from pao.pyomo import *
import pyomo.environ as pyo
from pyomo.opt import SolverFactory

def bilevel(maze):
    G = nx.DiGraph(maze.gamegraph)
    # remove self loops
    edges = list(G.edges())
    for i,j in edges:
        if i == j:
            G.remove_edge(i,j)

    model = pyo.ConcreteModel()
    # st()
    model.nodes = list(G.nodes())
    model.edges = list(G.edges())

    src = (0,0)
    sink = (2,4)
    int = (4,0)

    vars = ['f1_e', 'f2_e', 'd_e', 'F']
    model.y = pyo.Var(vars, model.edges, within=pyo.NonNegativeReals)
    model.t = pyo.Var(within=pyo.NonNegativeReals)

    # Introduce SUBMODEL
    fixed_variables = [model.y['d_e',i,j] for i,j in model.edges]
    fixed_variables.extend([model.y['f1_e',i,j] for i,j in model.edges])
    fixed_variables.extend([model.y['f2_e',i,j] for i,j in model.edges])
    fixed_variables.extend([model.y['F',i,j] for i,j in model.edges])
    fixed_variables.extend([model.t])

    model.L = SubModel(fixed=fixed_variables)
    model.L.edges = model.edges
    model.L.nodes = model.nodes
    model.L.f3 = pyo.Var(model.L.edges, within=pyo.NonNegativeReals)
    #

    # st()
    # Objective - minimize the flow cut gap + flow_3
    def mcf_flow(model):
        lam = 3
        flow_3 = sum(model.L.f3[i,j] for (i, j) in model.L.edges if i == src)
        # return (sum(model.t[i,j] for (i,j) in model.edges) + lam * flow_3)
        return (model.t + lam * flow_3)
        # return (flow_3)
    def flow_cut_gap(model):
        # flow_3 = sum(model.L.f3[i,j] for (i, j) in model.L.edges if i == src)
        return sum(model.y['d_e',i,j] for (i,j) in model.edges)
    model.o = pyo.Objective(rule=mcf_flow, sense=pyo.minimize)

    # Constraints
    # Maximize the flow into the sink
    def flow_src1(model):
        return 1 <= sum(model.y['f1_e', i,j] for (i, j) in model.edges if i == src)
    def flow_src2(model):
        return 1 <= sum(model.y['f2_e', i,j] for (i, j) in model.edges if i == int)
    # def flow_src3(model):
    #     return 0 == sum(model.y['f3_e',i,j] for (i, j) in model.edges if i == s3)
    model.min_constr1 = pyo.Constraint(rule=flow_src1)
    model.min_constr2 = pyo.Constraint(rule=flow_src2)
    # model.min_constr3 = pyo.Constraint(rule=flow_src3)

    def capacity1(model, i, j, k, l):
        return model.y['f1_e',(i,j),(k,l)] <= model.t
    def capacity2(model, i, j, k, l):
        return model.y['f2_e', (i,j),(k,l)] <= model.t
    # def capacity3(model, i, j, k, l):
    #     return model.y['f3_e',(i,j),(k,l)] <= model.t
    model.cap1 = pyo.Constraint(model.edges, rule=capacity1)
    model.cap2 = pyo.Constraint(model.edges, rule=capacity2)
    # model.cap3 = pyo.Constraint(model.edges, rule=capacity3)

    # conservation constraints
    def conservation1(model, k, l):
        if (k,l) == src or (k,l) == int:
            return pyo.Constraint.Skip
        incoming  = sum(model.y['f1_e', i,j] for (i,j) in model.edges if j == (k,l))
        outgoing = sum(model.y['f1_e',i,j] for (i,j) in model.edges if i == (k,l))
        return incoming == outgoing
    model.cons1 = pyo.Constraint(model.nodes, rule=conservation1)

    def conservation2(model, k, l):
        if (k,l) == int or (k,l) == sink:
            return pyo.Constraint.Skip
        incoming  = sum(model.y['f2_e', i,j] for (i,j) in model.edges if j == (k,l))
        outgoing = sum(model.y['f2_e', i,j] for (i,j) in model.edges if i == (k,l))
        return incoming == outgoing
    model.cons2 = pyo.Constraint(model.nodes, rule=conservation2)

    # def conservation3(model, k, l):
    #     if (k,l) == src or (k,l) == sink:
    #         return pyo.Constraint.Skip
    #     incoming  = sum(model.y['f3_e', i,j] for (i,j) in model.edges if j == (k,l))
    #     outgoing = sum(model.y['f3_e',i,j] for (i,j) in model.edges if i == (k,l))
    #     return incoming == outgoing
    # model.cons3 = pyo.Constraint(model.nodes, rule=conservation3)

    # no flow into sources and out of sinks
    def no_in_source1(model, i,j,k,l):
        if (k,l) == src:
            return model.y['f1_e',(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.no_in_source1 = pyo.Constraint(model.edges, rule=no_in_source1)
    # nothing leaves sink
    def no_out_sink1(model, i,j,k,l):
        if (i,j) == int:
            return model.y['f1_e', (i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.no_out_sink1 = pyo.Constraint(model.edges, rule=no_out_sink1)
    # =================================================================== #
    def no_in_source2(model, i,j,k,l):
        if (k,l) == int:
            return model.y['f2_e',(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.no_in_source2 = pyo.Constraint(model.edges, rule=no_in_source2)

    # nothing leaves sink
    def no_out_sink2(model, i,j,k,l):
        if (i,j) == sink:
            return model.y['f2_e',(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.no_out_sink2 = pyo.Constraint(model.edges, rule=no_out_sink2)
    # =================================================================== #
    # def no_in_source3(model, i,j,k,l):
    #     if (k,l) == src:
    #         return model.y['f3_e',(i,j),(k,l)] == 0
    #     else:
    #         return pyo.Constraint.Skip
    # model.no_in_source3 = pyo.Constraint(model.edges, rule=no_in_source3)
    # # nothing leaves sink
    # def no_out_sink3(model, i,j,k,l):
    #     if (i,j) == sink:
    #         return model.y['f3_e',(i,j),(k,l)] == 0
    #     else:
    #         return pyo.Constraint.Skip
    # model.no_out_sink3 = pyo.Constraint(model.edges, rule=no_out_sink3)

    def cut_cons1(model, i, j, k, l):
        return model.y['f1_e',(i,j),(k,l)] + model.y['d_e',(i,j),(k,l)]<= model.t
    model.cut_cons1 = pyo.Constraint(model.edges, rule=cut_cons1)

    def cut_cons2(model, i, j, k, l):
        return model.y['f2_e',(i,j),(k,l)] + model.y['d_e',(i,j),(k,l)]<= model.t
    model.cut_cons2 = pyo.Constraint(model.edges, rule=cut_cons2)

    # set F = 1
    def auxiliary(model ,i, j, k, l):
        return model.y['F',i,j,k,l] == 1
    # Adding other auxiliary constraints to make t finite:
    model.aux_constr = pyo.Constraint(model.edges, rule=auxiliary)


    # # SUBMODEL
    # fixed_variables = [model.y['d_e',i,j] for i,j in model.edges]
    # fixed_variables.extend([model.y['f1_e',i,j] for i,j in model.edges])
    # fixed_variables.extend([model.y['f2_e',i,j] for i,j in model.edges])
    # fixed_variables.extend([model.y['F',i,j] for i,j in model.edges])
    # fixed_variables.extend([model.t])
    #
    # model.L = SubModel(fixed=fixed_variables)
    # model.L.edges = model.edges
    # model.L.nodes = model.nodes

    # Maximize the flow into the sink
    def flow_sink(model):
        return sum(model.f3[i,j] for (i, j) in model.edges if j == sink)
    model.L.o = pyo.Objective(rule=flow_sink, sense=pyo.maximize)

    # st()
    # Capacity constraints
    def capacity(mdl, i, j, k, l):
        return mdl.f3[i, j, k, l] <= model.t
    model.L.cap3 = pyo.Constraint(model.L.edges, rule=capacity)
    # st()
    # Conservation constraints
    def conservation(model, k, l):
        if (k,l) == sink or (k,l) == src:
            return pyo.Constraint.Skip
        incoming  = sum(model.f3[i,j] for (i,j) in model.edges if j == (k,l))
        outgoing = sum(model.f3[i,j] for (i,j) in model.edges if i == (k,l))
        return incoming == outgoing
    model.L.con = pyo.Constraint(model.L.nodes, rule=conservation)

    # # nothing enters the source
    def no_in_source(model, i,j,k,l):
        if (k,l) == src:
            return model.f3[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.L.no_in_source = pyo.Constraint(model.L.edges, rule=no_in_source)
    # nothing leaves sink
    def no_out_sink(model, i,j,k,l):
        if (i,j) == sink:
            return model.f3[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.L.no_out_sink = pyo.Constraint(model.L.edges, rule=no_out_sink)

    # nothing enters the intermediate or leaves the intermediate
    def no_in_interm(model, i,j,k,l):
        if (k,l) == int:
            return model.f3[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.L.no_in_interm = pyo.Constraint(model.L.edges, rule=no_in_interm)

    def no_out_interm(model, i,j,k,l):
        if (i,j) == int:
            return model.f3[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.L.no_out_interm = pyo.Constraint(model.L.edges, rule=no_out_interm)

    def cut_cons(mdl, i, j, k, l):
        return mdl.f3[(i,j),(k,l)] + model.y['d_e',(i,j),(k,l)]<= model.t
    model.L.cut_cons = pyo.Constraint(model.L.edges, rule=cut_cons)

    print(" ==== Successfully added objective and constraints! ==== ")
    model.pprint()

    with Solver('pao.pyomo.REG') as solver:
        results = solver.solve(model, tee=True)

    model.pprint()
    # st()
    f1_e = dict()
    f2_e = dict()
    f3_e = dict()
    d_e = dict()
    F = 0
    for (i,j),(k,l) in model.edges:
        F = (model.y['F', i,j,k,l].value)/(model.t.value)
        f1_e.update({((i,j),(k,l)): model.y['f1_e', i,j,k,l].value*F})
        f2_e.update({((i,j),(k,l)): model.y['f2_e', i,j,k,l].value*F})
        d_e.update({((i,j),(k,l)): model.y['d_e', i,j,k,l].value*F})
    for (i,j),(k,l) in model.L.edges:
        f3_e.update({((i,j),(k,l)): model.L.f3[i,j,k,l].value*F})

    # st()
    # for key in d_e.items():
    #     if d_e[key][-1] >= 0.5:
    #         print('Edge {} cut'.format(key))
    print(d_e)
    st()
    plot_flow(maze, f1_e , 'red')
    plot_flow(maze, f2_e , 'green')
    plot_flow(maze, f3_e , 'blue')


def bilevel_pyomo(maze):
    G = nx.DiGraph(maze.gamegraph)
    # remove self loops
    edges = list(G.edges())
    for i,j in edges:
        if i == j:
            G.remove_edge(i,j)

    model = pyo.ConcreteModel()
    # st()
    model.nodes = list(G.nodes())
    model.edges = list(G.edges())

    source = (5,0)
    sink = (0,9)
    int = (2,2)

    model.de = pyo.Var(model.edges, within=pyo.NonNegativeReals)
    # model.f1 = pyo.Var(model.edges, within=pyo.NonNegativeReals)
    # model.f2 = pyo.Var(model.edges, within=pyo.NonNegativeReals)

    # model.L = SubModel(fixed=[model.f1,model.f2,model.de])

    model.L = pao.pyomo.SubModel(fixed=[model.de])
    model.L.f3 = pyo.Var(model.edges, within=pyo.NonNegativeReals)
    # model.L.de = pyo.Var(model.edges, within=pyo.NonNegativeReals)
    model.L.nodes = list(G.nodes())
    model.L.edges = list(G.edges())
    # Maximize the flow into the sink
    def F(model):
        return sum(model.f2[i,j] for (i, j) in model.edges if j == sink)

    def cuts(model):
        return sum(model.de[i,j] for (i, j) in model.edges)

    def cuts_plus_f3(model):
        return sum(model.de[i,j] for (i, j) in model.edges) + sum(model.L.f3[i,j] for (i, j) in model.edges if j == sink)

    def flow_cut_gap(model):
        return sum(model.de[i,j] for (i, j) in model.edges) / sum(model.f2[i,j] for (i, j) in model.edges if j == sink)

    def flow_3(model):
        return sum(model.L.f3[i,j] for (i, j) in model.L.edges if j == sink)

    def flow_3_max(model):
        return sum(model.f3[i,j] for (i, j) in model.edges if j == sink)

    model.o = pyo.Objective(rule=cuts, sense=pyo.minimize)
    model.L.o = pyo.Objective(rule=flow_3_max, sense=pyo.maximize)

    # model.L.d = pyo.Constraint(expr= model.de == model.L.de)

    # # Capacity constraints
    # def capacity1(model, i, j, k, l):
    #     return model.f1[(i,j),(k,l)] <= 1
    # model.cap1 = pyo.Constraint(model.edges, rule=capacity1)
    # def capacity2(model, i, j, k, l):
    #     return model.f2[(i,j),(k,l)] <= 1
    # model.cap2 = pyo.Constraint(model.edges, rule=capacity2)
    def capacity3(model, i, j, k, l):
        return model.f3[(i,j),(k,l)] <= 1
    model.L.cap3 = pyo.Constraint(model.edges, rule=capacity3)

    # CUT CONSTRAINTS
    # # If cut -> no flow
    # def cut1(model, i, j ,k ,l):
    #     return model.f1[(i,j),(k,l)] + model.de[(i,j),(k,l)] <= 1
    # model.cut1 = pyo.Constraint(model.edges, rule=cut1)
    # def cut2(model, i, j ,k ,l):
    #     return model.f2[(i,j),(k,l)] + model.de[(i,j),(k,l)] <= 1
    # model.cut2 = pyo.Constraint(model.edges, rule=cut2)
    def cut3(model, i, j ,k ,l):
        return model.L.f3[(i,j),(k,l)] + model.de[(i,j),(k,l)] <= 1
    model.cut3 = pyo.Constraint(model.edges, rule=cut3)

    # # Conservation constraints
    # def conservation1(model, k, l):
    #     if (k,l) == int or (k,l) == source:
    #         return pyo.Constraint.Skip
    #     incoming  = sum(model.f1[i,j] for (i,j) in model.edges if j == (k,l))
    #     outgoing = sum(model.f1[i,j] for (i,j) in model.edges if i == (k,l))
    #     return incoming == outgoing
    # model.con = pyo.Constraint(model.nodes, rule=conservation1)
    # def conservation2(model, k, l):
    #     if (k,l) == int or (k,l) == sink:
    #         return pyo.Constraint.Skip
    #     incoming  = sum(model.f2[i,j] for (i,j) in model.edges if j == (k,l))
    #     outgoing = sum(model.f2[i,j] for (i,j) in model.edges if i == (k,l))
    #     return incoming == outgoing
    # model.con2 = pyo.Constraint(model.nodes, rule=conservation2)
    def conservation3(model, k, l):
        if (k,l) == source or (k,l) == sink:
            return pyo.Constraint.Skip
        incoming  = sum(model.f3[i,j] for (i,j) in model.edges if j == (k,l))
        outgoing = sum(model.f3[i,j] for (i,j) in model.edges if i == (k,l))
        return incoming == outgoing
    model.L.con3 = pyo.Constraint(model.nodes, rule=conservation3)

    # # nothing enters the source
    # def no_in_source(model, i,j,k,l):
    #     if (k,l) == source:
    #         return model.f1[(i,j),(k,l)] == 0
    #     else:
    #         return pyo.Constraint.Skip
    # model.no_in_source1 = pyo.Constraint(model.edges, rule=no_in_source)
    # # nothing leaves sink
    # def no_out_sink(model, i,j,k,l):
    #     if (i,j) == int:
    #         return model.f1[(i,j),(k,l)] == 0
    #     else:
    #         return pyo.Constraint.Skip
    # model.no_out_sink1 = pyo.Constraint(model.edges, rule=no_out_sink)
    # # FLOW 2
    # # nothing enters the source
    # def no_in_source2(model, i,j,k,l):
    #     if (k,l) == int:
    #         return model.f2[(i,j),(k,l)] == 0
    #     else:
    #         return pyo.Constraint.Skip
    # model.no_in_source2 = pyo.Constraint(model.edges, rule=no_in_source2)
    # # nothing leaves sink
    # def no_out_sink2(model, i,j,k,l):
    #     if (i,j) == sink:
    #         return model.f2[(i,j),(k,l)] == 0
    #     else:
    #         return pyo.Constraint.Skip
    # model.no_out_sink2 = pyo.Constraint(model.edges, rule=no_out_sink2)
    # FLOW 3
    # nothing enters the source
    def no_in_source3(model, i,j,k,l):
        if (k,l) == source:
            return model.f3[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.L.no_in_source3 = pyo.Constraint(model.edges, rule=no_in_source3)
    # nothing leaves sink
    def no_out_sink3(model, i,j,k,l):
        if (i,j) == sink:
            return model.f3[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.L.no_out_sink3 = pyo.Constraint(model.edges, rule=no_out_sink3)
    # No flow through intermediate for flow 3
    def no_flow_through_int_3(model, i,j,k,l):
        if (i,j) == int or (k,l) == int:
            return model.f3[(i,j),(k,l)] == 0
        else:
            return pyo.Constraint.Skip
    model.L.no_flow_int_3 = pyo.Constraint(model.edges, rule=no_flow_through_int_3)

    # st()
    # Create a solver
    # opt = pao.SolverFactory('pao.pyomo.FA', solver_io='direct')
    # solve
    opt = pao.Solver("pao.pyomo.FA")
    results = opt.solve(model)

    de = dict()
    cuts = list()
    for (i,j),(k,l) in model.edges:
        de.update({((i,j),(k,l)): model.de[i,j,k,l].value})
        if model.de[i,j,k,l].value >= 0.5:
            cuts.append([(i,j),(k,l)])


    flow3 = dict()
    for (i,j),(k,l) in model.edges:
        flow3.update({((i,j),(k,l)): model.L.f3[i,j,k,l].value})

    st()

    # flow = dict()
    # for (i,j),(k,l) in model.edges:
    #     flow.update({((i,j),(k,l)): model.f[i,j,k,l].value})

    # st()
    plot_flow(maze, flow3)

def x_oracle(maze, source,sink, de): # find max flow for given de
     # create model
    G = nx.DiGraph(maze.gamegraph)
    m = gp.Model()
    # remove self loops
    edges = list(G.edges())
    for i,j in edges:
        if i == j:
            G.remove_edge(i,j)

    # cut edges according to dictionary
    for i,j in edges:
        if de[(i,j)] == 1:
            G.remove_edge(i,j)

    # create variables
    f = {}

    for (i,j) in G.edges():
        f[i,j] = m.addVar(vtype=GRB.CONTINUOUS, name = "flow")

    m.setObjective((gp.quicksum(f[i,j] for i,j in G.edges if i == source or j == sink)), GRB.MAXIMIZE)

    # capacity constraints
    m.addConstrs((f[i,j] <= 1 for i, j in G.edges), "capacity")

    # conservation constraints
    outgoing = {}
    incoming = {}
    for node in G.nodes():
        outlist = []
        inlist = []
        if not node == source and not node == sink:
            for i,j in G.edges():
                if i == node and not j == node:
                    outlist.append((i,j))
                elif j == node and not i == node:
                    inlist.append((i,j))
        outgoing.update({node: outlist})
        incoming.update({node: inlist})
    m.addConstrs((gp.quicksum(f[i, j] for i, j in outgoing[j]) == gp.quicksum(f[j, k] for j, k in incoming[j]) for j in G.nodes), "conservation")

    # nothing enters the source
    m.addConstr((gp.quicksum(f[i,j] for i,j in G.edges if j == source) == 0), "source")
    # nothing exits the sink
    m.addConstr((gp.quicksum(f[i,j] for i,j in G.edges if i == sink) == 0),"sink")

    m.update()
    # solve IP model
    m.optimize()
    # st()
    flow = dict()
    max_flow = 0
    for (i,j) in G.edges():
        if i == source and f[i,j].x == 1:
            max_flow += 1
        flow.update({(i,j): f[i,j].x})

    # plot_flow(maze,flow)
    print("The max flow is {}".format(max_flow))
    return max_flow

def outer_optimization(G, source , sink, intermediate):
    # set the initial conditions
    # define value function
    # add constraints
    # iterate
    # call oracle for inner optimization
    pass

def gradient_descent(X, y, eta=0.01, num_iter=1500):
    """
    Solve using Gradient Descent.
    """
    mazefile = os.getcwd()+'/maze_new.txt'
    maze = MazeNetwork(mazefile)
    G = nx.DiGraph(maze.gamegraph)
    edges = G.edges()
    flow=np.array(np.zeros(edges))
    eta = 0.1
    m = len(y)
    J_history = []
    theta0_history = []
    theta1_history = []
    theta = theta.reshape(2,1)

    for i in range(num_iter):
        error = (np.dot(X, theta) - y)

        term0 = (eta/m) * sum(error* X[:,0].reshape(m,1))
        term1 = (eta/m) * sum(error* X[:,1].reshape(m,1))

        # update theta
        term_vector = np.array([[term0],[term1]])
#         print(term_vector)
        theta = theta - term_vector.reshape(2,1)

        # store history values
        theta0_history.append(theta[0].tolist()[0])
        theta1_history.append(theta[1].tolist()[0])
        J_history.append(compute_objective(X,y,theta).tolist()[0])
    return (theta, J_history, theta0_history, theta1_history)

def compute_objective(flow):
    source = (5,0)
    c = np.array([[1]])
    # c = np.vstack((np.zeros((3*nE,1)), np.array([[1]]), 0*np.ones((nE,1))))


def construct_objective(E):
    '''
    Constructs objective function as F
    '''
    nE = len(E)
    c = np.vstack((np.zeros((3*nE,1)), np.array([[1]]), 0*np.ones((nE,1))))
    return c


def setup_problem(maze, source, sink, intermediate):
    G = nx.DiGraph(maze.gamegraph)
    de = dict()
    for edge in G.edges():
        de.update({edge:0})

    x_oracle(maze, source,sink, de)
    pass

if __name__ == '__main__':
    mazefile = os.getcwd()+'/small_mazefile.txt'
    maze = MazeNetwork(mazefile)
    # source = (5,0)
    # sink = (0,9)
    # intermediate = (2,2)
    # setup_problem(maze, source, sink, intermediate)
    bilevel(maze)
