# Linear Fractional Program (LFP)
# 13/5/2022
# Apurva Badithela
# Upper-level optimization without the connection to maximum flow at lower level
# This script solves a linear fractional program formulation of the flow bilevel optimization with three flows

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

class MCF():
    '''
    Class that takes in a graph with sources, sinks, and commodities and returns the multi-commodity flow (modified so commodities do not compete for capacities) self.m m that can be passed into Gurobi.
    '''
    def __init__(self):
        self.graph = nx.DiGraph()
        self.m = pyo.ConcreteModel()
        self.source_dict = None
        self.commodities = None
        self.sink_dict = None
        self.variables = None
        self.source_nodes = None
        self.sink_nodes = None
        self.sol = dict() # Dictionary storing solutions of variables
        self.edge_key_dict = dict() # Dictionary storing solutions of variables

    def add_graph(self, mazefile):
        '''
        Create game graph from file description.
        Graph's nodes are a list of tuples (i,j) where i and j are rows and columns of the graph
        Completed in Pyomo format.
        '''
        maze = MazeNetwork(mazefile)
        self.maze = maze
        self.graph = nx.DiGraph(maze.gamegraph)
        # for i,j in self.graph.edges:
        #     if i == j:
        #         self.graph.remove_edge(i,j)

        for i in self.graph.nodes:
            if (i,i) in self.graph.edges:
                self.graph.remove_edge(i,i)
        # pdb.set_trace() # Check how nodes of the graph work
        self.m.nodes = list(self.graph.nodes())
        self.m.edges = list(self.graph.edges())
        self.m.vars = ['f1_e', 'f2_e', 'f3_e', 'd_e', 'F']

    def add_source_dict(self, source_dict):
        '''
        Function to add source_dict to class of MCF!
        source_dict = {'s1': (1,2)}
        '''
        self.source_dict = source_dict
        if not self.commodities:
            self.commodities = ["c"+str(i) for i in range(len(source_dict.keys()))]
        if self.sink_dict:
            assert len(self.source_dict.keys()) == len(self.sink_dict.keys())
        self.source_nodes = [v for k, v in source_dict.items()]

    def add_sink_dict(self, sink_dict):
        '''
        Function to add sink_dict to class of MCF!
        sink_dict = {'t1': (1,2), 't2':(3,3), 't3': (1,2)}
        '''
        self.sink_dict = sink_dict
        if not self.commodities:
            self.commodities = ["c"+str(i) for i in range(len(sink_dict.keys()))]
        if self.source_dict:
            assert len(self.source_dict.keys()) == len(self.sink_dict.keys())
        self.sink_nodes = [v for k, v in sink_dict.items()]

    def add_positivity_constr(self):
        '''
        Pyomo conversion complete; In variable declaration
        '''
        for var in self.variables:
            if isinstance(var, dict):
                for e, fe in var.items():
                    self.m.addConstr(var[e] >= 0)
            else:
                try:
                    self.m.addConstr(var >= 0)
                except:
                    print("Variable type mismatch")
                    pdb.set_trace()

    def add_capacity_constr(self, model):
        '''
        Capacity constraints on each edge.
        Pyomo conversion complete
        '''
        def capacity1(model, i, j, k, l):
            return model.y['f1_e',(i,j),(k,l)] <= model.t
        def capacity2(model, i, j, k, l):
            return model.y['f2_e', (i,j),(k,l)] <= model.t
        def capacity3(model, i, j, k, l):
            return model.y['f3_e',(i,j),(k,l)] <= model.t

        model.cap1 = pyo.Constraint(model.edges, rule=capacity1)
        model.cap2 = pyo.Constraint(model.edges, rule=capacity2)
        model.cap3 = pyo.Constraint(model.edges, rule=capacity3)

    def add_cut_constr(self, model):
        '''
        If edge is cut; no flow along edge
        Pyomo conversion complete
        '''
        # self.m = self.m
        def cut_cons1(model, i, j, k, l):
            cut = model.y['f1_e',(i,j),(k,l)] + model.y['d_e',(i,j),(k,l)]<= model.t
            return cut

        def cut_cons2(model, i, j, k, l):
            cut = model.y['f2_e', (i,j),(k,l)] + model.y['d_e', (i,j),(k,l)]<= model.t
            return cut
        def cut_cons3(model, i, j, k, l):
            cut = model.y['f3_e',(i,j),(k,l)] + model.y['d_e',(i,j),(k,l)]<= model.t
            return cut

        model.cut_cons1 = pyo.Constraint(model.edges, rule=cut_cons1)
        model.cut_cons2 = pyo.Constraint(model.edges, rule=cut_cons2)
        model.cut_cons3 = pyo.Constraint(model.edges, rule=cut_cons3)
        def cut_ub(model, i,j,k,l):
            return model.y['d_e', (i,j), (k,l)] <= model.t
        model.cut_de = pyo.Constraint(model.edges, rule=cut_ub)

    # Debug: Check flow leaving nodes si:
    def check_flows(self, node):
        y, t = self.variables
        f1_e = self.m.y[0]
        f2_e = self.m.y[1]
        f3_e = self.m.y[2]
        outgoing = []
        incoming = []
        for ii,jj in self.graph.edges():
            if (ii == node) and not (jj == node):
                outgoing.append((ii,jj))
            elif (jj == node) and not (ii == node):
                incoming.append((ii,jj))
        flow_1 = {'in':{e:y['f1_e',e] for e in self.m.edges if e[1]==self.sink_dict['t1']}, "out":{e:y['f1_e',e] for e in self.m.edges if e[0]==self.source_dict['s1']}}

        flow_2 = {'in':{e:y['f2_e',e] for e in self.m.edges if e[1]==self.sink_dict['t2']}, "out":{e:y['f2_e',e] for e in self.m.edges if e[0]==self.source_dict['s2']}}

        flow_3 = {'in':{e:y['f3_e',e] for e in self.m.edges if e[1]==self.sink_dict['t3']}, "out":{e:y['f3_e',e] for e in self.m.edges if e[0]==self.source_dict['s3']}}
        # return incoming, outgoing, flow_1
        return incoming, outgoing, flow_1, flow_2, flow_3

    # Add constraints:
    def add_cons_constr(self,model):
        '''
        Pyomo conversion complete
        for v in V:
            ev_in <-- incoming edges to v
            ev_out <-- outgoing edges from v

            ck_in.T fk_e_in = ck_out.T fk_e_out
        '''
        # conservation constraints
        def conservation1(model, k, l):
            if (k,l) == self.source_dict["s1"] or (k,l) == self.sink_dict["t1"]:
                return pyo.Constraint.Skip
            incoming  = sum(model.y['f1_e', i,j] for (i,j) in model.edges if j == (k,l))
            outgoing = sum(model.y['f1_e',i,j] for (i,j) in model.edges if i == (k,l))
            return incoming == outgoing
        model.cons1 = pyo.Constraint(model.nodes, rule=conservation1)

        def conservation2(model, k, l):
            if (k,l) == self.source_dict["s2"] or (k,l) == self.sink_dict["t2"]:
                return pyo.Constraint.Skip
            incoming  = sum(model.y['f2_e', i,j] for (i,j) in model.edges if j == (k,l))
            outgoing = sum(model.y['f2_e', i,j] for (i,j) in model.edges if i == (k,l))
            return incoming == outgoing
        model.cons2 = pyo.Constraint(model.nodes, rule=conservation2)

        def conservation3(model, k, l):
            if (k,l) == self.source_dict["s3"] or (k,l) == self.sink_dict["t3"]:
                return pyo.Constraint.Skip
            incoming  = sum(model.y['f3_e', i,j] for (i,j) in model.edges if j == (k,l))
            outgoing = sum(model.y['f3_e',i,j] for (i,j) in model.edges if i == (k,l))
            return incoming == outgoing
        model.cons3 = pyo.Constraint(model.nodes, rule=conservation3)

    # Function to add variables:
    def add_vars(self, model):
        '''
        Pyomo conversion complete
        Creates and returns the variables for multi-commodity flow problem
        '''
        count = 0
        model.y = pyo.Var(model.vars, model.edges, within=pyo.NonNegativeReals)
        # pdb.set_trace()
        model.t = pyo.Var(within=pyo.NonNegativeReals)
        for (i,j) in self.graph.edges():
            self.edge_key_dict[count] = (i,j)
            count += 1
        self.variables = [model.y, model.t]

    # Function to add minimizing constraints:
    def add_min_constr(self, model):
        # conservation constraints
        s1 = self.source_dict["s1"]
        s2 = self.source_dict["s2"]
        s3 = self.source_dict["s3"]
        # Maximize the flow into the sink
        def flow_src1(model):
            return 1 <= sum(model.y['f1_e', i,j] for (i, j) in model.edges if i == s1)
        def flow_src2(model):
            return 1 <= sum(model.y['f2_e', i,j] for (i, j) in model.edges if i == s2)
        def flow_src3(model):
            return 1 <= sum(model.y['f3_e',i,j] for (i, j) in model.edges if i == s3)
        model.min_constr1 = pyo.Constraint(rule=flow_src1)
        model.min_constr2 = pyo.Constraint(rule=flow_src2)
        model.min_constr3 = pyo.Constraint(rule=flow_src3)

    # Function to add constraints:
    def add_constraints(self, model):
        self.add_min_constr(model) # F <= sum(f1_e) for e from s1 and F<= sum(f2_e) for e from s2
        self.add_capacity_constr(model)
        print(" == Added positivity, min, and capacity constraints! == ")
        self.add_cut_constr(model) # Cut constraints: if cut, no flow
        print(" == Added cut constraints! == ")

        # self.add_cons_constr() # Conservation at each node
        self.add_cons_constr(model) # Conservation at each node
        self.source_sink_cons(model) # Conservation at each node
        self.aux_constraint(model) # Conservation at each node
        print(" == Added conservation constraints and no inflow to source and no outflow from sink constraints! == ")


    # Adding auxiliary constraint:
    def aux_constraint(self, model):
        def auxiliary(model ,i, j, k, l):
            return model.y['F',i,j,k,l] == 1
        # Adding other auxiliary constraints to make t finite:

        model.aux_constr = pyo.Constraint(model.edges, rule=auxiliary)

    # Add objective function to the self.m:
    # Completed
    def add_objective(self, model):
        def mcf_flow(model):
            return sum(model.y['d_e',i,j] for (i,j) in model.edges)
        model.o = pyo.Objective(rule=mcf_flow, sense=pyo.minimize)


    def save_mcf_lp(self, filename):
        '''
        Saves the Multi-Commodity Flow LP in an LP self.m file.
        '''
        lp.save_self.m(self.m, filename)

    def compute_sol(self, model):
        '''
        Solves a linear program model saved in filename, and returns optimal output
        '''
        # Create a solver
        opt = SolverFactory('glpk')
        # solve
        results = opt.solve(model)

        f1_e = dict()
        f2_e = dict()
        f3_e = dict()
        d_e = dict()
        F = 0
        # pdb.set_trace()
        for (i,j),(k,l) in model.edges:
            F = (model.y['F', i,j,k,l].value)/(model.t.value)
            f1_e.update({((i,j),(k,l)): model.y['f1_e', i,j,k,l].value*F})
            f2_e.update({((i,j),(k,l)): model.y['f2_e', i,j,k,l].value*F})
            f3_e.update({((i,j),(k,l)): model.y['f3_e', i,j,k,l].value*F})
            d_e.update({((i,j),(k,l)): model.y['d_e', i,j,k,l].value*F})

        return f1_e, f2_e, f3_e, F, d_e

    # Process solution:
    def process_sol(self, opt_flows=None):
        '''
        Finds the outgoing commodity flows
        '''
        if not opt_flows:
            y, t = self.variables
            f1_e = y['f1_e',:]
            f2_e = y['f2_e',:]
            f3_e = y['f3_e',:]
        else:
            f1_e, f2_e, f3_e = opt_flows
        flow_c1 = sum([f1_e[ed]/t for ed in self.graph.edges if ed[0] == self.source_dict['s1']])
        flow_c2 = sum([f2_e[ed]/t for ed in self.graph.edges if ed[0] == self.source_dict['s2']])
        flow_c3 = sum([f3_e[ed]/t for ed in self.graph.edges if ed[0] == self.source_dict['s3']])
        # pdb.set_trace()
        cuts = [d_e[(i,j)]/t for i,j in self.graph.edges]
        # pdb.set_trace()
        return flow_c1, flow_c2, flow_c3, cuts
        # return flow_c1, cuts


    # No incoming flows to source and no outgoing flows to sink:
    # nothing enters the source
    def source_sink_cons(self, model):
        s1 = self.source_dict["s1"]
        t1 = self.sink_dict["t1"]
        s2 = self.source_dict["s2"]
        t2 = self.sink_dict["t2"]
        s3 = self.source_dict["s3"]
        t3 = self.sink_dict["t3"]

        def no_in_source1(model, i,j,k,l):
            if (k,l) == s1:
                return model.y['f1_e',(i,j),(k,l)] == 0
            else:
                return pyo.Constraint.Skip
        model.no_in_source1 = pyo.Constraint(model.edges, rule=no_in_source1)
        # nothing leaves sink
        def no_out_sink1(model, i,j,k,l):
            if (i,j) == t1:
                return model.y['f1_e', (i,j),(k,l)] == 0
            else:
                return pyo.Constraint.Skip
        model.no_out_sink1 = pyo.Constraint(model.edges, rule=no_out_sink1)
        # =================================================================== #
        def no_in_source2(model, i,j,k,l):
            if (k,l) == s2:
                return model.y['f2_e',(i,j),(k,l)] == 0
            else:
                return pyo.Constraint.Skip
        model.no_in_source2 = pyo.Constraint(model.edges, rule=no_in_source2)
        # nothing leaves sink
        def no_out_sink2(model, i,j,k,l):
            if (i,j) == t2:
                return model.y['f2_e',(i,j),(k,l)] == 0
            else:
                return pyo.Constraint.Skip
        model.no_out_sink2 = pyo.Constraint(model.edges, rule=no_out_sink2)
        # =================================================================== #
        def no_in_source3(model, i,j,k,l):
            if (k,l) == s3:
                return model.y['f3_e',(i,j),(k,l)] == 0
            else:
                return pyo.Constraint.Skip
        model.no_in_source3 = pyo.Constraint(model.edges, rule=no_in_source3)
        # nothing leaves sink
        def no_out_sink3(model, i,j,k,l):
            if (i,j) == t3:
                return model.y['f3_e',(i,j),(k,l)] == 0
            else:
                return pyo.Constraint.Skip
        model.no_out_sink3 = pyo.Constraint(model.edges, rule=no_out_sink3)
# =================================================================== #

    # Plot MCF flow
    def plot_flow(self, opt_flows=None):
        tilesize = 1
        if not opt_flows:
            y, t  = self.variables
            f1_e = y['f1_e',:]/t
            f2_e = y['f2_e',:]/t
            f3_e = y['f3_e',:]/t
            d_e = y['d_e',:]/t

        xs = np.linspace(0, 10*tilesize, 10+1)
        ys = np.linspace(0, 6*tilesize, 6+1)
        ax = plt.gca()
        ax.xaxis.set_visible(False)
        ax.yaxis.set_visible(False)
        # grid "shades" (boxes)
        w, h = xs[1] - xs[0], ys[1] - ys[0]
        for i, x in enumerate(xs[:-1]):
            for j, y in enumerate(ys[:-1]):
                if self.maze.map[j,i]=='*': # racing flag style
                    ax.add_patch(Rectangle((x, y), w, h, fill=True, color='black', alpha=.5))
                if self.maze.map[j,i]=='o': # racing flag style
                    ax.add_patch(Rectangle((x, y), w, h, fill=True, color='red', alpha=.5))
                elif self.maze.map[j,i]=='': # racing flag style
                    ax.add_patch(Rectangle((x, y), w, h, fill=True, color='black', alpha=.5))
                elif i % 2 == j % 2: # racing flag style
                    ax.add_patch(Rectangle((x, y), w, h, fill=True, color='gray', alpha=.1))
        # grid lines
        for x in xs:
            plt.plot([x, x], [ys[0], ys[-1]], color='black', alpha=.33, linestyle=':')
        for y in ys:
            plt.plot([xs[0], xs[-1]], [y, y], color='black', alpha=.33, linestyle=':')

        if solve_gurobi:
            color_flow = ['r', 'b', 'g']
            for idx, flow in enumerate([f3_e]):
                c = color_flow[idx]
                for item in flow:
                    if flow[item].x > 0.5:
                        startxy = item[0]
                        endxy = item[1]
                        plt.plot([startxy[1]+ tilesize/2, endxy[1]+ tilesize/2], [startxy[0]+ tilesize/2, endxy[0]+ tilesize/2], color=c, alpha=.33, linestyle='-')
        else:
            for flow in opt_flows:
                for item in flow:
                    if flow[item] > 0.5:
                        startxy = item[0]
                        endxy = item[1]
                        plt.plot([startxy[1]+ tilesize/2, endxy[1]+ tilesize/2], [startxy[0]+ tilesize/2, endxy[0]+ tilesize/2], color='red', alpha=.33, linestyle='-')

        ax.invert_yaxis()
        plt.show()


# Example of how this class should be used:
if __name__ == '__main__':
    mazefile = os.getcwd()+'/maze.txt'
    mcf_problem = MCF()

    # Setup
    mcf_problem.add_graph(mazefile) # Game graph
    source_dict = {'s1': (5,0), 's2': (2,2), 's3': (5,0)}
    sink_dict = {'t1': (2,2), 't2': (0,9), 't3': (0,9)}
    mcf_problem.add_source_dict(source_dict) # Source dict
    mcf_problem.add_sink_dict(sink_dict) # Sink dict
    model = mcf_problem.m
    mcf_problem.add_vars(model) # Create variables

    # Add objective and constraints:
    mcf_problem.add_objective(model)
    mcf_problem.add_constraints(model)
    print(" ==== Successfully added objective and constraints! ==== ")

    # mcf_problem.save_mcf_lp(filename)
    f1_e, f2_e, f3_e, F, d_e = mcf_problem.compute_sol(model)
    opt_flows = [f1_e, f2_e, f3_e]
    # mcf_problem.plot_flow(opt_flows=[f1_e, f2_e])
    pdb.set_trace()
    mcf_problem.plot_flow(opt_flows=[f1_e])
    mcf_problem.plot_flow(opt_flows=[f2_e])
    mcf_problem.plot_flow(opt_flows=[f3_e])
    flow_c1, cuts = mcf_problem.process_sol(opt_flows=opt_flows)

    # Solve for max flow:
    print("Solution constructed!")
    pdb.set_trace()
    print("Read solution")
    # Save LP solution:

    # Visualize the plot:
