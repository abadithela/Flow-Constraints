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

solve_gurobi = True
class MCF():
    '''
    Class that takes in a graph with sources, sinks, and commodities and returns the multi-commodity flow (modified so commodities do not compete for capacities) model m that can be passed into Gurobi.
    '''
    def __init__(self):
        self.graph = nx.DiGraph()
        self.m = gp.Model()
        self.maze = None
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

    def add_source_dict(self, source_dict):
        '''
        Function to add source_dict to class of MCF!
        source_dict = {'s1': (1,2), 's2':(3,3), 's3': (1,2)}
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

    def add_capacity_constr(self, edge_vars):
        for var in edge_vars:
            assert isinstance(var, dict)
            for e, fe in var.items():
                self.m.addConstr(var[e] <= 1)

    def add_cut_constr(self):
        f1_e, f2_e, f3_e, F, d_e = self.variables
        assert set(f1_e.keys()) == set(d_e.keys())
        assert set(f2_e.keys()) == set(d_e.keys())
        assert set(f3_e.keys()) == set(d_e.keys())

        for e in d_e.keys():
            f1e = f1_e[e]
            f2e = f2_e[e]
            f3e = f3_e[e]
            de = d_e[e]
            self.m.addConstr(f1e + de <= 1)
            self.m.addConstr(f2e + de <= 1)
            self.m.addConstr(f3e + de <= 1)
            self.m.addConstr(de <= 1) # Max value of d_e

    # Debug: Check flow leaving nodes si:
    def check_flows(self, node):
        f1_e, f2_e, f3_e, F, d_e = self.variables
        outgoing = []
        incoming = []
        for ii,jj in self.graph.edges():
            if (ii == node) and not (jj == node):
                outgoing.append((ii,jj))
            elif (jj == node) and not (ii == node):
                incoming.append((ii,jj))
        flow_1 = {'in':{e:f1_e[e].x for e in incoming}, "out":{e:f1_e[e].x for e in outgoing}}
        flow_2 = {'in':{e: f2_e[e].x for e in incoming}, "out":{e:f2_e[e].x for e in outgoing}}
        flow_3 = {'in':{e:f3_e[e].x for e in incoming}, "out":{e:f3_e[e].x for e in outgoing}}
        return incoming, outgoing, flow_1, flow_2, flow_3

    # Add constraints:
    def add_cons_constr(self):
        '''
        for v in V:
            ev_in <-- incoming edges to v
            ev_out <-- outgoing edges from v

            ck_in.T fk_e_in = ck_out.T fk_e_out
        '''
        # conservation constraints
        f1_e, f2_e, f3_e, F, d_e = self.variables
        outgoing = {}
        incoming = {}
        for node in self.graph.nodes():
            # if node not in self.source_nodes and node not in self.sink_nodes:
            outlist = []
            inlist = []
            for ii,jj in self.graph.edges():
                if (ii == node) and not (jj == node):
                    outlist.append((ii,jj))
                elif (jj == node) and not (ii == node):
                    inlist.append((ii,jj))
            outgoing.update({node: outlist})
            incoming.update({node: inlist})
        # nodes contains all vertices of the graph except soiurce and sink nodes
        nodes = set(self.graph.nodes)
        for n in self.source_nodes:
            if n in nodes:
                nodes.remove(n)
        for n in self.sink_nodes:
            if n in nodes:
                nodes.remove(n)

        self.m.addConstrs((gp.quicksum(f1_e[ii, jj] for ii, jj in outgoing[j]) == gp.quicksum(f1_e[jj, kk] for jj, kk in incoming[j]) for j in nodes), "conservation_f1_e")
        self.m.addConstrs((gp.quicksum(f2_e[ii, jj] for ii, jj in outgoing[j]) == gp.quicksum(f2_e[jj, kk] for jj, kk in incoming[j]) for j in nodes), "conservation_f2_e")
        self.m.addConstrs((gp.quicksum(f3_e[ii, jj] for ii, jj in outgoing[j]) == gp.quicksum(f3_e[jj, kk] for jj, kk in incoming[j]) for j in nodes), "conservation_f3_e")

    # Function to add variables:
    def add_vars(self):
        '''
        Creates and returns the variables for multi-commodity flow problem
        '''
        f1_e = {} # Flow for commodity 1
        f2_e = {} # Flow for commodity 2
        f3_e = {} # Flow for commodity
        d_e = {} # Cuts in graph
        count = 0
        for (i,j) in self.graph.edges():
            f1_e[i,j] = self.m.addVar(vtype=GRB.CONTINUOUS, name = "c1_flow")
            f2_e[i,j] = self.m.addVar(vtype=GRB.CONTINUOUS, name = "c2_flow")
            f3_e[i,j] = self.m.addVar(vtype=GRB.CONTINUOUS, name = "c3_flow")
            d_e[i,j] = self.m.addVar(vtype=GRB.CONTINUOUS, name = "cut_var")
            self.edge_key_dict[count] = (i,j)
            count += 1
        F = self.m.addVar(vtype=GRB.CONTINUOUS, name="F")
        self.variables = [f1_e, f2_e, f3_e, F, d_e]

    # Function to add minimizing constraints:
    def add_min_constr(self):
        f1_e, f2_e, f3_e, F, d_e = self.variables
        min_constr1 = (F <= gp.quicksum(f1_e[i,j] for i,j in self.graph.edges if i == self.source_dict['s1']))
        min_constr2 = (F <= gp.quicksum(f2_e[i,j] for i,j in self.graph.edges if i == self.source_dict['s2']))
        min_constr3 = (F <= gp.quicksum(f3_e[i,j] for i,j in self.graph.edges if i == self.source_dict['s3']))
        self.m.addConstr(min_constr1)
        self.m.addConstr(min_constr2)
        self.m.addConstr(min_constr3)
    # Function to add constraints:
    def add_constraints(self):
        f1_e, f2_e, f3_e, F, d_e = self.variables
        self.add_min_constr() # F <= sum(f1_e) for e from s1 and F<= sum(f2_e) for e from s2
        try:
            self.add_positivity_constr()
            self.add_capacity_constr([f1_e, f2_e, f3_e])

            print(" == Added positivity, min, and capacity constraints! == ")
        except:
            print(" == Error in adding positivity, min or capacity constraints == ")
            pdb.set_trace()

        try:
            self.add_cut_constr() # Cut constraints: if cut, no flow
            print(" == Added cut constraints! == ")
        except:
            print(" == Error in adding cut constraints! == ")
            pdb.set_trace()

        try:
            self.add_cons_constr() # Conservation at each node
            print(" == Added conservation constraints! == ")
        except:
            print(" == Error in adding conservation constraints! == ")
            pdb.set_trace()

    # Add objective function to the model:
    def add_objective(self):
        f1_e, f2_e, f3_e, F, d_e = self.variables
        self.m.setObjective(F, GRB.MAXIMIZE)

    def save_mcf_lp(self, filename):
        '''
        Saves the Multi-Commodity Flow LP in an LP model file.
        '''
        lp.save_model(self.m, filename)

    def compute_sol(self, filename):
        '''
        Solves a linear program model saved in filename, and returns optimal output
        '''
        sol = lp.solve_lp(self.m)
        nE = len(self.graph.edges())
        self.sol["f1opt"] = sol[0:nE]
        self.sol["f2opt"] = sol[nE+1: 2*nE]
        self.sol["f3opt"] = sol[2*nE+1: 3*nE]
        self.sol["Fopt"] = sol[3*nE+1]
        self.sol["deopt"] = sol[3*nE+2:]

    # Process solution:
    def process_sol(self):
        '''
        Finds the outgoing commodity flows
        '''
        f1_e, f2_e, f3_e, F, d_e = self.variables
        flow_c1 = sum([f1_e[ed].x for ed in self.graph.edges if ed[0] == self.source_dict['s1']])
        flow_c2 = sum([f2_e[(i,j)].x for i,j in self.graph.edges if i == self.source_dict['s2']])
        flow_c3 = sum([f3_e[(i,j)].x for i,j in self.graph.edges if i == self.source_dict['s3']])
        # pdb.set_trace()
        cuts = [d_e[(i,j)].x for i,j in self.graph.edges]
        # pdb.set_trace()
        return flow_c1, flow_c2, flow_c3, cuts

    # Plot MCF flow
    def plot_flow(self):
        tilesize = 1
        f1_e, f2_e, f3_e, F, d_e = self.variables
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
    mcf_problem.add_vars() # Create variables

    # Add objective and constraints:
    mcf_problem.add_objective()
    mcf_problem.add_constraints()
    print(" ==== Successfully added objective and constraints! ==== ")

    # Dump model in file:
    filename = "mcf.mps"
    mcf_problem.m.update() # Updating model
    mcf_problem.save_mcf_lp(filename)
    mcf_problem.compute_sol(filename)
    flow_c1, flow_c2, flow_c3, cuts = mcf_problem.process_sol()
    mcf_problem.plot_flow()

    # Solve for max flow:
    print("Solution constructed!")
    incoming_s1, outgoing_s1, flow1_s1, flow2_s1, flow3_s1 = mcf_problem.check_flows(source_dict['s1'])
    incoming_t1, outgoing_t1, flow1_t1, flow2_t1, flow3_t1 = mcf_problem.check_flows(sink_dict['t1'])
    pdb.set_trace()
    print("Read solution")
    # Save LP solution:

    # Visualize the plot:
