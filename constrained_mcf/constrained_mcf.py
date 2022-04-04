import networkx as nx
import numpy as np
import gurobipy as gp
from gurobipy import GRB

# Construct matrices:
# x = [--fp--, F, --de--]

G, source_dict, sink_dict = load_graph()
Edges = G.edges()
Verts = G.nodes()
nV = len(Verts)
Edict = {k: Edges(k) for k in range(len(Edges))}
Edict_rev = {v:k for k,v in Edict.items()}
Vdict = {k: Verts(k) for k in range(len(Verts))}
Vdict_rev = {v:k for k,v in Vdict.items()}
numx = len(E)*3 + 1 + len(E)

def load_graph(m, n):
    G = nx.DiGraph()
    nodes = []
    edges = []
    for row in range(1,m+1):
        for col in range(1,n+1):
            cell = (row-1)*n + col
            succ_cells = [cell-n, cell+n, cell-1, cell+1]
            nodes.append("c"+str(cell))

def source_outgoing_edge_indices(k=1):


def construct_objective(E):
    nE = len(E)
    c = np.vstack((np.zeros((3*nE,1)), np.array([[1]]), -1*np.ones((nE,1)))
    return c

def construct_min_constraints(E):
    numx = len(E)*3 + 1 + len(E)
    Fmin = np.zeros((2, numx))
    bmin = np.zeros((2,1))
    v1_indices = source_outgoing_edge_indices(k=1)
    return Fmin, bmin

def positivity_constraints(E):
    numx = len(E)*3 + 1 + len(E)
    Fmin = np.zeros((2, numx))
    bmin = np.zeros((2,1))
    v1_indices = source_outgoing_edge_indices(k=1)
    return Fmin, bmin

# ================================== Functions to handle cycle constraints ========================================== #
# Function for MiLP to accommodate cycles:
# Inputs Matrices: c, A, d
# Outputs: edges to remove: newC, optimal flow vector: fopt, timed_out or infeasibility optimization variables
def milp_cycles(c, A, b, G, h):
    numx = len(c)
    epsilon = 0.5 # Factor that keeps b variables to 1
    newC = []
    fopt = 0
    timed_out = False
    feas = False
    try:
        # Create a new model
        m = gp.Model("ConsMCF_LP")
        m.setParam('OutputFlag', 0)  # Also dual_subproblem.params.outputflag = 0
        m.setParam(GRB.Param.TimeLimit, 300.0) # Setting time limit: 5 min
        # Create variables:
        x = m.addMVar(numx, vtype=GRB.CONTINUOUS, name="x")
        z = m.addMVar(nV, vtype=GRB.CONTINUOUS, name="z")
        m.params.threads = 4
        # Set objective: c.T*b; minimizing cuts to augmenting paths pj to pj+1
        m.setObjective(c @ x, GRB.MAXIMIZE)



        ones_b = np.ones((nc,))
        ones_f = np.ones((nf,))
        m.addConstr(np.diag(D_f) @ f + A_f @ b <= A_f @ ones_b, name="c3")
        m.addConstr(A_f @ ones_b  + ones_f <= f + A_f @ b + D_f @ ones_f, name="c4")

        # Optimize model
        m.optimize()
        if(m.status==GRB.INFEASIBLE):
            feas = True
        if(m.status == GRB.TIME_LIMIT):
            timed_out = True

        if feas!=True and timed_out!=True:
            xopt=np.zeros((ne,1))
            bopt = np.zeros((nc,1))
            fopt = np.zeros((nf,1))
            xidx = 0
            bidx = 0
            fidx = 0
            for v in m.getVars():
                if(xidx < ne and bidx < nc):
                    xopt[xidx] = v.x
                    xidx+=1
                elif(xidx==ne and bidx < nc):
                    bopt[bidx] = v.x
                    bidx+=1
                else:
                    fopt[fidx] = v.x
                    fidx += 1

            # get_constraints

    except gp.GurobiError as e:
        print('Error code ' + str(e.errno) + ': ' + str(e))

    except AttributeError:
        print('Encountered an attribute error')

    return newC, fopt, timed_out, feas
