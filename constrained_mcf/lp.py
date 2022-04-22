import gurobipy as gp
from gurobipy import GRB
import sys

def solve_lp(filename):
    if not filename:
        print('Usage: lp.py filename')
        sys.exit(0)

    # Read and solve model
    model = gp.read(filename)
    model.optimize()

    if model.Status == GRB.INF_OR_UNBD:
        # Turn presolve off to determine whether model is infeasible
        # or unbounded
        model.setParam(GRB.Param.Presolve, 0)
        model.optimize()

    if model.Status == GRB.OPTIMAL:
        print('Optimal objective: %g' % model.ObjVal)
        model.write('model.sol')
        sys.exit(0)
    elif model.Status != GRB.INFEASIBLE:
        print('Optimization was stopped with status %d' % model.Status)
        sys.exit(0)
    # Model is infeasible - compute an Irreducible Inconsistent Subsystem (IIS)
    print('')
    print('Model is infeasible')
    model.computeIIS()
    model.write("model.ilp")
    print("IIS written to file 'model.ilp'")

def save_model(model, filename):
    model.write(filename)
