import gurobipy as gp
from gurobipy import GRB
import sys
import pdb


def solve_lp_from_file(filename):
    if not filename:
        print('Usage: lp.py filename')
        sys.exit(0)

    # Read and solve model
    solution = None
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
        solution = model.getAttr("X", model.getVars())

    elif model.Status != GRB.INFEASIBLE:
        print('Optimization was stopped with status %d' % model.Status)
    # Model is infeasible - compute an Irreducible Inconsistent Subsystem (IIS)
        print('')
        print('Model is infeasible')
        model.computeIIS()
        model.write("model.ilp")
        print("IIS written to file 'model.ilp'")
    return solution

def solve_lp(model):
    # Read and solve model
    solution = None
    model.optimize()

    if model.Status == GRB.INF_OR_UNBD:
        # Turn presolve off to determine whether model is infeasible
        # or unbounded
        model.setParam(GRB.Param.Presolve, 0)
        model.optimize()

    if model.Status == GRB.OPTIMAL:
        print('Optimal objective: %g' % model.ObjVal)
        model.write('model.sol')
        solution = model.getAttr("X", model.getVars())

    elif model.Status != GRB.INFEASIBLE:
        print('Optimization was stopped with status %d' % model.Status)
    # Model is infeasible - compute an Irreducible Inconsistent Subsystem (IIS)
        print('')
        print('Model is infeasible')
        model.computeIIS()
        model.write("model.ilp")
        print("IIS written to file 'model.ilp'")
    return solution

def save_model(model, filename):
    model.write(filename)
