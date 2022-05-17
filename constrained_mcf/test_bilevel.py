import pyomo.environ as pe
from pao.pyomo import *

# Create a model object
M = pe.ConcreteModel()

# Define decision variables
M.x = pe.Var(bounds=(0,None))
M.y = pe.Var(bounds=(0,None))

# Define the upper-level objective
M.o = pe.Objective(expr=M.x - 4*M.y)
M.c1 = pe.Constraint(expr=   -M.x +   2*M.y <= 3)
M.c2 = pe.Constraint(expr=   M.y <= 3)

# Create a SubModel component to declare a lower-level problem
# The variable M.x is fixed in this lower-level problem
M.L = SubModel(fixed=M.x)

# Define the lower-level objective
M.L.o = pe.Objective(expr=M.y)

# Define lower-level constraints
M.L.c1 = pe.Constraint(expr=   -M.x -   M.y <= -3)
M.L.c2 = pe.Constraint(expr= -2*M.x +   M.y <=  0)
M.L.c3 = pe.Constraint(expr=  2*M.x +   M.y <= 12)
M.L.c4 = pe.Constraint(expr=  3*M.x - 2*M.y <=  4)

# Create a solver and apply it
with Solver('pao.pyomo.FA') as solver:
    results = solver.solve(M)

# The final solution is loaded into the model
print(M.x.value)
print(M.y.value)
