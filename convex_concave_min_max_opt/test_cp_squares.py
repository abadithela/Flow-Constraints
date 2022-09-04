# Import packages.
import cvxpy as cp
import numpy as np
import osqp
import scipy as sp
from scipy import sparse
import pdb

def main_cvxpy():
    # Generate a random non-trivial linear program.
    m = 15
    n = 10
    np.random.seed(1)
    s0 = np.random.randn(m)
    lamb0 = np.maximum(-s0, 0)
    s0 = np.maximum(s0, 0)
    x0 = np.random.randn(n)
    y0 = np.random.randn(n)
    yc = -10*np.ones((n,)) + y0
    A = np.random.randn(m, n)
    b = A @ x0 + s0
    c = -A.T @ lamb0
    c1 = np.ones((n,))*0.5

    # Define and solve the CVXPY problem.
    x = cp.Variable(n)
    y = cp.Variable(n)
    prob = cp.Problem(cp.Minimize(c.T@x + c1.T@y),
                     [A @ x <= b, y==y0, y>= yc])
    prob.solve()

    # Print result.
    print(prob.status)
    print("\nThe optimal value is", prob.value)
    print("A solution x is")
    print(x.value)
    print("A dual solution is")
    print(prob.constraints[0].dual_value)
    print(prob.constraints[1].dual_value)
    print(prob.constraints[2].dual_value)

def main_osqp():
    # Generate problem data
    sp.random.seed(1)
    m = 30
    n = 20
    Ad = sparse.random(m, n, density=0.7, format='csc')
    b = np.random.randn(m)
    # OSQP data
    P = sparse.block_diag([sparse.csc_matrix((n, n)), sparse.eye(m)], format='csc')
    q = np.zeros(n+m)
    A = sparse.vstack([
            sparse.hstack([Ad, -sparse.eye(m)]),
            sparse.hstack([sparse.eye(n), sparse.csc_matrix((n, m))])], format='csc')
    l = np.hstack([b, np.zeros(n)])
    u = np.hstack([b, np.ones(n)])

    # Create an OSQP object
    prob = osqp.OSQP()

    # Setup workspace
    prob.setup(P, q, A, l, u)

    # Solve problem
    res = prob.solve()

# Generate data.
if __name__ == '__main__':
    main_cvxpy()
