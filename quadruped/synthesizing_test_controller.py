"""
Script to synthesize a controller to test the interface to the quadruped.
After executing this script the controller will be saved in quadruped_test_controller.py

Gridworld:

||_|_|
|_|_|_|
|_|_|_|

----------------
| X6 | X7 | X8 |
----------------
| X3 | X4 | X5 |
----------------
| X0 | X1 | X2 |
----------------
Auxiliary states: 'X2_lie', 'X8_jump', 'X6_stand'

"""

from __future__ import print_function
import logging
from tulip import transys, spec, synth
from tulip import dumpsmach

logging.basicConfig(level=logging.WARNING)
logging.getLogger('tulip.spec.lexyacc').setLevel(logging.WARNING)
logging.getLogger('tulip.synth').setLevel(logging.WARNING)
logging.getLogger('tulip.interfaces.omega').setLevel(logging.WARNING)

# Create a finite transition system
sys = transys.FTS()


# Define the states of the system
sys.states.add_from(['X0', 'X1', 'X2', 'X3', 'X4', 'X5', 'X6', 'X7', 'X8', 'X2_lie', 'X8_jump', 'X6_stand'])
sys.states.initial.add('X0')    # start in state X0

# Define the allowable transitions
sys.transitions.add_comb({'X0'}, {'X1','X3', 'X0'})
sys.transitions.add_comb({'X1'}, {'X0','X2','X4', 'X1'})
sys.transitions.add_comb({'X2'}, {'X5', 'X1','X2', 'X2_lie'})
sys.transitions.add_comb({'X3'}, {'X0', 'X4','X6', 'X3'})
sys.transitions.add_comb({'X4'}, {'X1','X3', 'X5', 'X7', 'X4'})
sys.transitions.add_comb({'X5'}, {'X2','X4', 'X8', 'X5'})
sys.transitions.add_comb({'X6'}, {'X3','X7', 'X6', 'X6_stand'})
sys.transitions.add_comb({'X7'}, {'X4','X6', 'X8'})
sys.transitions.add_comb({'X8'}, {'X5','X7', 'X8', 'X8_jump'})

# Add atomic propositions to the states
sys.atomic_propositions.add_from({'goal', 'jump', 'lie', 'stand'})
#sys.states.add('X0', ap={'init'})
sys.states.add('X2_lie', ap={'lie'})
sys.states.add('X6_stand', ap={'stand'})
sys.states.add('X8_jump', ap={'jump'})
sys.states.add('X6', ap={'goal'})

# if IPython and Matplotlib available
# sys.plot()

# @environ_section@
env_vars = {}
env_safe = set()
env_init = {}
env_prog = set()

# System specification
#
# The system specification is that the quadruped should reach the goal
# and show the motion primitives jump,lie, and stand before reaching goal
#     []<> goal && !goal U jump

# @specs_setup_section@
sys_vars = set()        # infer the rest from TS
sys_init = set()
sys_prog = {'goal'}             # []<>goal (try with jump and other primitives for now)
sys_safe = {'goal -> X(goal)'}  # stay at the goal once it is reached


# Create the specification
specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                    env_safe, sys_safe, env_prog, sys_prog)
print(specs.pretty())

specs.moore = True
specs.qinit = '\E \A'
if not synth.is_realizable(specs, solver='omega', sys=sys):
    print("Not realizable.")
    st()
else:
    ctrl = synth.synthesize(specs, solver='omega', sys=sys)

# Generate a graphical representation of the controller for viewing,
# or a textual representation if pydot is missing.
# @plot_print@
if not ctrl.save('quadruped_test_controller.png'):
    print(ctrl)
# @plot_print_end@

# save the controller to a file
dumpsmach.write_python_case("quadruped_test_controller.py", ctrl, classname="QuadrupedCTRL")
