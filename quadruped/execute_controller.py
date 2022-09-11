'''
Skeleton script to call the tulip controller for the quadruped.
'''
from ipdb import set_trace as st

# Here add the map (Gridworld states -> waypoints or actions)
map = {'X0': (0,0), 'X1': (1,0), 'X2': (0,2), 'X3': (1,0),'X4': (1,1),\
       'X5': (1,2),'X6': (2,0),'X7': (2,1),'X8': (2,2), \
       'X2_lie': 'lie', 'X8_jump': 'jump'}

# load the controller from file
from quadruped_test_controller import QuadrupedCTRL
M = QuadrupedCTRL()

# use this to call the next move starting from the initial position X0
t = 5
for i in range(0,t):
    output = M.move()
    next_state = map[output['loc']]
    print(next_state)
    st()
