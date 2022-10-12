# script to interface with the quadruped
from ipdb import set_trace as st

MAP = {'init': (0,1), 'd1': (1,0), 'd2': (1,1), 'd3': (1,2), 'int_goal': (3,1), \
        's1': (2,0), 's2': (2,1), 's3': (2,2), 'goal': (0,1)} # to be modified for actual hardware implementation

def quadruped_move(move_to):
    print('Quadruped move to {}'.format(MAP[move_to]))
    st()
    # call the quadruped move command here
    # qudruped move / execute motion primitive
    # once quadruped finished the move continue
