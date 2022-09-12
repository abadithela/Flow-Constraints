# script to interface with the quadruped
from ipdb import set_trace as st

MAP = {'init': (0,1), 'p1': (1,0), 'p2': (1,1), 'p3': (1,2), 'jump1': 'jump', 'stand1': 'stand', \
    'stand2': 'stand', 'stand3': 'stand', 'lie3': 'lie',\
    'd1_j': (1,0), 'd1_s': (1,0), 'd2_s': (1,1), 'd3_s': (1,2), 'd3_l': (1,2), 'goal': (2,1)} # to be modified for actual hardware implementation

def quadruped_move(move_to):
    print('Quadruped move to {}'.format(MAP[move_to]))
    st()
    # call the quadruped move command here
    # qudruped move / execute motion primitive
    # once quadruped finished the move continue
