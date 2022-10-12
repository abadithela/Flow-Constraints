# Simulated Robot Search and Rescue Example
# J. Graebener
# September 2022
# Synthesizing Test Environments for LTL Specifications

import sys
sys.path.append('..')
from random import choice
import numpy as np
from components.agent import Agent
from components.network import CustomGrid
from components.gridworld import GridWorld
import os
from ipdb import set_trace as st
from helpers.helper import *


def new_World():

    states = ['init', 'p1', 'p2', 'p3', 'jump1', 'stand1', 'stand2', 'stand3', 'lie3',\
    'd1_j', 'd1_s', 'd2_s', 'd3_s', 'd3_l', 'goal']
    transitions = [('init', 'p1'), ('init', 'p2'), ('init', 'p3'), \
    ('p1', 'jump1'), ('p1', 'stand1'), ('p2', 'stand2'), ('p3', 'stand3'), ('p3', 'lie3'), \
    ('jump1', 'd1_j'),('stand1', 'd1_s'), ('stand2', 'd2_s'), ('stand3', 'd3_s'), ('lie3', 'd3_l'), \
    ('d1_s', 'p1'), ('d1_j', 'p1'),\
    ('d2_s', 'p2'), \
    ('d3_s', 'p3'), ('d3_l', 'p3'),\
    ('p2', 'p1'), ('p3', 'p2'), ('p1', 'p2'), ('p2', 'p3'), \
    ('d1_s', 'goal'), ('d1_j', 'goal'), ('d2_s', 'goal'), ('d3_s', 'goal'),  ('d3_l', 'goal')]
    network = CustomGrid(states, transitions)
    # st()
    sys = Agent('sys','init',['goal'], network)
    gridworld = GridWorld(network, sys)
    return gridworld, network, sys

def run_sim(max_timestep, filepath):
    trace=[]
    gridworld, maze, sys = new_World()
    gridworld.print_gridworld()
    trace = save_scene(gridworld,trace)
    for t in range(1,max_timestep):
        print('Timestep {}'.format(t))
        gridworld.agent_take_step()
        gridworld.test_strategy()
        gridworld.print_gridworld()
        # save the trace
        trace = save_scene(gridworld,trace)
        if gridworld.is_terminal():
            break
    save_trace(filepath, gridworld.trace)


if __name__ == '__main__':
    max_timestep = 50
    output_dir = os.getcwd()+'/saved_traces/'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    filename = 'sim_trace.p'
    filepath = output_dir + filename
    run_sim(max_timestep, filepath)
