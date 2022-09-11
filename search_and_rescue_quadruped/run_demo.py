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
    states = ['init', 'd1', 'd2', 'd3', 'goal']
    transitions = [('init', 'd1'), ('init', 'd2'), ('init', 'd3'), ('d1', 'd2'), ('d2', 'd3'), ('d2', 'd1'), ('d3', 'd2'), ('d1', 'goal'), ('d2', 'goal'), ('d3', 'goal')]
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
