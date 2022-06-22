# Simulate Road Network Example
# J. Graebener
# June 2022
# Constrained Test Environments for GR(1) Specifications

import sys
sys.path.append('..')
from random import choice
import numpy as np
from components.agent import Agent
from components.road_network import RoadNetwork
from components.gridworld import GridWorld
import os
from ipdb import set_trace as st
from helpers.helper import *

def new_World():
    networkfile = os.getcwd()+'/road_network.txt'
    network = RoadNetwork(networkfile)
    sys = Agent('sys',(5,5), (4,0), network)
    gridworld = GridWorld(network, sys)
    return gridworld, network, sys


def run_sim(max_timestep, filepath):
    trace=[]
    gridworld, maze, sys = new_World()
    gridworld.print_gridworld()
    # st()
    trace = save_scene(gridworld,trace)
    for t in range(1,max_timestep):
        print('Timestep {}'.format(t))
        gridworld.agent_take_step()
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
