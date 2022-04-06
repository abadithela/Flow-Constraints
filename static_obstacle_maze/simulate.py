import sys
sys.path.append('..')
from random import choice
import numpy as np
from agent import Agent
from network import MazeNetwork
from gridworld import GridWorld
import os
from ipdb import set_trace as st
from helper import *


def new_World():
    mazefile = os.getcwd()+'/maze.txt'
    maze = MazeNetwork(mazefile)
    pacman = Agent('pacman',5,0,(0,9))
    gridworld = GridWorld(maze, pacman)
    return gridworld, maze, pacman


def run_sim(max_timestep, filepath):
    trace=[]
    gridworld, maze, pacman = new_World()
    gridworld.print_gridworld()
    st()
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
