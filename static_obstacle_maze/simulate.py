import sys
sys.path.append('..')
from random import choice
import numpy as np
from agent import Agent
from network import MazeNetwork
from gridworld import GridWorld
import os
from ipdb import set_trace as st


def new_World():
    mazefile = os.getcwd()+'/maze.txt'
    maze = MazeNetwork(mazefile)
    pacman = Agent('pacman',5,0,(0,9))
    gridworld = GridWorld(maze, pacman)
    return gridworld, maze, pacman


def run_sim(max_timestep):
    gridworld, maze, pacman = new_World()
    gridworld.print_gridworld()
    for t in range(1,max_timestep):
        print('Timestep {}'.format(t))
        gridworld.agent_take_step()
        gridworld.print_gridworld()
        if gridworld.is_terminal():
            break


if __name__ == '__main__':
    max_timestep = 50
    run_sim(max_timestep)
