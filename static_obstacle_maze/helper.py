import sys
sys.path.append('..')
from scene import Scene
import _pickle as pickle
import os
from ipdb import set_trace as st


def save_trace(filename,trace): # save the trace in pickle file for animation
    print('Saving trace in pkl file')
    with open(filename, 'wb') as pckl_file:
        pickle.dump(trace, pckl_file)

def save_scene(gridworld,trace): # save each scene in trace
    print('Saving scene {}'.format(gridworld.timestep))
    pac_snapshot = []
    pac_snapshot.append((gridworld.agent.name,gridworld.agent.x,gridworld.agent.y))
    current_scene = Scene(gridworld.timestep, gridworld.maze, pac_snapshot)
    trace.append(current_scene)
    gridworld.timestep += 1
    gridworld.trace = trace
    return trace
