import sys
sys.path.append('..')
from components.scene import Scene
import _pickle as pickle
import os
from ipdb import set_trace as st


def save_trace(filename,trace): # save the trace in pickle file for animation
    print('Saving trace in pkl file')
    with open(filename, 'wb') as pckl_file:
        pickle.dump(trace, pckl_file)

def save_scene(gridworld,trace): # save each scene in trace
    print('Saving scene {}'.format(gridworld.timestep))
    sys_snapshot = []
    sys_snapshot.append((gridworld.agent.name,gridworld.agent.s))
    current_scene = Scene(gridworld.timestep, gridworld.maze, sys_snapshot)
    trace.append(current_scene)
    gridworld.timestep += 1
    gridworld.trace = trace
    return trace

def load_opt_from_pkl_file():
    opt_file = os.getcwd()+'/stored_optimization_result.p'
    with open(opt_file, 'rb') as pckl_file:
        opt = pickle.load(pckl_file)
    G = opt['G']
    node_dict = opt['node_dict']
    inv_node_dict = opt['inv_node_dict']
    init = opt['init']
    cuts = opt['cuts']
    snr_to_nr = opt['snr_to_nr']
    snr_to_label = opt['snr_to_label']
    label_to_snr = opt['label_to_snr']
    return G, node_dict, inv_node_dict, init, cuts, snr_to_nr, snr_to_label, label_to_snr
