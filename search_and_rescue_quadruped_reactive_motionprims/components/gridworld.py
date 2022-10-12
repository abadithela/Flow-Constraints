from ipdb import set_trace as st
from copy import deepcopy
from find_constraints import find_cuts
import sys
sys.path.append('..')
from helpers.helper import load_opt_from_pkl_file
import _pickle as pickle
from components.quadruped_interface import quadruped_move

def load_optimization_results(network):
    # read pickle file - if not there save a new one
    try:
        print('Checking for the optimization results')
        G, node_dict, inv_node_dict, init, cuts, snr_to_nr, snr_to_label, label_to_snr = load_opt_from_pkl_file()
        print('Optimization results loaded successfully')
    except:
        print('Result file not found, running optimization')
        # st()
        G, node_dict, inv_node_dict, init, cuts, snr_to_nr, snr_to_label, label_to_snr = find_cuts(network)
        opt_dict = {'G': G, 'node_dict': node_dict, 'inv_node_dict':inv_node_dict, 'init': init, 'cuts': cuts, \
        'snr_to_nr':snr_to_nr, 'snr_to_label': snr_to_label, 'label_to_snr':label_to_snr}
        with open('stored_optimization_result.p', 'wb') as pckl_file:
            pickle.dump(opt_dict, pckl_file)
    return G, node_dict, inv_node_dict, init, cuts, snr_to_nr, snr_to_label, label_to_snr


class GridWorld:
    def __init__(self, maze, agent):
        self.maze = deepcopy(maze)
        self.orig_maze = maze
        self.agent = agent
        self.timestep = 0
        self.trace = []
        self.replanned = False

        self.G, self.node_dict, self.inv_node_dict, self.Ginit, self.cuts, self.snr_to_nr, self.snr_to_label, self.label_to_snr = load_optimization_results(maze)
        self.agent_in_state_x = self.Ginit[0]
        self.print_gridworld()
        # st()

    def print_gridworld(self):
        key_y_old = []
        # printline = ""
        for x in range(0,3):
            printline = ""
            for y in range(0,3):
                found_key = False
                for loc in self.maze.mapping.keys():
                    if self.maze.mapping[loc] == (x,y):
                        if self.agent.s == loc:
                            printline += 'A'
                        else:
                            printline += str(loc[0])
                        found_key = True
                if not found_key:
                    printline += ' '
            print(printline)

    def agent_take_step(self):
        x_old = self.agent.s
        self.agent.agent_move()
        # st()
        succs = self.G[self.agent_in_state_x]
        for node in succs:
            if self.node_dict[node][0] == 's'+str(self.maze.inv_map[self.agent.s]):
                self.agent_in_state_x = node
        if x_old != self.agent.s:
            self.replanned = False

        print('--- Agent in state {0}: {1}'.format(self.agent_in_state_x,self.node_dict[self.agent_in_state_x]))
        # st()
        quadruped_move(self.agent.s)


    def test_strategy(self):
        if not self.replanned:
            for cut in self.cuts:
                if self.agent_in_state_x == cut[0]:
                    cut_a = self.snr_to_label[self.node_dict[cut[0]][0]]
                    cut_b = self.snr_to_label[self.node_dict[cut[1]][0]]
                    self.drop_obstacle((cut_a,cut_b))
                    print('Door locked in location {}!'.format((cut_a,cut_b)))
                    self.agent.controller = self.agent.find_controller(self.maze, self.agent.s)
                    self.agent.state = 0
                    self.replanned = True


    def drop_obstacle(self, cut):
        # self.lift_obstacles()
        self.maze.add_cut(cut)

    def lift_obstacles(self):
        self.maze.remove_all_cuts()

    def is_terminal(self):
        terminal = False
        if self.agent.s in self.agent.goals:
            terminal = True
        return terminal
