from ipdb import set_trace as st
from copy import deepcopy
from find_constraints import find_cuts


class GridWorld:
    def __init__(self, maze, agent):
        self.maze = deepcopy(maze)
        self.orig_maze = maze
        self.agent = agent
        self.timestep = 0
        self.trace = []
        self.replanned = False
        self.G, self.node_dict, self.inv_node_dict, self.Ginit, self.cuts, self.snr_to_nr, self.snr_to_label, self.label_to_snr = find_cuts()
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
        # st()


    def test_strategy(self):
        if not self.replanned:
            for cut in self.cuts:
                if self.agent_in_state_x == cut[0]:
                    cut_a = self.snr_to_label[self.node_dict[cut[0]][0]]
                    cut_b = self.snr_to_label[self.node_dict[cut[1]][0]]
                    self.drop_obstacle((cut_a,cut_b))
                    print('Obstacle placed!')
                    self.agent.controller = self.agent.find_controller(self.maze, self.agent.s)
                    self.agent.state = 0
                    self.replanned = True


    def example_test_strategy(self):
        if not self.replanned:
            if (self.agent.y,self.agent.x) == self.maze.key1 and not self.maze.goal2_unlocked:
                self.lift_obstacles()
                self.drop_obstacle((0,7))
                self.agent.controller = self.agent.find_controller(self.maze)
                self.agent.state = 0
                self.replanned = True
            elif (self.agent.y,self.agent.x) == self.maze.key2 and not self.maze.goal1_unlocked:
                self.lift_obstacles()
                self.drop_obstacle((0,1))
                self.agent.controller = self.agent.find_controller(self.maze)
                self.agent.state = 0
                self.replanned = True
            # elif self.maze.goal1_unlocked and self.maze.goal2_unlocked:
            #     self.lift_obstacles()


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
