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
        self.G, self.state_map, self.node_dict, self.inv_node_dict, self.init, self.cuts = find_cuts()
        self.agent_in_node_x = self.init[0]

    def print_gridworld(self):
        key_y_old = []
        printline = ""
        for key in self.maze.map:
            key_y_new = key[0]
            if key_y_new == key_y_old:
                # st()
                if key == (self.agent.y,self.agent.x):
                    printline += 'P'
                else:
                    printline += self.maze.map[key]
            else:
                print(printline)
                if key == (self.agent.y,self.agent.x):
                    printline = 'P'
                else:
                    printline = self.maze.map[key]
            key_y_old = key_y_new
        print(printline)
        printline = self.maze.map[key]

    def agent_take_step(self):
        # st()
        succ = self.G[self.agent_in_node_x]
        self.agent.agent_move()
        for node in succ:
            if self.state_map[self.node_dict[node][0]] == (self.agent.y,self.agent.x):
                self.agent_in_node_x = node
        # st()
        if (self.agent.y,self.agent.x) == self.maze.key1:
            self.maze.goal1_unlocked = True
        elif (self.agent.y,self.agent.x) == self.maze.key2:
            self.maze.goal2_unlocked = True
        # if (self.agent.y,self.agent.x + 1) in self.maze.map:
        #     if self.maze.map[(self.agent.y,self.agent.x + 1)] != '*' and self.maze.map[(self.agent.y,self.agent.x + 1)] != 'o':
        #         self.agent.step_e()
        #     else:
        #         self.agent.step_n()
        # else:
        #     self.agent.step_n()

    def test_strategy(self):
        if not self.replanned:
            for cut in self.cuts:
                if self.agent_in_node_x == cut[0]:
                    self.drop_obstacle(self.state_map[self.node_dict[cut[1]][0]])
                    print('Obstacle placed!')
                    self.agent.controller = self.agent.find_controller(self.maze)
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



    def drop_obstacle(self, loc):
        self.lift_obstacles()
        self.maze.map[loc] = '*'
        self.maze.gamegraph, self.maze.states, self.maze.next_state_dict = self.maze.get_gamegraph()

    def lift_obstacles(self):
        self.maze = deepcopy(self.orig_maze)

    def is_terminal(self):
        terminal = False
        if (self.agent.y,self.agent.x) in self.agent.goals:
            terminal = True
        return terminal
