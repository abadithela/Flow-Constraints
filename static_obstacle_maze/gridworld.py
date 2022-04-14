from ipdb import set_trace as st

class GridWorld:
    def __init__(self, maze, agent):
        self.maze = maze
        self.agent = agent
        self.timestep = 0
        self.trace = []

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
        self.agent.agent_move()
        # if (self.agent.y,self.agent.x + 1) in self.maze.map:
        #     if self.maze.map[(self.agent.y,self.agent.x + 1)] != '*' and self.maze.map[(self.agent.y,self.agent.x + 1)] != 'o':
        #         self.agent.step_e()
        #     else:
        #         self.agent.step_n()
        # else:
        #     self.agent.step_n()

    def is_terminal(self):
        terminal = False
        if (self.agent.y,self.agent.x) == self.agent.goal:
            terminal = True
        return terminal
