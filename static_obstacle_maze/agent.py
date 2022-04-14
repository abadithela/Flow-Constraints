from __future__ import print_function
import logging
from tulip import transys, spec, synth
from tulip import dumpsmach
import tulip.gridworld as gw
from ipdb import set_trace as st

class Agent:
    def __init__(self, name, y, x, goal, maze):
        self.name = name
        self.x = x
        self.y = y
        self.goal = goal
        self.controller = self.find_controller(maze)

    def find_controller(self,maze):
        G = maze.gamegraph
        map = maze.map
        logging.basicConfig(level=logging.WARNING)
        logging.getLogger('tulip.spec.lexyacc').setLevel(logging.WARNING)
        logging.getLogger('tulip.synth').setLevel(logging.WARNING)
        logging.getLogger('tulip.interfaces.omega').setLevel(logging.WARNING)

        with open("tulip_maze.txt", "r") as f:
            maze_grid = gw.GridWorld(f.read(), prefix="Y")
        print(maze_grid)

        spc = maze_grid.spec()
        # print(spc.pretty())
        spc.moore = False
        spc.qinit = r'\A \E'
        if not synth.is_realizable(spc, solver='omega'):
            print("Not realizable.")
        else:
            ctrl = synth.synthesize(spc, solver='omega')
        # dunp the controller
        dumpsmach.write_python_case("mazecontroller.py", ctrl, classname="AgentCtrl")
        # load the controller
        from mazecontroller import AgentCtrl
        M = AgentCtrl()
        return M

    def agent_move(self):
        output = self.controller.move()
        self.x = output['Y_c']
        self.y = output['Y_r']


    def step_n(self):
        self.y = self.y - 1

    def step_e(self):
        self.x = self.x + 1

    def step_s(self):
        self.y = self.y + 1

    def step_w(self):
        self.x = self.x - 1

    def step_stay(self):
        pass
