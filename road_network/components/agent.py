from __future__ import print_function
import logging
from tulip import transys, spec, synth
from tulip import dumpsmach
import tulip.gridworld as gw
from ipdb import set_trace as st

class Agent:
    def __init__(self, name, pos, goal, maze):
        self.name = name
        self.x = pos[0]
        self.y = pos[1]
        self.goal = goal
        self.controller = self.find_controller(maze)

    def find_controller(self,maze):
        G = maze.gamegraph
        map = maze.map
        logging.basicConfig(level=logging.WARNING)
        logging.getLogger('tulip.spec.lexyacc').setLevel(logging.WARNING)
        logging.getLogger('tulip.synth').setLevel(logging.WARNING)
        logging.getLogger('tulip.interfaces.omega').setLevel(logging.WARNING)

        # with open("tulip_maze.txt", "r") as f:
        #     maze_grid = gw.GridWorld(f.read(), prefix="Y")
        # print(maze_grid)

        # dynamic_specs = maze.transition_specs()

        # spc = maze_grid.spec()
        sys_vars = {}
        sys_vars['X_h'] = (0, maze.len_x-1)
        sys_vars['X_v'] = (0, maze.len_y-1)
        sys_init = {'X_h='+str(self.x)+' && X_v='+str(self.y)}
        sys_prog = set()
        sys_prog |= {'X_h='+str(self.goal[1])+' && X_v='+str(self.goal[0])}
        sys_safe = set()
        # add the dynamics for the system
        sys_safe |= maze.transition_specs('X_h','X_v')

        env_vars = {}
        env_safe = set()
        env_init = {}
        env_prog = set()


        spc = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                        env_safe, sys_safe, env_prog, sys_prog)

        print(spc.pretty())
        # st()
        spc.moore = False
        spc.qinit = r'\A \E'
        if not synth.is_realizable(spc, solver='omega'):
            print("Not realizable.")
        else:
            ctrl = synth.synthesize(spc, solver='omega')
        # dunp the controller
        dumpsmach.write_python_case("car_controller.py", ctrl, classname="AgentCtrl")
        # load the controller
        from car_controller import AgentCtrl
        M = AgentCtrl()
        return M

    def agent_move(self):
        # st()
        output = self.controller.move()
        self.x = output['X_h']
        self.y = output['X_v']


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
