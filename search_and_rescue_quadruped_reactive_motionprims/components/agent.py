from __future__ import print_function
import logging
from tulip import transys, spec, synth
from tulip import dumpsmach
import tulip.gridworld as gw
from ipdb import set_trace as st

class Agent:
    def __init__(self, name, init, goals, network):
        self.name = name
        self.init = init
        self.goals = goals
        self.index = 0
        self.controller = self.find_controller(network, self.init)
        self.s = init
        self.network = network

    def find_controller(self,maze, init):
        print('------- (Re-)synthesizing the agent\'s controller -------')
        logging.basicConfig(level=logging.WARNING)
        logging.getLogger('tulip.spec.lexyacc').setLevel(logging.WARNING)
        logging.getLogger('tulip.synth').setLevel(logging.WARNING)
        logging.getLogger('tulip.interfaces.omega').setLevel(logging.WARNING)

        sys_vars = {}
        sys_vars['s'] = (0, len(maze.states))
        sys_init = {'s = '+str(maze.inv_map[init])}
        sys_prog = set()
        goalstr = '(s = '+str(maze.inv_map[self.goals[0]])+')'
        for goal in self.goals[1:]:
            goalstr += ' || (s = '+str(maze.inv_map[goal])+')'
        sys_prog = goalstr
        sys_safe = set()
        sys_safe |= maze.transition_specs('s') # add the dynamics for the system

        env_vars = {}
        env_safe = set()
        env_init = {}
        env_prog = set()

        spc = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                        env_safe, sys_safe, env_prog, sys_prog)

        print(spc.pretty())
        spc.moore = False
        spc.qinit = r'\A \E'
        if not synth.is_realizable(spc, solver='omega'):
            print("Not realizable.")
            st()
        else:
            ctrl = synth.synthesize(spc, solver='omega')
        # dunp the controller
        # controller_namestr = "robot_controller"+str(self.index)+".py"
        # dumpsmach.write_python_case(controller_namestr, ctrl, classname="AgentCtrl")
        # # load the controller
        # from controller_namestr import AgentCtrl
        # M = AgentCtrl()
        self.index += 1

        # print(dumpsmach.python_case(g, classname='AgentCtrl', start=))
        exe_globals = dict()
        exec(dumpsmach.python_case(ctrl, classname='AgentCtrl'), exe_globals)
        M = exe_globals['AgentCtrl']()  # previous line creates the class `AgentCtrl`
        print('------- Controller available -------')
        return M

    def agent_move(self):
        # st()
        output = self.controller.move()
        print('Agent moving to {}'.format(self.network.map[output['s']]))
        self.s = self.network.map[output['s']]
