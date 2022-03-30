class Agent:
    def __init__(self, name, y, x, goal):
        self.name = name
        self.x = x
        self.y = y
        self.goal = goal
        self.controller = None
    #
    # def take_step(self):
    #     action = self.controller[(self.y,self.x)]

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
