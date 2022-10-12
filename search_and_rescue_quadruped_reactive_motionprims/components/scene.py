from copy import deepcopy

class Scene:
    def __init__(self, timestamp, maze, agent):
        self.timestamp = timestamp
        self.maze = deepcopy(maze)
        self.agent = agent
