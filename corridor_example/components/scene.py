from copy import deepcopy

class Scene:
    def __init__(self, timestamp, maze, car):
        self.timestamp = timestamp
        self.maze = deepcopy(maze)
        self.car = car
