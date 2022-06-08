import sys
sys.path.append('..')
import numpy as np
from ipdb import set_trace as st
from collections import OrderedDict as od
import _pickle as pickle
import os
import networkx as nx
import gurobipy as gp
from gurobipy import GRB
from network import MazeNetwork, create_network_from_file
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

def plot_maze(maze):
    tilesize = 1
    xs = np.linspace(0, 10*tilesize, 10+1)
    ys = np.linspace(0, 6*tilesize, 6+1)
    ax = plt.gca()
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False)
    # grid "shades" (boxes)
    w, h = xs[1] - xs[0], ys[1] - ys[0]
    for i, x in enumerate(xs[:-1]):
        for j, y in enumerate(ys[:-1]):
            if maze.map[j,i]=='*':
                ax.add_patch(Rectangle((x, y), w, h, fill=True, color='black', alpha=.5))
            elif j == 2 and i == 2 :
                ax.add_patch(Rectangle((x, y), w, h, fill=True, color='blue', alpha=.2))
            elif maze.map[j,i]=='':
                ax.add_patch(Rectangle((x, y), w, h, fill=True, color='black', alpha=.5))
            elif i % 2 == j % 2: # racing flag style
                ax.add_patch(Rectangle((x, y), w, h, fill=True, color='gray', alpha=.1))
    # grid lines
    for x in xs:
        plt.plot([x, x], [ys[0], ys[-1]], color='black', alpha=.33, linestyle=':')
    for y in ys:
        plt.plot([xs[0], xs[-1]], [y, y], color='black', alpha=.33, linestyle=':')

    ax.invert_yaxis()
    plt.show()

def plot_flow(maze, flow , colorstr):
    tilesize = 1
    xs = np.linspace(0, maze.len_y*tilesize, maze.len_y+1)
    ys = np.linspace(0, maze.len_x*tilesize, maze.len_x+1)
    ax = plt.gca()
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False)
    # grid "shades" (boxes)
    w, h = xs[1] - xs[0], ys[1] - ys[0]
    for i, x in enumerate(xs[:-1]):
        for j, y in enumerate(ys[:-1]):
            if maze.map[j,i]=='*': # racing flag style
                ax.add_patch(Rectangle((x, y), w, h, fill=True, color='black', alpha=.5))
            if maze.map[j,i]=='o': # racing flag style
                ax.add_patch(Rectangle((x, y), w, h, fill=True, color='red', alpha=.5))
            elif maze.map[j,i]=='': # racing flag style
                ax.add_patch(Rectangle((x, y), w, h, fill=True, color='black', alpha=.5))
            elif i % 2 == j % 2: # racing flag style
                ax.add_patch(Rectangle((x, y), w, h, fill=True, color='gray', alpha=.1))
    # grid lines
    for x in xs:
        plt.plot([x, x], [ys[0], ys[-1]], color='black', alpha=.33, linestyle=':')
    for y in ys:
        plt.plot([xs[0], xs[-1]], [y, y], color='black', alpha=.33, linestyle=':')

    # for item in flow:
    #     if flow[item].x > 0.5:
    #         startxy = item[0]
    #         endxy = item[1]
    #         plt.plot([startxy[1]+ tilesize/2, endxy[1]+ tilesize/2], [startxy[0]+ tilesize/2, endxy[0]+ tilesize/2], color='red', alpha=.33, linestyle='-')
    for item in flow:
        if flow[item] > 0.5:
            startxy = item[0]
            endxy = item[1]
            plt.plot([startxy[1]+ tilesize/2, endxy[1]+ tilesize/2], [startxy[0]+ tilesize/2, endxy[0]+ tilesize/2], color=colorstr, alpha=.33, linestyle='-')

    ax.invert_yaxis()
    plt.show()
