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
from components.road_network import MazeNetwork, create_network_from_file
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

def plot_mcf(maze, flow1, flow2, flow3, cuts):
    tilesize = 1
    xs = np.linspace(0, maze.len_x*tilesize, maze.len_x+1)
    ys = np.linspace(0, maze.len_y*tilesize, maze.len_y+1)
    interm = maze.intermediate
    # st()

    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
    axis = [ax1, ax2, ax3, ax4]
    flows = [flow1, flow2, flow3]
    colorstr = ['red', 'blue', 'green']
    titlestr = ['Flow 1', 'Flow 2', 'Flow 3', 'Cuts']

    for idx in range(0,4):
    # ax = plt.gca()
        axis[idx].xaxis.set_visible(False)
        axis[idx].yaxis.set_visible(False)
        # grid "shades" (boxes)
        w, h = xs[1] - xs[0], ys[1] - ys[0]
        for i, x in enumerate(xs[:-1]):
            for j, y in enumerate(ys[:-1]):
                # st()
                if maze.map[j,i]=='*': # racing flag style
                    axis[idx].add_patch(Rectangle((x, y), w, h, fill=True, color='black', alpha=.5))
                elif j == interm[0] and i == interm[1] :
                    axis[idx].add_patch(Rectangle((x, y), w, h, fill=True, color='blue', alpha=.2))
                    axis[idx].text(x+tilesize/2-tilesize/5, y+tilesize/2-tilesize/10, 'I')
                elif maze.map[j,i]=='o': # racing flag style
                    axis[idx].add_patch(Rectangle((x, y), w, h, fill=True, color='red', alpha=.5))
                elif maze.map[j,i]=='↓' or maze.map[j,i]=='↑' or maze.map[j,i]=='←' or maze.map[j,i]=='→' or maze.map[j,i]=='+': # racing flag style
                    axis[idx].add_patch(Rectangle((x, y), w, h, fill=True, color='black', alpha=.1))
                elif i % 2 == j % 2: # racing flag style
                    axis[idx].add_patch(Rectangle((x, y), w, h, fill=True, color='gray', alpha=.1))
                if j == maze.source[0] and i == maze.source[1]:
                    # st()
                    axis[idx].text(x+tilesize/2-tilesize/5, y+tilesize/2-tilesize/10, 'S')
                if j == maze.goal[0] and i == maze.goal[1]:
                    # st()
                    axis[idx].text(x+tilesize/2-tilesize/5, y+tilesize/2-tilesize/10, 'T')
        # grid lines
        for x in xs:
            axis[idx].plot([x, x], [ys[0], ys[-1]], color='black', alpha=.33, linestyle=':')
        for y in ys:
            axis[idx].plot([xs[0], xs[-1]], [y, y], color='black', alpha=.33, linestyle=':')

        # for item in flow:
        #     if flow[item].x > 0.5:
        #         startxy = item[0]
        #         endxy = item[1]
        #         plt.plot([startxy[1]+ tilesize/2, endxy[1]+ tilesize/2], [startxy[0]+ tilesize/2, endxy[0]+ tilesize/2], color='red', alpha=.33, linestyle='-')
        if idx == 3:
            for cut in cuts:
                # st()
                startxy = cut[0]
                endxy = cut[1]
                x_val = (startxy[0]+endxy[0])/2
                y_val = (startxy[1]+endxy[1])/2
                if cuts[cut] <= 1.0:
                    intensity = max(0,cuts[cut])/2
                else:
                    intensity = 1.0/2
                axis[idx].plot([y_val+ tilesize/2, y_val+ tilesize/2], [x_val+ tilesize/2, x_val+ tilesize/2], color='black', alpha=intensity, marker='o')

        else:
            for flow in flows[idx]:
                startxy = flow[0]
                endxy = flow[1]
                if flows[idx][flow] <= 1.0:
                    intensity = max(0,flows[idx][flow])/2
                else:
                    intensity = 1.0/2
                axis[idx].plot([startxy[1]+ tilesize/2, endxy[1]+ tilesize/2], [startxy[0]+ tilesize/2, endxy[0]+ tilesize/2], color=colorstr[idx], alpha=intensity, linestyle='-')

        axis[idx].invert_yaxis()
        axis[idx].set_title(titlestr[idx])
    plt.show()

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
        # if flow[item] > 0.5:
        startxy = item[0]
        endxy = item[1]
        if flow[item] <= 1.0:
            intensity = max(0,flow[item])/2
        else:
            intensity = 1.0/2
        plt.plot([startxy[1]+ tilesize/2, endxy[1]+ tilesize/2], [startxy[0]+ tilesize/2, endxy[0]+ tilesize/2], color=colorstr, alpha=intensity, linestyle='-')

    ax.invert_yaxis()
    plt.show()
