import copy as cp
import sys
from ipdb import set_trace as st
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import glob
from PIL import Image, ImageOps
import _pickle as pickle
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator,
        FormatStrFormatter, AutoMinorLocator)
from matplotlib.collections import PatchCollection
import imageio

TILESIZE = 50
GRID_LINES = False

main_dir = os.path.dirname(os.path.dirname(os.path.realpath("__file__")))
car_figure = main_dir + '/imglib/robot.png'

def draw_maze(orig_maze, merge = False):
    z_min = 0
    z_max = 3 * TILESIZE
    x_min = 0
    x_max = 3 * TILESIZE

    ax.set_xlim(x_min, x_max)
    ax.set_ylim(z_min, z_max)
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False)

    # draw_the_3_door_example
    tiles = []

    tiles.append(patches.Rectangle((0,0),TILESIZE*3,TILESIZE,linewidth=1,edgecolor='black',facecolor='lightgray'))
    tiles.append(patches.Rectangle((0,1*TILESIZE),TILESIZE,TILESIZE,linewidth=1,edgecolor='black',facecolor='#648fff'))
    tiles.append(patches.Rectangle((1*TILESIZE,1*TILESIZE),TILESIZE,TILESIZE,linewidth=1,edgecolor='black',facecolor='#648fff'))
    tiles.append(patches.Rectangle((2*TILESIZE,1*TILESIZE),TILESIZE,TILESIZE,linewidth=1,edgecolor='black',facecolor='#648fff'))
    tiles.append(patches.Rectangle((0,2*TILESIZE),TILESIZE*3,TILESIZE,linewidth=1,edgecolor='black',facecolor='#ffb000'))

    ax.add_collection(PatchCollection(tiles, match_original=True))


def draw_cuts(maze):
    wedges = []
    for cut in maze.cuts:
        cut_loc = maze.mapping[cut]
        wedge = patches.Wedge(((cut_loc[1]+0.5)*TILESIZE,(cut_loc[0]+1)*TILESIZE),TILESIZE/2-5,0, 180, facecolor = 'black', alpha = 0.8)
        wedges.append(wedge)
    ax.add_collection(PatchCollection(wedges, match_original=True))


def draw_timestamp(t, merge = False):
    if merge:
        ax.text(0.5,0.7,t, transform=plt.gcf().transFigure,fontsize='large',
             bbox={"boxstyle" : "circle", "color":"white", "ec":"black"})
    else:
        ax.text(0.3,0.7,t, transform=plt.gcf().transFigure,fontsize='large',
             bbox={"boxstyle" : "circle", "color":"white", "ec":"black"})

def draw_car(pac_data, theta_d, merge = False):
    y_tile = pac_data[0][1]
    x_tile = pac_data[0][2]
    # theta_d = ORIENTATIONS[orientation]
    x = (x_tile) * TILESIZE
    z = (y_tile) * TILESIZE
    car_fig = Image.open(car_figure)
    car_fig = ImageOps.flip(car_fig)
    car_fig = car_fig.rotate(theta_d, expand=False)
    offset = 0.1
    ax.imshow(car_fig, zorder=1, interpolation='bilinear', extent=[z+10, z+TILESIZE-10, x+5, x+TILESIZE-5])

def draw_robot(robot_data, maze, theta_d):
    robot_loc = maze.mapping[robot_data[-1][-1]]
    x = robot_loc[0] * TILESIZE
    z = robot_loc[1] * TILESIZE
    rob_fig = Image.open(car_figure)
    rob_fig = ImageOps.flip(rob_fig)
    rob_fig = rob_fig.rotate(theta_d, expand=False)
    offset = 0.1
    ax.imshow(rob_fig, zorder=1, interpolation='bilinear', extent=[z+10, z+TILESIZE-10, x+5, x+TILESIZE-5])


def animate_images(output_dir):
    # Create the frames
    frames = []
    imgs = glob.glob(output_dir+'plot_'"*.png")
    imgs.sort()
    for i in imgs:
        new_frame = Image.open(i)
        frames.append(new_frame)

    # Save into a GIF file that loops forever
    frames[0].save(output_dir + 'png_to_gif.gif', format='GIF',
            append_images=frames[1:],
            save_all=True,
            duration=200, loop=3)

def traces_to_animation(filename, output_dir):
    # extract out traces from pickle file
    with open(filename, 'rb') as pckl_file:
        # st()
        traces = pickle.load(pckl_file)
    ##
    # st()
    t_start = 0
    t_end = len(traces)
    # t_start = traces[0].timestamp
    # t_end = traces[-1].timestamp
    maze = traces[0].maze
    global ax
    fig, ax = plt.subplots()

    t_array = np.arange(t_end)
    # plot map once
    for t in t_array:
        plt.gca().cla()
        draw_maze(traces[t].maze)
        draw_cuts(maze)
        robot_data = traces[t].agent
        theta_d = 180
        draw_robot(robot_data, traces[t].maze, theta_d)
        plot_name = str(t).zfill(5)
        img_name = output_dir+'/plot_'+plot_name+'.png'
        fig.savefig(img_name, dpi=1200)
    animate_images(output_dir)

def angle(traces, t):
    # st()
    map = traces[0].maze.map
    car_data = traces[t].car
    y_tile = car_data[0][1]
    x_tile = car_data[0][2]
    dir = map[(y_tile,x_tile)]

    dir_dict = {'→' : 0, '←' : 180, '↑': 270, '↓': 90, '*': 0, ' ': 0}
    # st()
    if dir != '+':
        angle = dir_dict[dir]
    else:
        if map[(traces[t-1].car[0][2],traces[t-1].car[0][1])] != '+':
            angle = dir_dict[map[(traces[t-1].car[0][2],traces[t-1].car[0][1])]]
        elif map[(traces[t+1].car[0][2],traces[t+1].car[0][1])] != '+':
            angle = dir_dict[map[(traces[t+1].car[0][2],traces[t+1].car[0][1])]]
        elif map[(traces[t+2].car[0][2],traces[t+2].car[0][1])] != '+':
            angle = dir_dict[map[(traces[t+2].car[0][2],traces[t+2].car[0][1])]]
        elif map[(traces[t-2].car[0][2],traces[t-2].car[0][1])] != '+':
            angle = dir_dict[map[(traces[t-2].car[0][2],traces[t-2].car[0][1])]]

    return angle




def make_animation():
    output_dir = os.getcwd()+'/animations/gifs/'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    traces_file = os.getcwd()+'/saved_traces/sim_trace.p'
    traces_to_animation(traces_file, output_dir)

if __name__ == '__main__':
    make_animation()
    # st()
