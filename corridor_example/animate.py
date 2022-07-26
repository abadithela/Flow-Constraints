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
    maze = orig_maze.map
    size = max(maze.keys())
    z_min = 0
    z_max = (size[0]+1) * TILESIZE
    x_min = 0
    x_max = (size[1]+1) * TILESIZE

    ax.set_xlim(x_min, x_max)
    ax.set_ylim(z_min, z_max)
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False)
    # ax.crop((x_min, z_min, x_max, z_max))

    # fill in the road regions
    road_tiles = []
    x_tiles = np.arange(0,size[0]+2)*TILESIZE
    z_tiles = np.arange(0,size[1]+2)*TILESIZE
    for i in np.arange(0,size[0]+1):
        for k in np.arange(0,size[1]+1):
            # print('{0},{1}'.format(i,k))
            if maze[(i,k)] != '*' and maze[(i,k)] != 'o':
                if i == 0 and k == 2:
                    tile = patches.Rectangle((z_tiles[k],x_tiles[i]),TILESIZE,TILESIZE,linewidth=1,facecolor='#dc267f', alpha=0.2)
                    # road_tiles.append(tile)
                elif i == 0 and k == 8:
                    # st()
                    if orig_maze.goal2_unlocked:
                        tile = patches.Rectangle((z_tiles[k],x_tiles[i]),TILESIZE,TILESIZE,linewidth=1,facecolor='#dc267f', alpha=0.3)
                    else:
                        tile = patches.Rectangle((z_tiles[k],x_tiles[i]),TILESIZE,TILESIZE,linewidth=1,facecolor='gray', alpha=0.3)

                elif i == 0 and k == 6:
                    tile = patches.Rectangle((z_tiles[k],x_tiles[i]),TILESIZE,TILESIZE,linewidth=1,facecolor='#648fff', alpha=0.3)
                elif i == 0 and k == 0:
                    # st()
                    if orig_maze.goal1_unlocked:
                        tile = patches.Rectangle((z_tiles[k],x_tiles[i]),TILESIZE,TILESIZE,linewidth=1,facecolor='#648fff', alpha=0.3)
                    else:
                        tile = patches.Rectangle((z_tiles[k],x_tiles[i]),TILESIZE,TILESIZE,linewidth=1,facecolor='gray', alpha=0.3)

                else:
                    # ax.add_patch(Rectangle((x, y), w, h, fill=True, color='black', alpha=.5))
                    tile = patches.Rectangle((z_tiles[k],x_tiles[i]),TILESIZE,TILESIZE, fill=True, color='gray', alpha=.1)

                    # tile = patches.Rectangle((z_tiles[k],x_tiles[i]),TILESIZE,TILESIZE, color='black', alpha=.1)
                    # if i % 2 == k % 2: # racing flag style
                    #     # ax.add_patch(Rectangle((x, y), w, h, fill=True, color='gray', alpha=.1))
                    #     tile = patches.Rectangle((z_tiles[k],x_tiles[i]),TILESIZE,TILESIZE, fill=True, color='gray', alpha=.1)

                    # tile = patches.Rectangle((z_tiles[k],x_tiles[i]),TILESIZE,TILESIZE,linewidth=1,facecolor='k', alpha=0.4)
                    # tile = patches.Rectangle((z_tiles[k],x_tiles[i]),TILESIZE,TILESIZE, fill=True, color='gray', alpha=.1)
                road_tiles.append(tile)
            elif maze[(i,k)] == '*':
                tile = patches.Rectangle((z_tiles[k],x_tiles[i]),TILESIZE,TILESIZE,linewidth=1,facecolor='k', alpha=0.8)
                road_tiles.append(tile)
            # elif maze[(i,k)] == 'o':
            #     # tile = patches.Rectangle((z_tiles[k],x_tiles[i]),TILESIZE,TILESIZE,linewidth=1,facecolor='tomato', alpha=0.5)
            #     tile = patches.Rectangle((z_tiles[k],x_tiles[i]),TILESIZE,TILESIZE, fill=True, color='gray', alpha=.1)
            #     road_tiles.append(tile)
    ax.add_collection(PatchCollection(road_tiles, match_original=True))
    # grid lines
    if GRID_LINES:
        for z in z_tiles:
            plt.plot([z, z], [x_tiles[0], x_tiles[-1]], color='black', alpha=.33, linestyle=':')
        for x in x_tiles:
            plt.plot([z_tiles[0], z_tiles[-1]], [x, x], color='black', alpha=.33, linestyle=':')
    plt.gca().invert_yaxis()

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
    #import pdb; pdb.set_trace()
    global ax
    fig, ax = plt.subplots()

    t_array = np.arange(t_end)
    # plot map once
    for t in t_array:
        plt.gca().cla()
        draw_maze(traces[t].maze)
        car_data = traces[t].car
        theta_d = 0#angle(traces,t)
        draw_car(car_data, theta_d)
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
