import copy as cp
import sys
from ipdb import set_trace as st
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import glob
from PIL import Image
import _pickle as pickle
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator,
        FormatStrFormatter, AutoMinorLocator)
from matplotlib.collections import PatchCollection
import imageio

TILESIZE = 50

main_dir = os.path.dirname(os.path.dirname(os.path.realpath("__file__")))
pacman_fig = main_dir + '/CompositionalTesting/imglib/pacman_fig.png'

def draw_map(map, merge = False):
    # st()
    size = max(map.keys())
    x_min = 0
    x_max = (size[0]+1) * TILESIZE
    y_min = 0
    y_max = (size[1]+1) * TILESIZE
    #x_min, x_max, y_min, y_max = get_map_corners(map)
    ax.axis('equal')
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False)

    # fill in the road regions
    road_tiles = []
    width_tiles = np.arange(0,size[0]+1)*TILESIZE
    lanes_tiles = np.arange(0,size[1]+1)*TILESIZE

    for i in np.arange(0,size[0]+1):
        for k in np.arange(0,size[1]+1):
            if map[(i,k)] != '*':
                tile = patches.Rectangle((width_tiles[k],lanes_tiles[i]),TILESIZE,TILESIZE,linewidth=1,facecolor='k', alpha=0.4)
                road_tiles.append(tile)
    ax.add_collection(PatchCollection(road_tiles, match_original=True))

    plt.gca().invert_yaxis()

def draw_timestamp(t, merge = False):
    if merge:
        ax.text(0.5,0.7,t, transform=plt.gcf().transFigure,fontsize='large',
             bbox={"boxstyle" : "circle", "color":"white", "ec":"black"})
    else:
        ax.text(0.3,0.7,t, transform=plt.gcf().transFigure,fontsize='large',
             bbox={"boxstyle" : "circle", "color":"white", "ec":"black"})

def draw_pacman(pac_data, merge = False):
    y_tile, x_tile = car_data
    # theta_d = ORIENTATIONS[orientation]
    x = (x_tile) * TILESIZE
    y = (y_tile) * TILESIZE
    car_fig = Image.open(pacman_fig)
    # car_fig = car_fig.rotate(theta_d, expand=False)
    offset = 0.1
    ax.imshow(car_fig, zorder=1, interpolation='bilinear', extent=[x+2, x+TILESIZE-2, y+2, y+TILESIZE-2])


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
        st()
        traces = pickle.load(pckl_file)
    ##
    t_start = 0
    t_end = len(traces)
    # t_start = traces[0].timestamp
    # t_end = traces[-1].timestamp
    map = traces[0].map
    #import pdb; pdb.set_trace()
    global ax
    fig, ax = plt.subplots()

    t_array = np.arange(t_end)
    # plot map once
    for t in t_array:
        plt.gca().cla()
        draw_map(map)
        pac_data = traces[t].pac
        draw_pacman(pac_data)
        plot_name = str(t).zfill(5)
        img_name = output_dir+'/plot_'+plot_name+'.png'
        fig.savefig(img_name)
    animate_images(output_dir)

def make_animation():
    output_dir = os.getcwd()+'/animations/gifs/'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    traces_file = os.getcwd()+'/static_obstacle_maze/saved_traces/sim_trace.p'
    traces_to_animation(traces_file, output_dir)

if __name__ == '__main__':
    make_animation()
