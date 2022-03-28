import sys
sys.path.append('..')
import numpy as np
from ipdb import set_trace as st
from collections import OrderedDict as od
from omega.symbolic import temporal as trl
import _pickle as pickle
import os
from omega.games import enumeration as enum
from omega.symbolic import enumeration as sym_enum
from omega.games import gr1
from omega.logic import syntax as stx
from omega.symbolic import fixpoint as fx
from omega.symbolic import fol as _fol
from omega.symbolic import prime as prm
from tulip.interfaces.omega import _grspec_to_automaton, _strategy_to_state_annotated
import logging
import pdb
from tulip import transys, spec, synth
from intersection.graph_construction import *

def synthesize_some_controller(aut):
    """Return a controller that implements the spec.
    If no controller exists,
    then raise an `Exception`.
    The returned controller is
    represented as a `networkx` graph.
    """
    z, yij, xijk = gr1.solve_streett_game(aut)
    gr1.make_streett_transducer(
        z, yij, xijk, aut)
    g = enum.action_to_steps(
        aut,
        env='env',
        sys='impl',
        qinit=aut.qinit)
    return g


def create_intersection_from_file(intersectionfile):
    map = od()
    f = open(intersectionfile, 'r')
    lines = f.readlines()
    len_y = len(lines)
    for i,line in enumerate(lines):
        for j,item in enumerate(line):
            if item != '\n':
                map[i,j] = item
    # make dictionary that maps each crosswalk state to a grid cell
    # currenly manual -> TODO crosswalk also automatically from file
    crosswalk = dict()
    start_cw = 1
    end_cw = 5
    y = 2
    for i, num in enumerate(range(2*start_cw,2*(end_cw+1))):
        crosswalk.update({i: (int(np.floor(num/2)), y)})
    # st()
    return map, crosswalk
