#!/usr/bin/env python3

# File: t4.1-fk.py
# =======================
#
# Compute Forward Kinematics

from math import pi, cos
from time import sleep
import os
from amino import SceneWin, SceneGraph, SceneFK

# Scene Parameters
# Change scene_plugin based on your directory structure
scene_plugin = (
    "%s/git/amino/demo/tutorial/plugin/7dof/libscene.so" % os.environ['HOME'])
scene_name = "7dof"

# Create an (empty) scene graph
sg = SceneGraph()

# Load the scene plugin
sg.load(scene_plugin, scene_name)

# Initialize the scene graph
sg.init()

# Create a window, pass the scenegraph, and start
win = SceneWin(scenegraph=sg, start=True, background=True)

# Create the FK context
fk = SceneFK(sg)

# Do a simple wave
dt = 1.0 / 60
t = 0
config_dict = {
    "s1": -.75 * pi,
    "e": .75 * pi,
    "f0": .125 * pi,
    "f1": .125 * pi,
    "f2": .125 * pi
}
while win.is_runnining():
    # update forward kinematics
    config_dict['e'] = (120 + 15 * cos(t)) * (pi / 180)
    fk.config = config_dict

    # update window
    win.fk = fk

    # absolute hand pose is changing
    TF_g_h = fk["hand"]
    print("g->hand: %s" % TF_g_h)

    # Relative pose between elbow and hand does not change; there are
    # only fixed transforms and joints with unchanging configuration
    # between these two frames.
    TF_e_h = fk["e", "hand"]
    print("e->hand: %s" % TF_e_h)

    # sleep till next cycle
    sleep(dt)
    t += dt
