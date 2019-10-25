#!/usr/bin/env python

# File: t5.1-collision-check.py
# =======================
#
# Collision Checking

from amino import SceneWin, SceneGraph, SceneFK, SceneCollisionSet, SceneCollision
from math import pi, cos, sin
from time import sleep
import os

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

# Initial Configuration
config_dict = {
    "s1": -.75 * pi,
    "e": .75 * pi,
    "f0": .125 * pi,
    "f1": .125 * pi,
    "f2": .125 * pi
}

# Collision context and collision set
cl = SceneCollision(sg)
cl_set = SceneCollisionSet(sg)

# Allowable collisions
cl.allow_config(config_dict)

dt = 1.0 / 60
t = 0
while win.is_runnining():
    # wiggle fingers
    f = (30 + 30 * sin(t)) * (pi / 180)
    config_dict['f0'] = f
    config_dict['f1'] = f
    config_dict['f2'] = f

    # update window
    win.config = config_dict

    # update forward kinematics
    fk.config = config_dict

    # check collision
    is_collision = cl.check(fk, cl_set)

    # print collision results
    print("Collision: %s" % ('yes' if is_collision else 'no'))
    for i in range(0, sg.frame_count):
        for j in range(0, i):
            if cl_set[i, j]:
                print("    %s, %s" % (sg.ensure_frame_name(i),
                                      sg.ensure_frame_name(j)))

    # sleep till next cycle
    sleep(dt)
    t += dt
