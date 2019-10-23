#!/usr/bin/env python

# File: t4.3-ik.py
# =======================
#
# Compute Inverse Kinematics
#
# Slide the end-effector forward and backward in the X-direction

from amino import SceneWin, SceneGraph, SceneIK, QuatTrans, YAngle
from math import pi
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

# Create the sub-scenegraph from root to "hand"
ssg = sg[:"hand"]

# Initialize and Start Window
win = SceneWin(scenegraph=sg, start=True, background=True)

# Create the inverse kinematics context
ik = SceneIK(ssg)

# IK Parameters
ik.set_seed_center()
ik.restart_time = 100e-3

# Set the reference transform
ik.ref_tf = (YAngle(pi / 2), (0.2, 0, 0.7))

dt = .1
period = 3
while win.is_runnining():
    # Solve IK
    q_sub = ik.solve()

    # Display in Window
    if q_sub:  # check for valid solution
        print q_sub
        win.config = ssg.scatter_config(q_sub)
    else:
        print "No IK Solution"

    # reseed for next run
    ik.set_seed_rand()

    # Sleep a bit
    t = 0
    while win.is_runnining() and t < period:
        sleep(dt)
        t += dt
