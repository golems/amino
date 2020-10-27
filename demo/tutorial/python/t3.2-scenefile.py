#!/usr/bin/env python3

# File: t3.2-scenefile.py
# =======================
#
# Load a compiled scene plugin

from math import pi, cos
from time import sleep
import os
from amino import SceneWin, SceneGraph

# Scene Parameters
scene_plugin = ("{}/../plugin/scenefile/libscene.so".format(os.path.dirname(__file__)))
scene_name = "example"

# Create an (empty) scene graph
sg = SceneGraph()

# Load the scene plugin
sg.load(scene_plugin, scene_name)

# Initialize the scene graph
sg.init()

# Create a window, pass the scenegraph, and start
win = SceneWin(scenegraph=sg, start=True, background=True)

# Do a simple wave
dt = 1.0 / 60
t = 0
while win.is_runnining():
    t += dt
    e_angle = (120 + 15 * cos(t)) * (pi / 180)
    win.config = {"s1": -.75 * pi, "e": e_angle}
    sleep(dt)
