#!/usr/bin/env python3

from math import pi
import os
from amino import SceneGraph, SceneCollision, MotionPlan


# Compute Motion Plan


scene_plugin = (
    "%s/git/amino/demo/tutorial/plugin/urdf/libbaxter-model.so" % os.environ['HOME'])
scene_name = "baxter"


# Create an (empty) scene graph
sg = SceneGraph()

# Load the scene plugin
sg.load(scene_plugin, scene_name)

# Initialize the scene graph
sg.init()
sc = SceneCollision(sg)

# Create the sub-scenegraph from root to "hand"
ssg = sg[:"right_w2"]


# Initialize and Start Window
# win = SceneWin(scenegraph=sg, start=True, background=True)
# print("window made")

mp = MotionPlan(ssg)

start = [0 for _ in range(0, 15)]
goal = [.05 * pi,
        -.25 * pi,
        0,
        .25*pi,
        0,
        .25*pi,
        0 ]
path = mp.motion_plan(start, goal, 5)

[print(x) for x in path]
