#!/usr/bin/env python3

# File: t3.2-urdf.py
# ===================
#
# Loading a compiled scene plugin

import os
from amino import SceneWin, SceneGraph, Geom

# Scene Parameters
# Change scene_plugin based on your directory structure
scene_plugin = ("%s/git/amino/demo/tutorial/plugin/urdf/libbaxter-model.so" %
                os.environ['HOME'])
scene_name = "baxter"

# Create an (empty) scene graph
sg = SceneGraph()

# Draw a grid
sg.add_frame_fixed(
    "",
    "grid",
    tf=(1, (0, 0, -.9)),
    geom=Geom.grid({
        "color": (0, 0, 1)
    }, [1.0, 1.0], [.1, .1], .005))

# Load the scene plugin
sg.load(scene_plugin, scene_name)

# Initialize the scene graph
sg.init()

# Create a window and pass the scenegraph
win = SceneWin(start=False)
win.scenegraph = sg

# Start the window in a background thread
win.start(False)
