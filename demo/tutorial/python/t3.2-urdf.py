#!/usr/bin/env python

# File: t3.2-urdf.py
# ===================
#
# Loading a compiled scene plugin

from amino import SceneWin, SceneGraph, Geom, GeomOpt, QuatTrans, Vec3, YAngle, XAngle
from math import pi, cos
from time import sleep
import os


# Scene Parameters
# Change scene_plugin based on your directory structure
scene_plugin="%s/git/amino/demo/tutorial/plugin/libbaxter-model.so" % os.environ['HOME']
scene_name="baxter"

# Create an (empty) scene graph
sg = SceneGraph()

# Draw a grid
sg.add_frame_fixed("","grid",
                   tf=(1,(0,0,-.9)),
                   geom=Geom.Grid({"color": (0,0,1)},
                                  [1.0,1.0], [.1,.1], .005) )

# Load the scene plugin
sg.load(scene_plugin,scene_name)

# Initialize the scene graph
sg.init()

# Create a window and pass the scenegraph
win = SceneWin(start=False)
win.set_scenegraph(sg)

# Start the window in a background thread
win.start(async=False)
