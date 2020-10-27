#!/usr/bin/env python3

# File: t3-scene.py
# =================
#
# Create a scene with a grid and three boxes.  Display the scene in
# the viewer window.

from amino import SceneWin, SceneGraph, Geom, QuatTrans, GeomOpt

# Create a window
win = SceneWin(start=False)

# Create an (empty) scene graph
sg = SceneGraph()

# Draw a grid
sg.add_frame_fixed(
    "",
    "grid",
    geom=Geom.grid({
        "color": (0, 0, 1)
    }, [1.0, 1.0], [.1, .1], .005))

# Draw some boxes
h = .1  # box height
dim = [h, h, h]  # box dimension
sg.add_frame_fixed(
    "grid",
    "box-a",
    tf=QuatTrans((1, (0, 0, h / 2))),
    geom=Geom.box({
        "color": (1, 0, 0)
    }, dim))

sg.add_frame_fixed(
    "box-a",
    "box-b",
    tf=QuatTrans((1, (0, 0, h))),
    geom=Geom.box({
        "color": (0, 1, 0)
    }, dim))

sg.add_frame_fixed(
    "grid",
    "box-c",
    tf=QuatTrans((1, (3 * h, 0, h / 2))),
    geom=Geom.box({
        "color": (0, 0, 1)
    }, dim))

# Initalize the scene graph
sg.init()

# Pass the scenegraph to the window
win.scenegraph = sg

# Start the window in the current (main) thread
win.start(False)
