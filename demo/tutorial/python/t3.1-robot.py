#!/usr/bin/env python3

# File: t3.1-robot.py
# ===================
#
# Procedural construction of a robot scene.

from math import pi, cos
from time import sleep
from amino import SceneWin, SceneGraph, Geom, GeomOpt, Vec3, YAngle, XAngle


def draw_robot(sg, parent):
    """Draw a robot in the scenegraph sg under parent"""
    radius = .05  # link radius
    len0 = 1  # upper arm length
    len1 = 1  # forearm length

    r = 1.618

    joint_opts = GeomOpt({"color": (0, 0, 0), "specular": (.5, .5, .5)})

    link_opts = GeomOpt({"color": (.5, .5, .5), "specular": (2, 2, 2)})

    # Shoulder
    sg.add_frame_revolute(
        parent, "s0", axis=(1, 0, 0), geom=Geom.sphere(joint_opts, r * radius))
    sg.add_frame_revolute("s0", "s1", axis=(0, 1, 0))
    sg.add_frame_revolute("s1", "s2", axis=(1, 0, 0))
    sg.add_frame_fixed(
        "s2",
        "upper-link",
        tf=(YAngle(pi / 2), Vec3.identity()),
        geom=Geom.cylinder(link_opts, len0, radius))

    # Elbow
    sg.add_frame_revolute("s2", "e", axis=(0, 1, 0), tf=(1, (len0, 0, 0)))
    sg.add_frame_fixed(
        "e",
        "e-link",
        tf=(XAngle(.5 * pi), (0, radius, 0)),
        geom=Geom.cylinder(joint_opts, 2 * radius, r * radius))
    sg.add_frame_fixed(
        "e",
        "lower-link",
        tf=(YAngle(pi / 2), Vec3.identity()),
        geom=Geom.cylinder(link_opts, len1, radius))

    # "Hand"
    sg.add_frame_fixed(
        "e",
        "hand",
        tf=(YAngle(.5 * pi), (len1, 0, 0)),
        geom=Geom.cone(joint_opts, 2 * (r**2) * radius, r * radius, 0))

    sg.init()


# Create an (empty) scene graph
sg = SceneGraph()

# Draw a grid
sg.add_frame_fixed(
    "",
    "grid",
    geom=Geom.grid({
        "color": (0, 0, 1)
    }, [1.0, 1.0], [.1, .1], .005))
# Draw the robot
draw_robot(sg, "grid")

# Initialize the scene graph
sg.init

# Create a window and pass the scenegraph
win = SceneWin(start=False)
win.scenegraph = sg

# Start the window in a background thread
win.start(True)

# Do a simple wave
dt = 1.0 / 60
t = 0
while win.is_runnining():
    t += dt
    e_angle = (120 + 15 * cos(t)) * (pi / 180)
    win.config = {"s1": -.75 * pi, "e": e_angle}
    sleep(dt)
