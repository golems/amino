#!/usr/bin/env python

# File: t5.1-collision-check.py
# =======================
#
# Collision Checking

from amino import SceneWin, SceneGraph, SceneFK, SceneCollisionSet, SceneCollision, QuatTrans, Geom
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

config0 = config_dict

# Collision context and collision set
cl = SceneCollision(sg)
cl_set = cl.collision_set()
cl_dist = cl.collision_dist()

# Allowable collisions
cl.allow_config(config_dict)

# Mark closes points
fk.config = config_dict
cl_dist.check(fk)


def mark_points(cl_dist, sg, i, j, color):
    (dist, point_i, point_j) = cl_dist.points[i, j]
    print("--")
    print("%s, %s" % (i, j))
    print("%f" % dist)
    print("%s" % point_i)
    print("%s" % point_j)
    sg.add_frame_fixed(
        "",
        i + "/closest-point/" + j,
        tf=QuatTrans((1, point_i)),
        geom=Geom.sphere({
            "color": color
        }, .07))

    sg.add_frame_fixed(
        "",
        j + "/closest-point/" + i,
        tf=QuatTrans((1, point_j)),
        geom=Geom.sphere({
            "color": color
        }, .07))


def check_dist(sg, config):
    cl = SceneCollision(sg)
    cl_set = cl.collision_set()
    cl_dist = cl.collision_dist()

    # Allowable collisions
    cl.allow_config(config0)

    # Mark closest points
    fk.config = config_dict
    cl_dist.check(fk)

    mark_points(cl_dist, sg, "s0", "lower-link", (1, 0, 0))
    mark_points(cl_dist, sg, "upper-link", "w2", (0, 0, 1))

    sg.init()


# Create a window, pass the scenegraph, and start
win = SceneWin(scenegraph=sg, start=True, background=True, config=config_dict)

dt = 1.0 / 60
t = 0
while win.is_runnining():
    # wiggle fingers
    config_dict = config0.copy()
    config_dict['e'] = (15 * sin(t)) * (pi / 180) + config0['e']

    win.lock()

    check_dist(sg, config_dict)

    # update window
    win.scenegraph = sg
    win.config = config_dict

    win.unlock()

    # sleep till next cycle
    sleep(dt)
    t += dt
