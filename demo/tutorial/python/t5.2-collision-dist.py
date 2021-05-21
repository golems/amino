#!/usr/bin/env python3

# File: t5.2-collision-dist.py
# ============================
#
# Collision Distances.
#
# Check collision distances and mark the closest points in the scene
# viewer window.

from math import pi, sin
from time import sleep
import os
from amino import SceneWin, SceneGraph, SceneFK, SceneCollision,  Geom

# Scene Parameters
scene_plugin = ("{}/../plugin/7dof/libscene.so".format(os.path.dirname(__file__)))
scene_name = "7dof"

# We will mark the closest points in the scene with colored spheres.
# Each sphere marker will be at the end of a chain of three prismatic
# joints allowing to set the x-y-z position of the marker.  To
# following functions setup the markers.


def marker_dim_name(frame, other, dim):
    """Returns frame and config name for a marker dimension."""
    return "closest-points/%s/%s/%d" % (frame, other, dim)


def add_marker(sg, frame, other, color):
    """Adds a marker to the scene."""
    frame_x = marker_dim_name(frame, other, 0)
    frame_y = marker_dim_name(frame, other, 1)
    frame_z = marker_dim_name(frame, other, 2)
    sg.add_frame_prismatic("", frame_x, axis=(1, 0, 0))
    sg.add_frame_prismatic(frame_x, frame_y, axis=(0, 1, 0))
    sg.add_frame_prismatic(frame_y, frame_z, axis=(0, 0, 1))
    sg.attach_geom(frame_z, Geom.sphere({"color": color}, .05))


def add_marker_pair(sg, frame0, frame1, color):
    """Adds a pair of markers."""
    add_marker(sg, frame0, frame1, color)
    add_marker(sg, frame1, frame0, color)


def set_marker_config(sg, config, frame, other, closest_point):
    """Sets the configuration for the marker to closest_point."""
    for k in range(0, 3):
        f_id = sg.config_id(marker_dim_name(frame, other, k))
        config[f_id] = closest_point[k]


def mark_dists(
        sg,
        cl_dist,
        config,
        frame_i,
        frame_j,
):
    """Updates the marker configuration to the closest point."""

    # Get the closest points
    (dist, point_i, point_j) = cl_dist.points[frame_i, frame_j]

    # Set the marker configurations
    set_marker_config(sg, config, frame_i, frame_j, point_i)
    set_marker_config(sg, config, frame_j, frame_i, point_j)


# Create an (empty) scene graph
sg = SceneGraph()

# Load the scene plugin
sg.load(scene_plugin, scene_name)

# Add markers for closest points
add_marker_pair(sg, "s0", "lower-link", (1, 0, 0))
add_marker_pair(sg, "w2", "upper-link", (0, 0, 1))

# Initialize the scene graph
sg.init()

# Initial Configuration
config_dict = {
    "s1": -.75 * pi,
    "e": .75 * pi,
    "f0": .125 * pi,
    "f1": .125 * pi,
    "f2": .125 * pi
}

# ForwardKinematics, Collision context and collision set
fk = SceneFK(sg)
cl = SceneCollision(sg)
cl_set = cl.collision_set()
cl_dist = cl.collision_dist()

# Allowable collisions
cl.allow_config(config_dict)

# Create a window in background thread
win = SceneWin(scenegraph=sg, start=True, background=True, config=config_dict)

dt = 1.0 / 60
t = 0
config_vec = sg.config_vector(config_dict)
e_id = sg.config_id('e')
while win.is_runnining():
    # Move elbow
    config_vec[e_id] = (15 * sin(t)) * (pi / 180) + config_dict['e']

    # Check Distances
    fk.config = config_vec
    cl_dist.check(fk)

    # Set marker positions
    mark_dists(sg, cl_dist, config_vec, "s0", "lower-link")
    mark_dists(sg, cl_dist, config_vec, "w2", "upper-link")

    # Update window
    win.fk = fk

    # Sleep till next cycle
    sleep(dt)
    t += dt
