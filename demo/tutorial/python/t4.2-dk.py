#!/usr/bin/env python

# File: t4.2-dk.py
# =======================
#
# Compute Differential Kinematics
#
# Slide the end-effector forward and backward in the X-direction

from amino import SceneWin, SceneGraph, Geom, SceneFK, SceneDK, TfVel, QuatTrans, Vec3, YAngle
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

# Initial configuration
q_all = sg.config_vector({
    "s1": -.75 * pi,
    "e": .75 * pi,
    "f0": .125 * pi,
    "f1": .125 * pi,
    "f2": .125 * pi
})

# Draw in the the path that the robot will follow
fk = SceneFK(sg)  # temporary forward kinematics context
fk.config = q_all
tf0 = fk["hand"]
path_opts = {"color": (1, 0, 0), "alpha": .25}
path_h = .75
path_r = 5e-3
sg.add_frame_fixed(
    "",
    "path",
    tf=QuatTrans((YAngle(pi / 2), tf0.translation)),
    geom=(Geom.cylinder(path_opts, path_h, path_r),
          Geom.cylinder(path_opts, -path_h, path_r)))
sg.init()  # need to re-init after adding a frame

# Forward kinematics context
fk = SceneFK(sg)

# Initialize Window
win = SceneWin(scenegraph=sg, start=True, background=True)

# Create the sub-scenegraph from root to "hand"
ssg = sg[:"hand"]

# Create the differential kinematics context
dk = SceneDK(ssg, fk)

# Slide hand forward and back
dt = 1.0 / 60
t = 0
tfvel = TfVel()
while win.is_runnining():
    # update forward kinematics
    fk.config = q_all

    # X-direction position and velocity
    x = .25 * sin(t)  # position
    dx = .25 * cos(t)  # velocity, the derivative of x

    # Set the reference velocity.
    tfvel.translational = (dx, 0, 0)
    dk.ref_tf_vel = tfvel

    # Set the reference pose.  If we omit position feedback, the
    # robot will drift over time due to floating point error.
    dk.ref_tf = (tf0.rotation, (tf0.translation + (x, 0, 0)))

    # Get config velocity
    dq_sub = dk.solve_vel()

    # Euler integration of configuration
    if dq_sub:
        dq_all = ssg.scatter_config(dq_sub)
        q_all += dt * dq_all
    else:
        print("Error")

    # update window
    win.config = q_all

    # sleep till next cycle
    sleep(dt)
    t += dt
