#!/usr/bin/env python

# File: t4.2-fk.py
# =======================
#
# Compute Differential Kinematics
#
# Slide the end-effector forward and backward in the X-direction

from amino import SceneWin, SceneGraph, SceneIK, QuatTrans, YAngle
from math import pi
import os

# Scene Parameters
# Change scene_plugin based on your directory structure
scene_plugin="%s/git/amino/demo/tutorial/plugin/7dof/libscene.so" % os.environ['HOME']
scene_name="7dof"

# Create an (empty) scene graph
sg = SceneGraph()

# Load the scene plugin
sg.load(scene_plugin,scene_name)

# Initialize the scene graph
sg.init()

# Create the sub-scenegraph
ssg = sg.chain("","hand")

# Create the inverse kinematics context
ik = SceneIK(ssg)

# IK Parameters
ik.set_seed_rand()
#ik.set_seed_center()
ik.set_restart_time(100e-3)

# Set the reference transform
tf_ref = QuatTrans((YAngle(pi/2), (0.2, 0, 0.7)))
ik.set_ref_tf(tf_ref)

# Solve IK
q_sub = ik.solve()
if not q_sub: # check for valid solution
    raise Exception("No IK Solution")
q_all = ssg.scatter_config(q_sub)

# Initialize Window
win = SceneWin(scenegraph=sg,config=q_all,start=True,async=False)
