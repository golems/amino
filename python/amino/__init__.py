#  Copyright (c) 2019, Colorado School of Mines
#  All rights reserved.
#
#  Author(s): Neil T. Dantam <ndantam@mines.edu>
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above
#      copyright notice, this list of conditions and the following
#      disclaimer in the documentation and/or other materials provided
#      with the distribution.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
#   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
#   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
#   BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
#   TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
#   TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
#   THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
#   SUCH DAMAGE.

##
## @file __init__.py Package init file
##
"""Amino Library Bindings"""

import ctypes

from amino.lib import libamino
from amino.tf import Vec3, \
    XAngle, YAngle, ZAngle, EulerRPY, EulerZYX, AxAng, Quat, RotMat, \
    TfMat, DualQuat, QuatTrans, \
    TfVel, Twist
from amino.mat import DVec, DMat
from amino.scenegraph import GeomOpt, Geom, SceneGraph, SubSceneGraph
from amino.scenewin import SceneWin

from amino.kinematics import SceneFK, SceneDK, SceneIK

from amino.collision import SceneCollisionSet, SceneCollision
