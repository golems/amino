# !/usr/bin/env python
#
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

from math import pi,sin,cos
import time
from amino import *

def wiggle_elbow(win, sg, v_default):
    t = 0
    dt = 1.0 / 60;
    while True:
        theta = .25*pi - .25*pi*cos(t)
        v = sg.config_vector({'right_e1': theta}, v_default)
        win.set_config(v)
        time.sleep(dt)
        t = t + dt;


def wiggle_workspace(win, sg_sub, q):
    t = 0
    dt = 1.0 / 60;
    sg = sg_sub.scenegraph
    v = sg.config_vector(q)

    while True:
        # Set workspace reference velocity
        dx = DVec.create(6).set(0)
        dx[2] = .05*cos(t)

        # Compute jointspace velocity
        tf = sg.get_tf_abs(v)
        J = sg_sub.jacobian(tf)
        dq_sub = J.pinv(.005) * dx
        dq_all = sg_sub.scatter_config(dq_sub)

        # Integrate and display
        v += dt*dq_all
        t +=  dt;
        win.set_config(v)
        time.sleep(dt)


if __name__ == '__main__':
    sg = SceneGraph().load(".libs/libamino_baxter.so", "baxter").init()
    win = SceneWin(scenegraph=sg, start=True, async=True)
    q = {'right_s0': .05*pi,
         'right_s1': -.25*pi,
         'right_e1': .25*pi,
         'right_w1': .25*pi }
    wiggle_workspace( win, sg.chain("","right_w2"), q )
