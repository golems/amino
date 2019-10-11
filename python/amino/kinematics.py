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
## @file kinematics.py Scene Graphs
##

import ctypes
from lib import libamino
from tf import Vec3,Quat,QuatTrans, TfVel, Twist
from mat import DVec, DMat
from scenegraph import SceneGraph, sg, sg_sub

########################
## Forward Kinematics ##
########################

class fk(ctypes.Structure):
    """Opaque type for fk result pointer"""
    pass

class SceneFK(object):
    """Forward Kinematics"""
    __slots__ = ['ptr', 'q', 'scenegraph']

    def __init__(self, scenegraph):
        self.scenegraph = scenegraph
        self.ptr = libamino.aa_rx_fk_malloc(scenegraph.ptr)
        self.q = DVec.create(scenegraph.config_count())

    def __del__(self):
        libamino.aa_rx_fk_destroy(self.ptr)

    def update(self,config):
        self.scenegraph.config_vector(config, self.q)
        libamino.aa_rx_fk_all(self.ptr, self.q)

    def tf_abs(self, frame):
        """Get the absolute TF for frame"""
        E = QuatTrans()
        frame = self.scenegraph.ensure_frame_id(frame)
        libamino.aa_rx_fk_get_abs_qutr(self.ptr, frame, E)
        return E

    def tf_rel(self, parent, child):
        """Get the relative TF for frame"""
        E = QuatTrans()
        parent = self.scenegraph.ensure_frame_id(parent)
        child = self.scenegraph.ensure_frame_id(child)
        libamino.aa_rx_fk_get_rel_qutr(self.ptr, parent, child, E)
        return E


libamino.aa_rx_fk_malloc.argtypes = [ctypes.POINTER(sg)]
libamino.aa_rx_fk_malloc.restype = ctypes.POINTER(fk)

libamino.aa_rx_fk_destroy.argtypes = [ctypes.POINTER(fk)]

libamino.aa_rx_fk_all.argtypes = [ctypes.POINTER(fk), ctypes.POINTER(DVec)]

libamino.aa_rx_fk_get_abs_qutr.argtypes = [ctypes.POINTER(fk), ctypes.c_int,
                                           ctypes.POINTER(QuatTrans)]

libamino.aa_rx_fk_get_rel_qutr.argtypes = [ctypes.POINTER(fk),
                                           ctypes.c_int, ctypes.c_int,
                                           ctypes.POINTER(QuatTrans)]



#############################
## Differential Kinematics ##
#############################


class wk_opts(ctypes.Structure):
    """Opaque type for workspace options result pointer"""
    pass


class SceneDK(object):
    """Differential Kinematics"""
    __slots__ = ["opts", "ssg", "fk",
                 "tf_ref",
                 "vel_ref",
                 "config_vector_ref",
                 "dx",
    ]

    def __init__(self, sub_scenegraph, scene_fk):
        self.ssg = sub_scenegraph
        self.fk = scene_fk
        self.opts = libamino.aa_rx_wk_opts_create()
        self.tf_ref = None
        self.vel_ref = None
        self.config_vector_ref = None
        self.dx = DVec(6)

    def __del__(self):
        libamino.aa_rx_wk_opts_destroy(self.opts)

    def set_ref_tf(self,tf):
        """Set the reference pose."""
        # TODO: should we to copy?
        self.tf_ref = tf

    def set_ref_tf_vel(self, dx):
        """Set the reference pose velocity."""
        # TODO: should we to copy?
        self.vel_ref = dx
        pass

    def set_ref_config_vector_all(self, config_vector):
        """Set the reference configuration vector."""
        # TODO: should we to copy?
        self.config_vector_ref = config_vector

    def set_ref_config_dict(self, config_dict):
        sg = self.ssg.scenegraph
        self.config_vector_ref = sg.config_vector(config_dict, self.config_vector_ref)


    def solve_vel(self, dq=None):
        """Solve for velocity.

        Returns:
            sub-scenegraph configuration velocity or None if no solution.
        """
        if( (self.tf_ref is None) and (self.vel_ref is None) ):
            raise Exception("No reference provided.")

        if( dq is None ):
            dq = DVec(self.ssg.config_count());

        dx = self.dx

        # velocity reference
        if( not (self.vel_ref is None) ):
            dx.copy_from(self.vel_ref)
        else:
            dx.zero()

        # position reference
        if( not (self.tf_ref is None) ):
            tf_act = self.fk.tf_abs( self.ssg.end_effector_id() )
            libamino.aa_rx_wk_dx_pos(self.opts, tf_act, self.tf_ref, dx )

        # configuration reference
        # TODO

        r = libamino.aa_rx_wk_dx2dq( self.ssg.ptr, self.opts, self.fk.ptr,
                                     dx, dq )

        if( 0 == r ):
            return dq
        else:
            return None

libamino.aa_rx_wk_opts_create.argtypes = []
libamino.aa_rx_wk_opts_create.restype = ctypes.POINTER(wk_opts)
libamino.aa_rx_wk_opts_destroy.argtypes = [ctypes.POINTER(wk_opts)]


libamino.aa_rx_wk_dx2dq.argtypes = [ ctypes.POINTER(sg_sub),
                                     ctypes.POINTER(wk_opts),
                                     ctypes.POINTER(fk),
                                     ctypes.POINTER(DVec),
                                     ctypes.POINTER(DVec) ]
libamino.aa_rx_wk_dx2dq.restype = ctypes.c_int

libamino.aa_rx_wk_dx_pos.argtypes = [ctypes.POINTER(wk_opts),
                                     ctypes.POINTER(QuatTrans),
                                     ctypes.POINTER(QuatTrans),
                                     ctypes.POINTER(DVec)]

########################
## Inverse Kinematics ##
########################

class ik_cx(ctypes.Structure):
    """Opaque type for inverse kinematics context"""
    pass

class ik_parm(ctypes.Structure):
    """Opaque type for inverse kinematics parameters"""
    pass


class SceneIK(object):
    """Inverse Kinematics"""
    __slots__ = ["ptr", "ik_parm",
                 "ssg",
                 "tf_ref" ]

    def __init__(self, sub_scenegraph):
        self.ssg = sub_scenegraph
        self.tf_ref = None
        self.ik_parm = libamino.aa_rx_ik_parm_create()
        self.ptr = libamino.aa_rx_ik_cx_create(self.ssg.ptr, self.ik_parm)

    def __del__(self):
        libamino.aa_rx_ik_cx_destroy(self.ptr)
        libamino.aa_rx_ik_parm_destroy(self.ik_parm)

    def set_ref_tf(self, tf_ref):
        """Set the reference transformation"""
        self.tf_ref = tf_ref

    def set_seed(self, config_sub):
        q = self.ssg.config_vector(config_sub)
        libamino.aa_rx_ik_set_seed(self.ptr, q)

    def set_seed_center(self):
        libamino.aa_rx_ik_set_seed_center(self.ptr)

    def set_seed_rand(self):
        libamino.aa_rx_ik_set_seed_rand(self.ptr)

    def set_restart_time(self, t):
        libamino.aa_rx_ik_set_restart_time(self.ptr, t)

    def solve(self):
        """
        Solve the IK problem.

        Returns:
            The sub-scenegraph configuration vector or None if no solution.
        """
        if self.tf_ref is None:
            raise Exception("No reference provided.")

        M = DMat((len(self.tf_ref), 1))
        M.col_vec(0).copy_from( self.tf_ref )

        q = DVec(self.ssg.config_count())

        r = libamino.aa_rx_ik_solve( self.ptr, M, q )

        return q if 0 == r else None


libamino.aa_rx_ik_parm_create.argtypes = []
libamino.aa_rx_ik_parm_create.restype = ctypes.POINTER(ik_parm)
libamino.aa_rx_ik_parm_destroy.argtypes = [ctypes.POINTER(ik_parm)]

libamino.aa_rx_ik_cx_create.argtypes = [ctypes.POINTER(sg_sub), ctypes.POINTER(ik_parm) ]
libamino.aa_rx_ik_cx_create.restype = ctypes.POINTER(ik_cx)
libamino.aa_rx_ik_cx_destroy.argtypes = [ctypes.POINTER(ik_cx)]


libamino.aa_rx_ik_solve.argtypes = [ ctypes.POINTER(ik_cx),
                                     ctypes.POINTER(DMat),
                                     ctypes.POINTER(DVec) ]
libamino.aa_rx_ik_solve.restype = ctypes.c_int


libamino.aa_rx_ik_set_seed.argtypes = [ ctypes.POINTER(ik_cx), ctypes.POINTER(DVec) ]
libamino.aa_rx_ik_set_seed_center.argtypes = [ctypes.POINTER(ik_cx)]
libamino.aa_rx_ik_set_seed_rand.argtypes = [ctypes.POINTER(ik_cx)]
libamino.aa_rx_ik_set_restart_time.argtypes = [ctypes.POINTER(ik_cx), ctypes.c_double]
