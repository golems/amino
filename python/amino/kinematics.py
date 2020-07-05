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
"""Forward, differential, and inverse kinematics."""

import ctypes
from amino.lib import libamino
from amino.tf import QuatTrans
from amino.mat import DVec, DMat
from amino.scenegraph import RxSg, RxSgSub

########################
## Forward Kinematics ##
########################


class RxFK(ctypes.Structure):
    """Opaque type for fk result pointer"""

class SceneFK:
    """Forward Kinematics."""
    __slots__ = ['_ptr', '_q', 'scenegraph']

    def __init__(self, scenegraph):
        self.scenegraph = scenegraph
        self._ptr = libamino.aa_rx_fk_malloc(scenegraph._ptr)
        self._q = DVec(scenegraph.config_count)

    def __del__(self):
        libamino.aa_rx_fk_destroy(self._ptr)

    def __getitem__(self, key):
        """Get a transform.

        Args:
            key: a frame name/id or pair of frame names/ids.
        """
        E = QuatTrans()
        if isinstance(key, tuple):
            parent, child = key
            parent = self.scenegraph.ensure_frame_id(parent)
            child = self.scenegraph.ensure_frame_id(child)
            libamino.aa_rx_fk_get_rel_qutr(self._ptr, parent, child, E)
        else:
            key = self.scenegraph.ensure_frame_id(key)
            libamino.aa_rx_fk_get_abs_qutr(self._ptr, key, E)
        return E

    @property
    def config(self):
        """The current configuration."""
        return self._q

    @config.setter
    def config(self, config):
        self.scenegraph.config_vector(config, self._q)
        libamino.aa_rx_fk_all(self._ptr, self._q)
        return self._q

    def __len__(self):
        """Number of frames."""
        return libamino.aa_rx_fk_cnt(self._ptr)

    def _data_ptr(self):
        """Return pointer to data."""
        return libamino.aa_rx_fk_data(self._ptr)

    def _data_ld(self):
        """Return leading dimension of data."""
        return libamino.aa_rx_fk_ld(self._ptr)


libamino.aa_rx_fk_malloc.argtypes = [ctypes.POINTER(RxSg)]
libamino.aa_rx_fk_malloc.restype = ctypes.POINTER(RxFK)

libamino.aa_rx_fk_destroy.argtypes = [ctypes.POINTER(RxFK)]

libamino.aa_rx_fk_all.argtypes = [ctypes.POINTER(RxFK), ctypes.POINTER(DVec)]

libamino.aa_rx_fk_get_abs_qutr.argtypes = [
    ctypes.POINTER(RxFK), ctypes.c_int,
    ctypes.POINTER(QuatTrans)
]

libamino.aa_rx_fk_get_rel_qutr.argtypes = [
    ctypes.POINTER(RxFK), ctypes.c_int, ctypes.c_int,
    ctypes.POINTER(QuatTrans)
]

libamino.aa_rx_fk_data.argtypes = [ctypes.POINTER(RxFK)]
libamino.aa_rx_fk_data.restype = ctypes.POINTER(ctypes.c_double)

libamino.aa_rx_fk_ld.argtypes = [ctypes.POINTER(RxFK)]
libamino.aa_rx_fk_ld.restype = ctypes.c_size_t

libamino.aa_rx_fk_cnt.argtypes = [ctypes.POINTER(RxFK)]
libamino.aa_rx_fk_cnt.restype = ctypes.c_size_t

#############################
## Differential Kinematics ##
#############################


class RxWkOpts(ctypes.Structure):
    """Opaque type for workspace options result pointer"""

class SceneDK:
    """Differential Kinematics"""
    __slots__ = [
        "opts",
        "ssg",
        "fk",
        "_tf_ref",
        "_tf_vel_ref",
        "_config_ref",
        "dx",
    ]

    def __init__(self, sub_scenegraph, scene_fk):
        self.ssg = sub_scenegraph
        self.fk = scene_fk
        self.opts = libamino.aa_rx_wk_opts_create()
        self._tf_ref = None
        self._tf_vel_ref = None
        self._config_ref = None
        self.dx = DVec(6)

    def __del__(self):
        libamino.aa_rx_wk_opts_destroy(self.opts)

    @property
    def ref_tf(self):
        """Reference pose."""
        return self._tf_ref

    @ref_tf.setter
    def ref_tf(self, tf):
        self._tf_ref = QuatTrans(tf)

    @property
    def ref_tf_vel(self):
        """Reference pose velocity."""
        # TODO: should we copy?
        return self._tf_vel_ref

    @ref_tf_vel.setter
    def ref_tf_vel(self, dx):
        # TODO: should we copy?
        self._tf_vel_ref = dx

    @property
    def ref_config_all(self):
        """Reference configuration."""
        # TODO: should we to copy?
        return self._config_ref

    @ref_config_all.setter
    def ref_config_all(self, value):
        """Reference configuration."""
        sg = self.ssg.scenegraph
        self._config_ref = sg.copy_config(value)

    def solve_vel(self, dq=None):
        """Solve for velocity.

        Returns:
            sub-scenegraph configuration velocity or None if no solution.
        """
        if (self._tf_ref is None) and (self._tf_vel_ref is None):
            raise Exception("No reference provided.")

        if dq is None:
            dq = DVec(self.ssg.config_count)

        dx = self.dx

        # velocity reference
        if self._tf_vel_ref:
            dx.copy_from(self._tf_vel_ref)
        else:
            dx.zero()

        # position reference
        if self._tf_ref:
            tf_act = self.fk[self.ssg.end_effector_id]
            libamino.aa_rx_wk_dx_pos(self.opts, tf_act, self._tf_ref, dx)

        # configuration reference
        # TODO

        r = libamino.aa_rx_wk_dx2dq(self.ssg._ptr, self.opts, self.fk._ptr, dx,
                                    dq)

        return dq if r == 0 else None


libamino.aa_rx_wk_opts_create.argtypes = []
libamino.aa_rx_wk_opts_create.restype = ctypes.POINTER(RxWkOpts)
libamino.aa_rx_wk_opts_destroy.argtypes = [ctypes.POINTER(RxWkOpts)]

libamino.aa_rx_wk_dx2dq.argtypes = [
    ctypes.POINTER(RxSgSub),
    ctypes.POINTER(RxWkOpts),
    ctypes.POINTER(RxFK),
    ctypes.POINTER(DVec),
    ctypes.POINTER(DVec)
]
libamino.aa_rx_wk_dx2dq.restype = ctypes.c_int

libamino.aa_rx_wk_dx_pos.argtypes = [
    ctypes.POINTER(RxWkOpts),
    ctypes.POINTER(QuatTrans),
    ctypes.POINTER(QuatTrans),
    ctypes.POINTER(DVec)
]

########################
## Inverse Kinematics ##
########################


class RxIK(ctypes.Structure):
    """Opaque type for inverse kinematics context"""

class RxIKParm(ctypes.Structure):
    """Opaque type for inverse kinematics parameters"""

class SceneIK:
    """Inverse Kinematics"""
    __slots__ = ["_ptr", "ik_parm", "ssg", "_tf_ref"]

    def __init__(self, sub_scenegraph):
        self.ssg = sub_scenegraph
        self._tf_ref = None
        self.ik_parm = libamino.aa_rx_ik_parm_create()
        self._ptr = libamino.aa_rx_ik_cx_create(self.ssg._ptr, self.ik_parm)

    def __del__(self):
        libamino.aa_rx_ik_cx_destroy(self._ptr)
        libamino.aa_rx_ik_parm_destroy(self.ik_parm)

    @property
    def ref_tf(self):
        """Reference transformation"""
        return self._tf_ref

    @ref_tf.setter
    def ref_tf(self, tf_ref):
        """Set the reference transformation"""
        self._tf_ref = QuatTrans(tf_ref)

    def set_seed(self, config_sub):
        """Sets the optimization seed."""
        q = self.ssg.config_vector(config_sub)
        libamino.aa_rx_ik_set_seed(self._ptr, q)

    def set_seed_center(self):
        """Sets the optimization seed to the joint center position."""
        libamino.aa_rx_ik_set_seed_center(self._ptr)

    def set_seed_rand(self):
        """Sets the optimization seed to a random position."""
        libamino.aa_rx_ik_set_seed_rand(self._ptr)

    def set_obj(self, fun):
        """Sets the objective function for optimization."""
        libamino.aa_rx_ik_parm_set_obj(self.ik_parm, fun)

    def set_tol_angle(self, angle):
        """Sets the angle error tolerance"""
        print(angle)
        libamino.aa_rx_ik_parm_set_tol_angle(self.ik_parm, angle)

    def set_tol_trans(self, trans):
        """Sets the translational error tolerance"""
        libamino.aa_rx_ik_parm_set_tol_trans(self.ik_parm, trans)

    @property
    def restart_time(self):
        """Maximum time limit for IK restarts."""
        return libamino.aa_rx_ik_get_restart_time(self._ptr)

    @restart_time.setter
    def restart_time(self, t):
        libamino.aa_rx_ik_set_restart_time(self._ptr, t)

    def solve(self):
        """
        Solves the IK problem.

        Returns:
            The sub-scenegraph configuration vector or None if no solution.
        """
        if self._tf_ref is None:
            raise Exception("No reference provided.")

        M = DMat((len(self._tf_ref), 1))
        M.col_vec(0).copy_from(self._tf_ref)

        q = DVec(self.ssg.config_count)

        r = libamino.aa_rx_ik_solve(self._ptr, M, q)

        return q if r == 0 else None


libamino.aa_rx_ik_parm_create.argtypes = []
libamino.aa_rx_ik_parm_create.restype = ctypes.POINTER(RxIKParm)
libamino.aa_rx_ik_parm_destroy.argtypes = [ctypes.POINTER(RxIKParm)]

libamino.aa_rx_ik_cx_create.argtypes = [
    ctypes.POINTER(RxSgSub), ctypes.POINTER(RxIKParm)
]
libamino.aa_rx_ik_cx_create.restype = ctypes.POINTER(RxIK)
libamino.aa_rx_ik_cx_destroy.argtypes = [ctypes.POINTER(RxIK)]

libamino.aa_rx_ik_solve.argtypes = [
    ctypes.POINTER(RxIK),
    ctypes.POINTER(DMat),
    ctypes.POINTER(DVec)
]
libamino.aa_rx_ik_solve.restype = ctypes.c_int

libamino.aa_rx_ik_set_seed.argtypes = [
    ctypes.POINTER(RxIK), ctypes.POINTER(DVec)
]
libamino.aa_rx_ik_set_seed_center.argtypes = [ctypes.POINTER(RxIK)]
libamino.aa_rx_ik_set_seed_rand.argtypes = [ctypes.POINTER(RxIK)]

libamino.aa_rx_ik_set_restart_time.argtypes = [
    ctypes.POINTER(RxIK), ctypes.c_double
]

libamino.aa_rx_ik_parm_set_tol_angle.argtypes = [ctypes.POINTER(RxIKParm), ctypes.c_double]
libamino.aa_rx_ik_parm_set_tol_trans.argtypes = [ctypes.POINTER(RxIKParm), ctypes.c_double]
libamino.aa_rx_ik_get_restart_time.argtypes = [ctypes.POINTER(RxIK)]
libamino.aa_rx_ik_get_restart_time.restype = ctypes.c_double
