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
## @file collision.py Collision Checking
##
"""Collision checking"""

import ctypes
from amino.scenegraph import RxSg
from amino.kinematics import SceneFK

LIBAMINOCL = ctypes.CDLL("libamino-collision.so")


class RxClSet(ctypes.Structure):
    """Opaque type for struct aa_rx_cl_set."""
    pass


class SceneCollisionSet(object):
    """Set of frame pairs indicating collision between frames."""

    __slots__ = ["_ptr", "scenegraph"]

    def __init__(self, scenegraph):
        self.scenegraph = scenegraph
        self._ptr = LIBAMINOCL.aa_rx_cl_set_create(self.scenegraph._ptr)

    def __del__(self):
        LIBAMINOCL.aa_rx_cl_set_destroy(self._ptr)

    def __getitem__(self, key):
        """Returns the value for the frame pair indicated by key.

        Args:
            key: A tuple of two frame names or ids.

        Raises:
            IndexError: Invalid frame name or ID.
        """
        i, j = key
        i = self.scenegraph.ensure_frame_id_actual(i)
        j = self.scenegraph.ensure_frame_id_actual(j)
        v = LIBAMINOCL.aa_rx_cl_set_get(self._ptr, i, j)
        return bool(v)

    def __setitem__(self, key, item):
        """Sets the value for the frame pair indicated by key.

        Args:
            key: A tuple of two frame names or ids.
            item: True or False

        Raises:
            IndexError: Invalid frame ID.
        """
        i, j = key
        i = self.scenegraph.ensure_frame_id_actual(i)
        j = self.scenegraph.ensure_frame_id_actual(j)
        v = 1 if item else 0
        LIBAMINOCL.aa_rx_cl_set_set(self._ptr, i, j, v)

    def clear(self):
        """Sets all entries to False."""
        LIBAMINOCL.aa_rx_cl_set_clear(self._ptr)

    def fill(self, src):
        """Fills self with all true entries in other."""
        LIBAMINOCL.aa_rx_cl_set_fill(self._ptr, src._ptr)


LIBAMINOCL.aa_rx_cl_set_create.argtypes = [ctypes.POINTER(RxSg)]
LIBAMINOCL.aa_rx_cl_set_create.restype = ctypes.POINTER(RxClSet)

LIBAMINOCL.aa_rx_cl_set_destroy.argtypes = [ctypes.POINTER(RxClSet)]

LIBAMINOCL.aa_rx_cl_set_get.argtypes = [
    ctypes.POINTER(RxClSet), ctypes.c_int, ctypes.c_int
]
LIBAMINOCL.aa_rx_cl_set_get.restypes = ctypes.c_int

LIBAMINOCL.aa_rx_cl_set_set.argtypes = [
    ctypes.POINTER(RxClSet), ctypes.c_int, ctypes.c_int, ctypes.c_int
]

LIBAMINOCL.aa_rx_cl_set_clear.argtypes = [ctypes.POINTER(RxClSet)]

LIBAMINOCL.aa_rx_cl_set_fill.argtypes = [
    ctypes.POINTER(RxClSet), ctypes.POINTER(RxClSet)
]


class RxCl(ctypes.Structure):
    """Opaque type for struct aa_rx_cl."""
    pass


class SceneCollision(object):
    """Context object for collision detection."""

    __slots__ = ["_ptr", "scenegraph"]

    def __init__(self, scenegraph):
        self.scenegraph = scenegraph
        LIBAMINOCL.aa_rx_sg_cl_init(self.scenegraph._ptr)
        self._ptr = LIBAMINOCL.aa_rx_cl_create(self.scenegraph._ptr)

    def __del__(self):
        LIBAMINOCL.aa_rx_cl_destroy(self._ptr)

    def allow(self, i, j):
        """Allows collisions between frames i and j."""
        i = self.scenegraph.ensure_frame_id_actual(i)
        j = self.scenegraph.ensure_frame_id_actual(j)
        LIBAMINOCL.aa_rx_cl_allow(self._ptr, i, j)

    def allow_set(self, collision_set):
        """Allows collisions between all frames in the collision set."""
        LIBAMINOCL.aa_rx_cl_allow_set(self._ptr, collision_set._ptr)

    def allow_config(self, config):
        """Allows collisions between all frames coliding at config."""
        fk = SceneFK(self.scenegraph)
        fk.config = config
        cl_set = SceneCollisionSet(self.scenegraph)
        self.check(fk, cl_set)
        self.allow_set(cl_set)

    def check(self, scene_fk, collision_set=None):
        """Checks for collisions and optionally outputs colliding frames.

        Args:
            fk: A SceneFK updated with the configuration to check.
            collision_set: If not None, a SceneCollisionSet that will
                be filled with all detected, non-allowed collisions.

        Returns:
           True if collisions are detected and False otherwise.

        """
        cl_set_ptr = None
        if collision_set:
            collision_set.clear()
            cl_set_ptr = collision_set._ptr
        r = LIBAMINOCL.aa_rx_cl_check(self._ptr, len(scene_fk),
                                      scene_fk._data_ptr(),
                                      scene_fk._data_ld(), cl_set_ptr)
        return bool(r)


LIBAMINOCL.aa_rx_cl_create.argtypes = [ctypes.POINTER(RxSg)]
LIBAMINOCL.aa_rx_cl_create.restype = ctypes.POINTER(RxCl)

LIBAMINOCL.aa_rx_cl_destroy.argtypes = [ctypes.POINTER(RxCl)]

LIBAMINOCL.aa_rx_cl_allow.argtypes = [
    ctypes.POINTER(RxCl), ctypes.c_int, ctypes.c_int, ctypes.c_int
]

LIBAMINOCL.aa_rx_cl_allow_set.argtypes = [
    ctypes.POINTER(RxCl), ctypes.POINTER(RxClSet)
]

LIBAMINOCL.aa_rx_cl_check.argtypes = [
    ctypes.POINTER(RxCl), ctypes.c_size_t,
    ctypes.POINTER(ctypes.c_double), ctypes.c_size_t,
    ctypes.POINTER(RxClSet)
]
LIBAMINOCL.aa_rx_cl_check.restypes = ctypes.c_int

LIBAMINOCL.aa_rx_sg_cl_init.argtypes = [ctypes.POINTER(RxSg)]
