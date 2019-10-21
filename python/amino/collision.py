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



import ctypes
from lib import libamino
from scenegraph import SceneGraph, sg
from kinematics import SceneFK

libaminocl = ctypes.CDLL("libamino-collision.so")

####################
## Collision Sets ##
####################

class rx_cl_set(ctypes.Structure):
    """Opaque type for struct aa_rx_cl_set pointer"""
    pass

class SceneCollisionSet(object):
    """Set of frame pairs indicating collision between frames."""

    __slots__ = ["ptr", "scenegraph"]

    def __init__(self,scenegraph):
        self.scenegraph = scenegraph
        self.ptr = libaminocl.aa_rx_cl_set_create(self.scenegraph.ptr)

    def __del__(self):
        libaminocl.aa_rx_cl_set_destroy(self.ptr)

    def __getitem__(self, key):
        i,j = key
        i = self.scenegraph.ensure_frame_id_actual(i)
        j = self.scenegraph.ensure_frame_id_actual(j)
        v = libaminocl.aa_rx_cl_set_get(self.ptr, i, j)
        return False if v == 0 else True

    def __setitem__(self, key, item):
        i,j = key
        i = self.scenegraph.ensure_frame_id_actual(i)
        j = self.scenegraph.ensure_frame_id_actual(j)
        v = 1 if item else 0
        libaminocl.aa_rx_cl_set_set(self.ptr, i, j, v)

    def clear(self):
        """Set all entries to False."""
        libaminocl.aa_rx_cl_set_clear(self.ptr)

    def fill(self, src):
        """Fill self with all true entries in other."""
        libaminocl.aa_rx_cl_set_fill(self.ptr, src.ptr)


libaminocl.aa_rx_cl_set_create.argtypes = [ctypes.POINTER(sg)]
libaminocl.aa_rx_cl_set_create.restype = ctypes.POINTER(rx_cl_set)

libaminocl.aa_rx_cl_set_destroy.argtypes = [ctypes.POINTER(rx_cl_set)]


libaminocl.aa_rx_cl_set_get.argtypes = [ctypes.POINTER(rx_cl_set), ctypes.c_int, ctypes.c_int]
libaminocl.aa_rx_cl_set_get.restypes = ctypes.c_int

libaminocl.aa_rx_cl_set_set.argtypes = [ctypes.POINTER(rx_cl_set), ctypes.c_int, ctypes.c_int,
                                        ctypes.c_int ]

libaminocl.aa_rx_cl_set_clear.argtypes = [ctypes.POINTER(rx_cl_set)]

libaminocl.aa_rx_cl_set_fill.argtypes = [ctypes.POINTER(rx_cl_set),ctypes.POINTER(rx_cl_set)]

#######################
## Collision Context ##
#######################

class rx_cl(ctypes.Structure):
    """Opaque type for struct aa_rx_cl pointer"""
    pass

class SceneCollision(object):
    """Context object for collision detection."""

    __slots__ = ["ptr", "scenegraph"]

    def __init__(self,scenegraph):
        self.scenegraph = scenegraph
        libaminocl.aa_rx_sg_cl_init(self.scenegraph.ptr)
        self.ptr = libaminocl.aa_rx_cl_create(self.scenegraph.ptr)

    def __del__(self):
        libaminocl.aa_rx_cl_destroy(self.ptr)

    def allow(self, i, j):
        """Allow collisions between frames i and j."""
        i = self.scenegraph.ensure_frame_id_actual(i)
        j = self.scenegraph.ensure_frame_id_actual(j)
        libaminocl.aa_rx_cl_allow(self.ptr, i, j)

    def allow_set(self, collision_set):
        """Allow collisions between all frames in the collision set."""
        libaminocl.aa_rx_cl_allow_set(self.ptr, collision_set.ptr)

    def allow_config(self, config):
        """Allow collisions between all frames coliding at config."""
        fk = SceneFK(self.scenegraph)
        fk.update(config)
        cl_set = SceneCollisionSet(self.scenegraph)
        self.check(fk, cl_set)
        self.allow_set(cl_set)

    def check(self, fk, collision_set = None):
        """Check whether the scene is in collision."""
        cl_set_ptr = None
        if collision_set:
            collision_set.clear()
            cl_set_ptr = collision_set.ptr
        r = libaminocl.aa_rx_cl_check(self.ptr,
                                      len(fk), fk._data_ptr(), fk._data_ld(),
                                      cl_set_ptr )
        return False if 0 == r else True


libaminocl.aa_rx_cl_create.argtypes = [ctypes.POINTER(sg)]
libaminocl.aa_rx_cl_create.restype = ctypes.POINTER(rx_cl)

libaminocl.aa_rx_cl_destroy.argtypes = [ctypes.POINTER(rx_cl)]

libaminocl.aa_rx_cl_allow.argtypes = [ ctypes.POINTER(rx_cl), ctypes.c_int, ctypes.c_int,
                                       ctypes.c_int ]

libaminocl.aa_rx_cl_allow_set.argtypes = [ ctypes.POINTER(rx_cl), ctypes.POINTER(rx_cl_set) ]

libaminocl.aa_rx_cl_check.argtypes = [ ctypes.POINTER(rx_cl),
                                       ctypes.c_size_t,
                                       ctypes.POINTER(ctypes.c_double),
                                       ctypes.c_size_t,
                                       ctypes.POINTER(rx_cl_set) ]
libaminocl.aa_rx_cl_check.restypes = ctypes.c_int

libaminocl.aa_rx_sg_cl_init.argtypes = [ctypes.POINTER(sg)]
