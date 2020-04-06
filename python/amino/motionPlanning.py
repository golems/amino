#  Copyright (c) 2019, Colorado School of Mines
#  All rights reserved.
#
#  Author(s): Matthew A. Schack <mschack@mines.edu>
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
## @file motion-planning.py Motion Planning
##
"""Motion Planning"""



import ctypes
from amino.scenegraph import RxSgSub

LIBAMINOMP = ctypes.CDLL("libamino-planning.so")

class RxMp(ctypes.Structure):
    """Opaque type for struct aa_rx_mp"""
    pass


class MotionPlan(object):
    """Context object for motion planning."""

    __slots__ = ["_ptr"]

    def __init__(self, ssg):
        self._ptr = LIBAMINOMP.aa_rx_mp_create(ssg._ptr)
       

    def __del__(self):
        LIBAMINOMP.aa_rx_mp_destroy(self._ptr)


    def motion_plan(self, start, goal, timeout):
        c_start = (ctypes.c_double * len(start))(*start)
        c_goal  = (ctypes.c_double * len(goal))(*goal)
        LIBAMINOMP.aa_rx_mp_set_start(self._ptr, len(start), c_start)
        LIBAMINOMP.aa_rx_mp_set_goal(self._ptr, len(goal), c_goal)

        LIBAMINOMP.aa_rx_mp_set_simplify(self._ptr, ctypes.c_int(1))
        LIBAMINOMP.aa_rx_mp_set_sbl(self._ptr, ctypes.POINTER(ctypes.c_int)())

        n_path = ctypes.c_size_t(0)

        path = ctypes.POINTER(ctypes.c_double)()
        LIBAMINOMP.aa_rx_mp_plan(self._ptr, timeout,ctypes.byref(n_path), ctypes.byref(path))
        py_path = []
        for i in range(0, n_path.value*len(start), len(start)):
            cord = []
            for j in range(0, len(start)):
                cord.append(path[i+j])
            py_path.append(cord)
            
        return py_path
       

LIBAMINOMP.aa_rx_mp_create.argtypes = [ctypes.POINTER(RxSgSub)]
LIBAMINOMP.aa_rx_mp_create.restype = ctypes.POINTER(RxMp)

LIBAMINOMP.aa_rx_mp_destroy.argtypes = [ctypes.POINTER(RxMp)]

LIBAMINOMP.aa_rx_mp_set_simplify.argtypes = [ctypes.POINTER(RxMp), ctypes.c_int]
LIBAMINOMP.aa_rx_mp_set_sbl.argtypes = [ctypes.POINTER(RxMp), ctypes.POINTER(ctypes.c_int)]

LIBAMINOMP.aa_rx_mp_set_start.argtypes = [ctypes.POINTER(RxMp), ctypes.c_int,
                                          ctypes.POINTER(ctypes.c_double)]

LIBAMINOMP.aa_rx_mp_set_goal.argtypes = [ctypes.POINTER(RxMp), ctypes.c_int,
                                         ctypes.POINTER(ctypes.c_double)]

LIBAMINOMP.aa_rx_mp_plan.argtypes = [ctypes.POINTER(RxMp), ctypes.c_double,
                                     ctypes.POINTER(ctypes.c_size_t),
                                     ctypes.POINTER(ctypes.POINTER(ctypes.c_double))]
