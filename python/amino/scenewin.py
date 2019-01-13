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
## @file scenegraph.py Scene Graphs
##



import ctypes
from lib import libamino
from tf import Vec3,Quat
from scenegraph import *

libaminogl = ctypes.CDLL("libamino-gl.so")


class win(ctypes.Structure):
    """Opaque type for scene window"""
    pass

class SceneWin(object):
    __slots__ = ['ptr', 'scenegraph']

    def __init__(self, title="PyAmino", width=800, height=600,
                 start=True, async=True, scenegraph=None):
        self.ptr = libaminogl.aa_rx_win_default_create(title,width,height)
        if scenegraph:
            self.set_scenegraph(scenegraph)
        if start:
            self.start(async)

    def __del__(self):
        libaminogl.aa_rx_win_destroy( self.ptr )

    def start( self, async=False ):
        if( async ):
            libaminogl.aa_rx_win_run_async()
        else:
            libaminogl.aa_rx_win_run()

    def set_scenegraph(self, scenegraph):
        self.scenegraph = scenegraph
        libaminogl.aa_rx_win_set_sg(self.ptr,scenegraph.ptr)

    def set_config(self,config):
        if isinstance(config,dict):
            self.set_config( self.scenegraph.config_vector(config) )
        elif len(config) != self.scenegraph.config_count():
            raise IndexError()
        else:
            libaminogl.aa_rx_win_set_bconfig(self.ptr,DVec.ensure(config))

    def stop(self):
        libaminogl.aa_rx_win_run_async()
        pass


libaminogl.aa_gl_init.argtypes = []

libaminogl.aa_rx_win_default_create.argtypes = [ ctypes.c_char_p, ctypes.c_int, ctypes.c_int ]
libaminogl.aa_rx_win_default_create.restype = ctypes.POINTER(win)

libaminogl.aa_rx_win_destroy.argtypes = [ctypes.POINTER(win)]

libaminogl.aa_rx_win_set_bconfig.argtypes = [ctypes.POINTER(win), ctypes.POINTER(DVec)]

libaminogl.aa_rx_win_run.argtypes = [ ]
libaminogl.aa_rx_win_run_async.argtypes = [ ]
libaminogl.aa_rx_win_stop.argtypes = [ ctypes.POINTER(win) ]

libaminogl.aa_rx_win_set_sg.argtypes = [ ctypes.POINTER(win), ctypes.POINTER(sg) ]
