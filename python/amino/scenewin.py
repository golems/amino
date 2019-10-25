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
## @file scenewin.py Scene Viewer Window
##
"""Scene Viewer Window"""

import ctypes
from amino.mat import DVec
from amino.scenegraph import RxSg
from amino.kinematics import SceneFK
from amino.util import ensure_cstring

LIBAMINOGL = ctypes.CDLL("libamino-gl.so")


class RxWin(ctypes.Structure):
    """Opaque type for scene window"""
    pass


class SceneWin(object):
    """Scene Viewer Window."""
    __slots__ = ['_ptr', '_scenegraph', '_q']

    def __init__(self,
                 title="PyAmino",
                 width=800,
                 height=600,
                 start=True,
                 background=True,
                 scenegraph=None,
                 config=None):
        """Create a new window.

        Args:
            title: the window title
            width: window width in pixels
            height: window width in pixels
            start: whether to immediatly start the window
            background: whether to run the window in an asynchronous background thread
            scenegraph: scene to display
            config: configuration of the scene
        """
        title = ensure_cstring(title)
        self._ptr = LIBAMINOGL.aa_rx_win_default_create(title, width, height)
        self._q = None
        self._scenegraph = None

        if scenegraph:
            self.scenegraph = scenegraph
        if config:
            self.config = config
        if start:
            self.start(background)

    def __del__(self):
        LIBAMINOGL.aa_rx_win_destroy(self._ptr)

    def start(self, background=False):
        """Start the window.

        Args:
            background: whether to run the window in an asynchronous background thread
        """

        if background:
            LIBAMINOGL.aa_rx_win_run_async()
        else:
            LIBAMINOGL.aa_rx_win_run()

    @property
    def scenegraph(self):
        """The displayed scenegraph."""
        return self._scenegraph

    @scenegraph.setter
    def scenegraph(self, scenegraph):
        self._scenegraph = scenegraph
        self._q = DVec(scenegraph.config_count)
        LIBAMINOGL.aa_rx_win_set_sg(self._ptr, scenegraph._ptr)

    @property
    def config(self):
        """The current configuration."""
        return DVec(self._q)

    @config.setter
    def config(self, config):
        self._q = self._scenegraph.config_vector(config, self._q)
        LIBAMINOGL.aa_rx_win_set_bconfig(self._ptr, self._q)

    @property
    def fk(self):
        """The current configuration FK."""
        fk = SceneFK(self._scenegraph)
        fk.config = self._q
        return fk

    @fk.setter
    def fk(self, value):
        self.config = value.config

    def stop(self):
        """Stop the window."""
        LIBAMINOGL.aa_rx_win_stop(self._ptr)

    def is_runnining(self):
        """Returns True if the window is still running."""
        return bool(LIBAMINOGL.aa_rx_win_is_running(self._ptr))


LIBAMINOGL.aa_gl_init.argtypes = []

LIBAMINOGL.aa_rx_win_default_create.argtypes = [
    ctypes.c_char_p, ctypes.c_int, ctypes.c_int
]
LIBAMINOGL.aa_rx_win_default_create.restype = ctypes.POINTER(RxWin)

LIBAMINOGL.aa_rx_win_destroy.argtypes = [ctypes.POINTER(RxWin)]

LIBAMINOGL.aa_rx_win_set_bconfig.argtypes = [
    ctypes.POINTER(RxWin), ctypes.POINTER(DVec)
]

LIBAMINOGL.aa_rx_win_run.argtypes = []
LIBAMINOGL.aa_rx_win_run_async.argtypes = []
LIBAMINOGL.aa_rx_win_stop.argtypes = [ctypes.POINTER(RxWin)]

LIBAMINOGL.aa_rx_win_set_sg.argtypes = [
    ctypes.POINTER(RxWin), ctypes.POINTER(RxSg)
]

LIBAMINOGL.aa_rx_win_is_running.argtypes = [ctypes.POINTER(RxWin)]
LIBAMINOGL.aa_rx_win_is_running.restype = ctypes.c_int
