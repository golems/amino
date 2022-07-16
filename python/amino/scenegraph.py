#  Copyright (c) 2020, Colorado School of Mines
#  All rights reserved.
#
#  Author(s): Neil T. Dantam <ndantam@mines.edu>
#             Matthew A. Schack <mschack@mines.edu>
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
"""Scene graphs"""

import ctypes
from amino.lib import libamino
from amino.tf import Vec3, Quat, QuatTrans
from amino.mat import DVec, DMat
from amino.util import ensure_cstring, is_string

FRAME_ROOT = ctypes.c_long.in_dll(libamino, "aa_rx_frame_root").value
FRAME_NONE = ctypes.c_long.in_dll(libamino, "aa_rx_frame_none").value
CONFIG_NONE = ctypes.c_long.in_dll(libamino, "aa_rx_config_none").value
CONFIG_MULTI = ctypes.c_long.in_dll(libamino, "aa_rx_config_multi").value


class RxSg(ctypes.Structure):
    """Opaque type for scenegraph pointer."""

class RxSgSub(ctypes.Structure):
    """Opaque type for sub-scenegraph pointer."""

class RxGeom(ctypes.Structure):
    """Opaque type for geometry pointer."""

class RxGeomOpt(ctypes.Structure):
    """Opaque type for geometry opts pointer."""

class GeomOpt:
    """Geometry options object.

       Attributes:

    """
    __slots__ = ['_ptr']

    def __init__(self, options=None):
        """Constructs object with default and specified options.

        Args:
            options: A dictionary of options corresponding to GeomOpt attributes."""
        self._ptr = libamino.aa_rx_geom_opt_create()
        if options:
            for key in options:
                self.__setattr__(key, options[key])

    def __del__(self):
        libamino.aa_rx_geom_opt_destroy(self._ptr)

    def _get_val(self, func):
        return func(self._ptr).value

    def _get_val3(self, func0, func1, func2):
        return [
            self._get_val(func0),
            self._get_val(func1),
            self._get_val(func2)
        ]

    def _get_bool(self, func):
        v = self._get_val(func)
        return bool(v)

    @property
    def visual(self):
        """Boolean indicating whether to use for visual geometry."""
        return self._get_bool(libamino.aa_rx_geom_opt_get_visual)

    @property
    def collision(self):
        """Boolean indicating whether to use for collision geometry."""
        return self._get_bool(libamino.aa_rx_geom_opt_get_collision)

    @property
    def no_shadow(self):
        """Boolean indicating to not display a shadow."""
        return self._get_bool(libamino.aa_rx_geom_opt_get_no_shadow)

    @property
    def alpha(self):
        """Float indicating alpha (transparency)."""
        return self._get_val(libamino.aa_rx_geom_opt_get_alpha)

    @property
    def scale(self):
        """Float indicating how much to scale the geometry."""
        return self._get_val(libamino.aa_rx_geom_opt_get_scale)

    @property
    def color(self):
        """Three floats, between 0 and 1, for color in red-blue-green order."""
        return self._get_val3(libamino.aa_rx_geom_opt_get_color_red,
                              libamino.aa_rx_geom_opt_get_color_blue,
                              libamino.aa_rx_geom_opt_get_color_green)

    @property
    def specular(self):
        """Three floats, between 0 and 1, for color in red-blue-green order."""
        return self._get_val3(libamino.aa_rx_geom_opt_get_specular_red,
                              libamino.aa_rx_geom_opt_get_specular_blue,
                              libamino.aa_rx_geom_opt_get_specular_green)

    @visual.setter
    def visual(self, value):
        libamino.aa_rx_geom_opt_set_visual(self._ptr, value)

    @collision.setter
    def collision(self, value):
        libamino.aa_rx_geom_opt_set_collision(self._ptr, value)

    @no_shadow.setter
    def no_shadow(self, value):
        libamino.aa_rx_geom_opt_set_no_shadow(self._ptr, value)

    @alpha.setter
    def alpha(self, value):
        libamino.aa_rx_geom_opt_set_alpha(self._ptr, value)

    @scale.setter
    def scale(self, value):
        libamino.aa_rx_geom_opt_set_scale(self._ptr, value)

    @color.setter
    def color(self, value):
        libamino.aa_rx_geom_opt_set_color3(self._ptr, value[0], value[1],
                                           value[2])

    @specular.setter
    def specular(self, value):
        libamino.aa_rx_geom_opt_set_specular3(self._ptr, value[0], value[1],
                                              value[2])

    @staticmethod
    def ensure(thing):
        """Ensures thing is a GeomOpt, converting dicts if necessary."""
        if isinstance(thing, GeomOpt):
            return thing
        elif isinstance(thing, dict):
            return GeomOpt(thing)
        elif thing is None:
            return GeomOpt()
        else:
            raise Exception()


class Geom:
    """A geometry object."""

    __slots__ = ['_ptr']

    def __init__(self, _ptr):
        self._ptr = _ptr

    @staticmethod
    def box(opt, dimension):
        """Creates a box geometry object."""
        o = GeomOpt.ensure(opt)
        return Geom(libamino.aa_rx_geom_box(o._ptr, Vec3.ensure(dimension)))

    @staticmethod
    def sphere(opt, radius):
        """Creates a sphere geometry object."""
        o = GeomOpt.ensure(opt)
        return Geom(libamino.aa_rx_geom_sphere(o._ptr, radius))

    @staticmethod
    def cylinder(opt, height, radius):
        """Creates a cylinder geometry object."""
        o = GeomOpt.ensure(opt)
        return Geom(libamino.aa_rx_geom_cylinder(o._ptr, height, radius))

    @staticmethod
    def cone(opt, height, start_radius, end_radius):
        """Creates a cone geometry object."""
        o = GeomOpt.ensure(opt)
        return Geom(
            libamino.aa_rx_geom_cone(o._ptr, height, start_radius, end_radius))

    @staticmethod
    def torus(opt, angle, major_radius, minor_radius):
        """Creates a torus geometry object."""
        o = GeomOpt.ensure(opt)
        return Geom(
            libamino.aa_rx_geom_torus(o._ptr, angle, major_radius,
                                      minor_radius))

    @staticmethod
    def grid(opt, dimension, delta, width):
        """Creates a grid geometry object."""
        o = GeomOpt.ensure(opt)
        c_dimension = (ctypes.c_double * 2)(dimension[0], dimension[1])
        c_delta = (ctypes.c_double * 2)(delta[0], delta[1])
        # c_dimension[0] = dimension[0]
        # c_dimension[1] = dimension[1]
        # c_delta[0] = delta[0]
        # c_delta[1] = delta[1]
        return Geom(
            libamino.aa_rx_geom_grid(o._ptr, c_dimension, c_delta, width))


libamino.aa_rx_geom_opt_create.argtypes = []
libamino.aa_rx_geom_opt_create.restype = ctypes.POINTER(RxGeomOpt)
libamino.aa_rx_geom_opt_destroy.argtypes = [ctypes.POINTER(RxGeomOpt)]

libamino.aa_rx_geom_opt_set_no_shadow.argtypes = [
    ctypes.POINTER(RxGeomOpt), ctypes.c_int
]

libamino.aa_rx_geom_opt_set_visual.argtypes = [
    ctypes.POINTER(RxGeomOpt), ctypes.c_int
]

libamino.aa_rx_geom_opt_set_collision.argtypes = [
    ctypes.POINTER(RxGeomOpt), ctypes.c_int
]

libamino.aa_rx_geom_opt_set_alpha.argtypes = [
    ctypes.POINTER(RxGeomOpt), ctypes.c_double
]

libamino.aa_rx_geom_opt_set_color3.argtypes = [
    ctypes.POINTER(RxGeomOpt), ctypes.c_double, ctypes.c_double,
    ctypes.c_double
]

libamino.aa_rx_geom_opt_set_specular3.argtypes = [
    ctypes.POINTER(RxGeomOpt), ctypes.c_double, ctypes.c_double,
    ctypes.c_double
]

libamino.aa_rx_geom_opt_set_scale.argtypes = [
    ctypes.POINTER(RxGeomOpt), ctypes.c_double
]

libamino.aa_rx_geom_opt_get_scale.argtypes = [ctypes.POINTER(RxGeomOpt)]
libamino.aa_rx_geom_opt_get_scale.restypes = ctypes.c_double

libamino.aa_rx_geom_opt_get_alpha.argtypes = [ctypes.POINTER(RxGeomOpt)]
libamino.aa_rx_geom_opt_get_alpha.restypes = ctypes.c_double

libamino.aa_rx_geom_opt_get_color_red.argtypes = [ctypes.POINTER(RxGeomOpt)]
libamino.aa_rx_geom_opt_get_color_red.restypes = ctypes.c_double
libamino.aa_rx_geom_opt_get_color_blue.argtypes = [ctypes.POINTER(RxGeomOpt)]
libamino.aa_rx_geom_opt_get_color_blue.restypes = ctypes.c_double
libamino.aa_rx_geom_opt_get_color_green.argtypes = [ctypes.POINTER(RxGeomOpt)]
libamino.aa_rx_geom_opt_get_color_green.restypes = ctypes.c_double

libamino.aa_rx_geom_opt_get_specular_red.argtypes = [ctypes.POINTER(RxGeomOpt)]
libamino.aa_rx_geom_opt_get_specular_red.restypes = ctypes.c_double
libamino.aa_rx_geom_opt_get_specular_blue.argtypes = [
    ctypes.POINTER(RxGeomOpt)
]
libamino.aa_rx_geom_opt_get_specular_blue.restypes = ctypes.c_double
libamino.aa_rx_geom_opt_get_specular_green.argtypes = [
    ctypes.POINTER(RxGeomOpt)
]
libamino.aa_rx_geom_opt_get_specular_green.restypes = ctypes.c_double

libamino.aa_rx_geom_box.argtypes = [
    ctypes.POINTER(RxGeomOpt), ctypes.POINTER(Vec3)
]
libamino.aa_rx_geom_box.restype = ctypes.POINTER(RxGeom)

libamino.aa_rx_geom_sphere.argtypes = [
    ctypes.POINTER(RxGeomOpt), ctypes.c_double
]
libamino.aa_rx_geom_sphere.restype = ctypes.POINTER(RxGeom)

libamino.aa_rx_geom_cylinder.argtypes = [
    ctypes.POINTER(RxGeomOpt), ctypes.c_double, ctypes.c_double
]
libamino.aa_rx_geom_cylinder.restype = ctypes.POINTER(RxGeom)

libamino.aa_rx_geom_cone.argtypes = [
    ctypes.POINTER(RxGeomOpt), ctypes.c_double, ctypes.c_double,
    ctypes.c_double
]
libamino.aa_rx_geom_cone.restype = ctypes.POINTER(RxGeom)

libamino.aa_rx_geom_grid.argtypes = [
    ctypes.POINTER(RxGeomOpt),
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double), ctypes.c_double
]
libamino.aa_rx_geom_grid.restype = ctypes.POINTER(RxGeom)

libamino.aa_rx_geom_torus.argtypes = [
    ctypes.POINTER(RxGeomOpt), ctypes.c_double, ctypes.c_double,
    ctypes.c_double
]
libamino.aa_rx_geom_torus.restype = ctypes.POINTER(RxGeom)


class SceneGraph:
    """A scene graph."""
    __slots__ = ['_ptr']

    def __init__(self, ptr=None):
        if ptr is None:
            self._ptr = libamino.aa_rx_sg_create()
        else:
            self._ptr = ptr

    def __del__(self):
        libamino.aa_rx_sg_destroy(self._ptr)

    def add_frame_fixed(self, parent, name, tf=QuatTrans.identity(),
                        geom=None):
        """Adds a new fixed frame to the scene."""
        E = QuatTrans.ensure(tf)
        name = ensure_cstring(name)
        parent = ensure_cstring(parent)
        libamino.aa_rx_sg_add_frame_fixed(self._ptr, parent, name, E.quat,
                                          E.trans)
        self.attach_geom(name, geom)

    def add_frame_revolute(self,
                           parent,
                           name,
                           tf=QuatTrans.identity(),
                           config_name=None,
                           axis=(0, 0, 1),
                           offset=0,
                           geom=None):
        """Adds a new revolute frame to the scene."""
        E = QuatTrans.ensure(tf)
        if config_name is None:
            config_name = name
        name = ensure_cstring(name)
        parent = ensure_cstring(parent)
        config_name = ensure_cstring(config_name)
        libamino.aa_rx_sg_add_frame_revolute(self._ptr, parent, name, E.quat,
                                             E.trans, config_name,
                                             Vec3.ensure(axis), offset)
        self.attach_geom(name, geom)

    def add_frame_prismatic(self,
                            parent,
                            name,
                            tf=QuatTrans.identity(),
                            config_name=None,
                            axis=(0, 0, 1),
                            offset=0,
                            geom=None,
                            limits=None):
        """Adds a new prismatic frame to the scene."""
        E = QuatTrans.ensure(tf)
        if config_name is None:
            config_name = name
        name = ensure_cstring(name)
        parent = ensure_cstring(parent)
        config_name = ensure_cstring(config_name)
        libamino.aa_rx_sg_add_frame_prismatic(self._ptr, parent, name, E.quat,
                                              E.trans, config_name,
                                              Vec3.ensure(axis), offset)
        self.attach_geom(name, geom)

        if limits != None:
            libamino.aa_rx_sg_set_limit_pos(self._ptr, config_name, limits[0], limits[1])

    def remove_frame(self, name):
        """Removes a frame from the scene"""
        libamino.aa_rx_sg_rm_frame(self._ptr, ensure_cstring(name))

    def attach_geom(self, name, geom):
        """Attaches geometry to the named frame."""
        name = ensure_cstring(name)
        if geom is None:
            return
        elif isinstance(geom, Geom):
            libamino.aa_rx_geom_attach(self._ptr, name, geom._ptr)
        elif isinstance(geom, (list, tuple)):
            for elt in geom:
                self.attach_geom(name, elt)
        else:
            raise Exception("Invalid Geom config provided.")

    def reparent(self, new_parent, frame, rel_tf=QuatTrans.identity()):
        """Change the parent of frame in the scenegraph."""
        frame = ensure_cstring(frame)
        new_parent = ensure_cstring(new_parent)
        libamino.aa_rx_sg_reparent_name(self._ptr, new_parent, frame, rel_tf)


    def load(self, filename, name, root=""):
        """Loads a scene plugin into this scene.

        Args:
            filname: plugin name, passed directly to dlopen().
            name: scene name, as provided to the scene compiler.
            root: parent frame for the loaded scene.

        Raises:
            LookupError: the shared object or named scene could not be found.
        """
        root = ensure_cstring(root)
        name = ensure_cstring(name)
        filename = ensure_cstring(filename)
        r = libamino.aa_rx_dl_sg_at(filename, name, self._ptr, root)
        if r is None:
            raise LookupError("Could not load scene %s:%s" % (filename, name))

    def init(self):
        """Initializes the scene, must be called after all frames are added."""
        libamino.aa_rx_sg_init(self._ptr)
        return self

    def allow_collision(self, i, j, allowed=1):
        """Set allowed collisions between frames id0 and id1."""
        id0 = self.ensure_frame_id(i)
        id1 = self.ensure_frame_id(j)
        libamino.aa_rx_sg_allow_collision(self._ptr, id0, id1, allowed)


    @property
    def config_count(self):
        """Number of configuration variables in the scene."""
        return libamino.aa_rx_sg_config_count(self._ptr)

    @property
    def frame_count(self):
        """Number of frames in the scene."""
        return libamino.aa_rx_sg_frame_count(self._ptr)

    def config_id(self, name):
        """Returns the config id for string name."""
        name = ensure_cstring(name)
        return libamino.aa_rx_sg_config_id(self._ptr, name)

    def frame_id(self, name):
        """Returns the frame id for string name."""
        name = ensure_cstring(name)
        return libamino.aa_rx_sg_frame_id(self._ptr, name)

    def config_name(self, i):
        """Returns the config name for the id."""
        name = libamino.aa_rx_sg_config_name(self._ptr, i)
        return str(name, encoding="utf-8")

    def frame_name(self, i):
        """Returns the frame name for the id.

        Raises:
            IndexError: value is out of range.
        """
        if i >= self.frame_count or i < FRAME_NONE:
            raise IndexError("Invalid frame id: %d" % i)

        name = libamino.aa_rx_sg_frame_name(self._ptr, i)
        return str(name, encoding="utf-8")

    def ensure_config_id(self, value):
        """Ensures value is a config id, converting strings if necessary.

        Raises:
            IndexError: value is out range.
            ValueError: config name is invalid.
        """
        if is_string(value):
            if self.config_id(value) == CONFIG_NONE:
                raise ValueError("Invalid config name: %s" % value)
            return self.config_id(value)

        if value >= self.config_count or value < 0:
            raise IndexError("Invalid config id: %d" % value)
        return value

    def ensure_config_name(self, value):
        """Ensures value is a string config name, converting int ids if necessary.

        Raises:
            IndexError: value is out of range.
            ValueError: config name is invalid.
        """
        if is_string(value):
            id = self.config_id(value)
            if id == CONFIG_NONE:
                raise ValueError("Invalid config name: %s" % value)
            return value

        if value >= self.config_count or value < 0:
            raise IndexError("Invalid config id: %d" % value)
        return self.frame_name(value)

    def ensure_frame_id(self, value):
        """Ensures value is a frame id, converting strings if necessary.

        Raises:
            IndexError: value is out range.
            ValueError: frame name is invalid.
        """
        if is_string(value):
            id = self.frame_id(value)
            if id == FRAME_NONE:
                raise ValueError("Invalid frame name: %s" % value)
            return id

        if value >= self.frame_count or value <= FRAME_NONE:
            raise IndexError("Invalid frame id: %d" % value)
        return value

    def ensure_frame_id_actual(self, value):
        """Ensures value is a frame id, converting strings if necessary.

        The difference between this function and ensure_frame_id() is
        that this function throws errors if given reserved frame ids,
        FRAME_ROOT and FRAME_NONE.

        Raises:
            IndexError: value is out range.
        """
        r = self.ensure_frame_id(value)
        if r < 0:
            raise IndexError("Invalid frame id: %d" % value)
        return r

    def ensure_frame_name(self, value):
        """Ensures value is a string frame name, converting int ids if necessary.

        Raises:
            IndexError: value is out of range.
            ValueError: frame name is invalid.
        """
        if is_string(value):
            id = self.frame_id(value)
            if id == FRAME_NONE:
                raise ValueError("Invalid frame name: %s" % value)
            return value

        if value >= self.frame_count or value <= FRAME_NONE:
            raise IndexError("Invalid frame id: %d" % value)
        return self.frame_name(value)

    def config_vector(self, config, vector=None):
        """Create or convert to a configuration vector.

        Raises:
            IndexError: provided vector is the wrong size"""
        if isinstance(config, dict):
            if vector is None:
                vector = DVec(self.config_count)
                vector.set(0)
            elif len(vector) != self.config_count:
                raise IndexError()
            for key in config:
                vector[self.config_id(key)] = config[key]
            return vector
        elif vector is None:
            return DVec.ensure(config)
        else:
            vector.copy_from(config)
            return vector

    def copy_config(self, config):
        """Copy configuration as a vector."""
        return self.config_vector(config, DVec(self.config_count))

    def config_dict(self, vector):
        """Convert vector to a dict."""
        d = {}
        for i in range(0, self.config_count):
            d[self.config_name(i)] = vector[i]
        return d

    def get_parent(self, frame):
        """Get the name of the parent of the input frame"""
        frame_id = self.ensure_frame_id(frame)
        parent = libamino.aa_rx_sg_frame_parent(self._ptr, frame_id)
        return self.ensure_frame_name(parent)

    def __getitem__(self, key):
        """Return a SubSceneGraph for the chain from root to tip.

        Args:
            key: a slice starting at the root and ending at the tip.

        Raises:
            LookupError: invalid slice or frames
        """
        if isinstance(key, slice):
            if key.step is not None:
                raise LookupError("Cannot step scenegraph")
            root_key = "" if key.start is None else key.start
            root = self.ensure_frame_id(root_key)
            tip = self.ensure_frame_id(key.stop)
            ptr = libamino.aa_rx_sg_chain_create(self._ptr, root, tip)
            return SubSceneGraph(self, ptr)

        raise LookupError("Could not get scene graph items")

    def copy(self):
        """Create a copy of the scenegraph"""
        sg_copy = SceneGraph(libamino.aa_rx_sg_copy(self._ptr))
        sg_copy.init()
        return sg_copy



libamino.aa_rx_sg_create.argtypes = []
libamino.aa_rx_sg_create.restype = ctypes.POINTER(RxSg)
libamino.aa_rx_sg_destroy.argtypes = [ctypes.POINTER(RxSg)]
libamino.aa_rx_sg_init.argtypes = [ctypes.POINTER(RxSg)]

libamino.aa_rx_sg_config_count.argtypes = [ctypes.POINTER(RxSg)]
libamino.aa_rx_sg_config_count.restype = ctypes.c_size_t

libamino.aa_rx_sg_frame_count.argtypes = [ctypes.POINTER(RxSg)]
libamino.aa_rx_sg_frame_count.restype = ctypes.c_size_t

libamino.aa_rx_sg_config_id.argtypes = [ctypes.POINTER(RxSg), ctypes.c_char_p]
libamino.aa_rx_sg_config_id.restype = ctypes.c_int

libamino.aa_rx_sg_frame_id.argtypes = [ctypes.POINTER(RxSg), ctypes.c_char_p]
libamino.aa_rx_sg_frame_id.restype = ctypes.c_int

libamino.aa_rx_sg_frame_name.argtypes = [ctypes.POINTER(RxSg), ctypes.c_int]
libamino.aa_rx_sg_frame_name.restype = ctypes.c_char_p

libamino.aa_rx_sg_config_name.argtypes = [ctypes.POINTER(RxSg), ctypes.c_int]
libamino.aa_rx_sg_config_name.restype = ctypes.c_char_p

libamino.aa_rx_sg_reparent_name.argtypes = [ctypes.POINTER(RxSg), ctypes.c_char_p,
                                            ctypes.c_char_p, ctypes.POINTER(QuatTrans)]

libamino.aa_rx_sg_tf.argtypes = [ctypes.POINTER(RxSg), ctypes.c_size_t,
                                 ctypes.POINTER(ctypes.c_double), ctypes.c_size_t,
                                 ctypes.POINTER(ctypes.c_double), ctypes.c_size_t,
                                 ctypes.POINTER(ctypes.c_double), ctypes.c_size_t]

libamino.aa_rx_sg_frame_parent.argtypes = [ctypes.POINTER(RxSg), ctypes.c_int]
libamino.aa_rx_sg_frame_parent.restype = ctypes.c_int


libamino.aa_rx_sg_copy.argtypes = [ctypes.POINTER(RxSg)]
libamino.aa_rx_sg_copy.restype = ctypes.POINTER(RxSg)

libamino.aa_rx_sg_allow_collision.argtypes = [ctypes.POINTER(RxSg), ctypes.c_int,
                                              ctypes.c_int, ctypes.c_int]

libamino.aa_rx_sg_set_limit_pos.argtypes = [ctypes.POINTER(RxSg), ctypes.c_char_p,
                                            ctypes.c_double, ctypes.c_double]

libamino.aa_rx_dl_sg_at.argtypes = [
    ctypes.c_char_p, ctypes.c_char_p,
    ctypes.POINTER(RxSg), ctypes.c_char_p
]
libamino.aa_rx_dl_sg_at.restype = ctypes.POINTER(RxSg)

libamino.aa_rx_sg_add_frame_fixed.argtypes = [
    ctypes.POINTER(RxSg), ctypes.c_char_p, ctypes.c_char_p,
    ctypes.POINTER(Quat),
    ctypes.POINTER(Vec3)
]

libamino.aa_rx_sg_add_frame_revolute.argtypes = [
    ctypes.POINTER(RxSg), ctypes.c_char_p, ctypes.c_char_p,
    ctypes.POINTER(Quat),
    ctypes.POINTER(Vec3), ctypes.c_char_p,
    ctypes.POINTER(Vec3), ctypes.c_double
]

libamino.aa_rx_sg_add_frame_prismatic.argtypes = [
    ctypes.POINTER(RxSg), ctypes.c_char_p, ctypes.c_char_p,
    ctypes.POINTER(Quat),
    ctypes.POINTER(Vec3), ctypes.c_char_p,
    ctypes.POINTER(Vec3), ctypes.c_double
]

libamino.aa_rx_sg_rm_frame.argtypes = [ctypes.POINTER(RxSg), ctypes.c_char_p]

libamino.aa_rx_geom_attach.argtypes = [
    ctypes.POINTER(RxSg), ctypes.c_char_p,
    ctypes.POINTER(RxGeom)
]

libamino.aa_rx_sg_fill_tf_abs.argtypes = [
    ctypes.POINTER(RxSg),
    ctypes.POINTER(DVec),
    ctypes.POINTER(DMat)
]


class SubSceneGraph:
    """A subset of a scene graph"""

    __slots__ = ['_ptr', 'scenegraph']

    def __init__(self, scenegraph, _ptr):
        """Initialize from a scenegraph and sub-scenegraph pointer."""
        self.scenegraph = scenegraph
        self._ptr = _ptr

    def __del__(self):
        libamino.aa_rx_sg_sub_destroy(self._ptr)

    # def jacobian(self, tf):
    #     rows = ctypes.c_size_t()
    #     cols = ctypes.c_size_t()
    #     libamino.aa_rx_sg_sub_jacobian_size(self._ptr,
    #                                         ctypes.byref(rows), ctypes.byref(cols))
    #     J = DMat.create(rows.value, cols.value)
    #     libamino.aa_rx_sg_sub_jacobian(self._ptr,
    #                                    tf.cols(), tf.data(), tf.ld(),
    #                                    J.data(), J.ld())
    #     return J

    @property
    def config_count(self):
        """Returns the number of configuration variables in the sub-scenegraph."""
        return libamino.aa_rx_sg_sub_config_count(self._ptr)

    def scatter_config(self, config_sub, vector_all=None):
        """Copy elements of a sub-scenegraph config into the full config vector."""
        if isinstance(config_sub, dict):
            return self.scenegraph.config_vector(config_sub, vector_all)

        if vector_all is None:
            vector_all = DVec(self.scenegraph.config_count)

        libamino.aa_rx_sg_sub_config_scatter(self._ptr, config_sub, vector_all)
        return vector_all

    def gather_config(self, config_all, vector_sub=None):
        """Copy elements of a full config into the sub config vector."""
        # TODO: convert dictionary
        if vector_sub is None:
            vector_sub = DVec(self.config_count)
        libamino.aa_rx_sg_sub_config_gather(self._ptr, config_all, vector_sub)
        return vector_sub

    @property
    def end_effector_id(self):
        """Returns the frame id of the end-effector."""
        return libamino.aa_rx_sg_sub_frame_ees(self._ptr)[0]

    def config_vector(self, config, vector=None):
        """Convert config to a vector."""
        # TODO: convert dictionary
        if vector is None:
            return DVec.ensure(config)
        else:
            vector.copy_from(config)
            return vector

    # @staticmethod
    # def chain(scenegraph, root, tip):
    #     root = scenegraph.ensure_frame_id(root)
    #     tip = scenegraph.ensure_frame_id(tip)
    #     ptr = libamino.aa_rx_sg_chain_create(scenegraph._ptr, root, tip)
    #     return SubSceneGraph( scenegraph, ptr )


libamino.aa_rx_sg_sub_destroy.argtypes = [ctypes.POINTER(RxSgSub)]

libamino.aa_rx_sg_sub_jacobian_size.argtypes = [
    ctypes.POINTER(RxSgSub),
    ctypes.POINTER(ctypes.c_size_t),
    ctypes.POINTER(ctypes.c_size_t)
]

libamino.aa_rx_sg_sub_jacobian.argtypes = [
    ctypes.POINTER(RxSgSub), ctypes.c_size_t,
    ctypes.POINTER(ctypes.c_double), ctypes.c_size_t,
    ctypes.POINTER(ctypes.c_double), ctypes.c_size_t
]


libamino.aa_rx_sg_sub_config_count.argtypes = [ctypes.POINTER(RxSgSub)]
libamino.aa_rx_sg_sub_config_count.restype = ctypes.c_size_t

libamino.aa_rx_sg_sub_config_scatter.argtypes = [
    ctypes.POINTER(RxSgSub),
    ctypes.POINTER(DVec),
    ctypes.POINTER(DVec)
]

libamino.aa_rx_sg_sub_config_gather.argtypes = [
    ctypes.POINTER(RxSgSub),
    ctypes.POINTER(DVec),
    ctypes.POINTER(DVec)
]

libamino.aa_rx_sg_chain_create.argtypes = [
    ctypes.POINTER(RxSg), ctypes.c_int, ctypes.c_int
]
libamino.aa_rx_sg_chain_create.restype = ctypes.POINTER(RxSgSub)

libamino.aa_rx_sg_sub_frame_ees.argtypes = [ctypes.POINTER(RxSgSub)]
libamino.aa_rx_sg_sub_frame_ees.restype = ctypes.POINTER(ctypes.c_int)

libamino.aa_rx_sg_sub_frame_ee.argtypes = [ctypes.POINTER(RxSgSub)]
libamino.aa_rx_sg_sub_frame_ee.restype = ctypes.c_int
