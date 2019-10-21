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
from tf import Vec3,Quat,QuatTrans
from mat import DVec, DMat


FRAME_ROOT=-1
FRAME_NONE=-2
CONFIG_NONE=-1

class sg(ctypes.Structure):
    """Opaque type for scenegraph pointer"""
    pass

class sg_sub(ctypes.Structure):
    """Opaque type for sub-scenegraph pointer"""
    pass

class geom(ctypes.Structure):
    """Opaque type for geometry object"""
    pass

class geom_opt(ctypes.Structure):
    """Opaque type for geometry opts object"""
    pass

class GeomOpt(object):
    __slots__ = ['ptr']
    def __init__(self,options={}):
        self.ptr = libamino.aa_rx_geom_opt_create()
        for key in options:
            self.__setattr__(key,options[key])

    def __del__(self):
        libamino.aa_rx_geom_opt_destroy(self.ptr)


    def _get_val(self,func):
        return func(self.ptr).value

    def _get_val3(self,func0, func1, func2):
        return [self._get_val(func0),
                self._get_val(func1),
                self._get_val(func2)]

    def _get_bool(self,func):
        if 0 == self.get_val(func):
            return False
        else:
            return True

    def __getattr__(self, name):
        if 'visual' == name:
            return self._get_bool(libamino.aa_rx_geom_opt_get_visual)
        elif 'collision' == name:
            return self._get_bool(libamino.aa_rx_geom_opt_get_collision)
        elif 'no_shadow' == name:
            return self._get_bool(libamino.aa_rx_geom_opt_get_no_shadow)
        elif 'alpha' == name:
            return self._get_val(libamino.aa_rx_geom_opt_get_alpha)
        elif 'scale' == name:
            return self._get_val(libamino.aa_rx_geom_opt_get_scale)
        elif 'color' == name:
            return self._get_val3( libamino.aa_rx_geom_opt_get_color_red,
                                   libamino.aa_rx_geom_opt_get_color_blue,
                                   libamino.aa_rx_geom_opt_get_color_green )
        elif 'specular' == name:
            return self._get_val3( libamino.aa_rx_geom_opt_get_specular_red,
                                   libamino.aa_rx_geom_opt_get_specular_blue,
                                   libamino.aa_rx_geom_opt_get_specular_green )
        # elif 'ptr' == name:
        #     return object.__getattr__(self,name)
        else:
            raise AttributeError("Undefine attribute: " + name)

    def __setattr__(self, name, value):
        if 'visual' == name:
            libamino.aa_rx_geom_opt_set_visual(self.ptr, value)
        elif 'collision' == name:
            libamino.aa_rx_geom_opt_set_collision(self.ptr, value)
        elif 'no_shadow' == name:
            libamino.aa_rx_geom_opt_set_no_shadow(self.ptr, value)
        elif 'alpha' == name:
            libamino.aa_rx_geom_opt_set_alpha(self.ptr, value)
        elif 'scale' == name:
            libamino.aa_rx_geom_opt_set_scale(self.ptr, value)
        elif 'color' == name:
            libamino.aa_rx_geom_opt_set_color3(self.ptr,
                                               value[0], value[1], value[2])
        elif 'specular' == name:
            libamino.aa_rx_geom_opt_set_specular3(self.ptr,
                                                  value[0], value[1], value[2])
        elif 'ptr' == name:
            object.__setattr__(self,name,value)
        else:
            raise AttributeError("Undefine attribute: " + name)


    @staticmethod
    def ensure(thing):
        if isinstance(thing, GeomOpt):
            return thing
        elif isinstance(thing, dict):
            return GeomOpt(thing)
        elif thing is None:
            return GeomOpt()
        else:
            raise Exception()

class Geom(object):
    __slots__ = ['ptr']
    def __init__(self,ptr):
        self.ptr = ptr

    @staticmethod
    def Box(opt, dimension):
        """Create a box geometry object."""
        o = GeomOpt.ensure(opt)
        return Geom(libamino.aa_rx_geom_box(o.ptr, Vec3.ensure(dimension)))

    @staticmethod
    def Sphere(opt, radius):
        """Create a sphere geometry object."""
        o = GeomOpt.ensure(opt)
        return Geom(libamino.aa_rx_geom_sphere(o.ptr, radius))

    @staticmethod
    def Cylinder(opt, height, radius):
        """Create a cylinder geometry object."""
        o = GeomOpt.ensure(opt)
        return Geom(libamino.aa_rx_geom_cylinder(o.ptr, height, radius))

    @staticmethod
    def Cone(opt, height, start_radius, end_radius):
        """Create a cone geometry object."""
        o = GeomOpt.ensure(opt)
        return Geom(libamino.aa_rx_geom_cone(o.ptr, height, start_radius, end_radius))

    @staticmethod
    def Torus(opt, angle, major_radius, minor_radius):
        """Create a torus geometry object."""
        o = GeomOpt.ensure(opt)
        return Geom(libamino.aa_rx_geom_torus(o.ptr, height, major_radius, minor_radius))

    @staticmethod
    def Grid(opt, dimension, delta, width):
        """Create a grid geometry object."""
        o = GeomOpt.ensure(opt)
        c_dimension = (ctypes.c_double*2)(dimension[0], dimension[1])
        c_delta = (ctypes.c_double*2)(delta[0], delta[1])
        # c_dimension[0] = dimension[0]
        # c_dimension[1] = dimension[1]
        # c_delta[0] = delta[0]
        # c_delta[1] = delta[1]
        return Geom(libamino.aa_rx_geom_grid(o.ptr, c_dimension, c_delta, width))

libamino.aa_rx_geom_opt_create.argtypes = []
libamino.aa_rx_geom_opt_create.restype = ctypes.POINTER(geom_opt)
libamino.aa_rx_geom_opt_destroy.argtypes = [ctypes.POINTER(geom_opt) ]

libamino.aa_rx_geom_opt_set_no_shadow.argtypes = [ctypes.POINTER(geom_opt),
                                                  ctypes.c_int ]

libamino.aa_rx_geom_opt_set_visual.argtypes = [ctypes.POINTER(geom_opt),
                                                  ctypes.c_int ]

libamino.aa_rx_geom_opt_set_collision.argtypes = [ctypes.POINTER(geom_opt),
                                                  ctypes.c_int ]

libamino.aa_rx_geom_opt_set_alpha.argtypes = [ctypes.POINTER(geom_opt),
                                              ctypes.c_double ]

libamino.aa_rx_geom_opt_set_color3.argtypes = [ctypes.POINTER(geom_opt),
                                               ctypes.c_double,
                                               ctypes.c_double,
                                               ctypes.c_double ]

libamino.aa_rx_geom_opt_set_specular3.argtypes = [ctypes.POINTER(geom_opt),
                                                  ctypes.c_double,
                                                  ctypes.c_double,
                                                  ctypes.c_double ]

libamino.aa_rx_geom_opt_set_scale.argtypes = [ctypes.POINTER(geom_opt),
                                              ctypes.c_double ]

libamino.aa_rx_geom_opt_get_scale.argtypes = [ctypes.POINTER(geom_opt)]
libamino.aa_rx_geom_opt_get_scale.restypes = ctypes.c_double

libamino.aa_rx_geom_opt_get_alpha.argtypes = [ctypes.POINTER(geom_opt)]
libamino.aa_rx_geom_opt_get_alpha.restypes = ctypes.c_double

libamino.aa_rx_geom_opt_get_color_red.argtypes = [ctypes.POINTER(geom_opt)]
libamino.aa_rx_geom_opt_get_color_red.restypes = ctypes.c_double
libamino.aa_rx_geom_opt_get_color_blue.argtypes = [ctypes.POINTER(geom_opt)]
libamino.aa_rx_geom_opt_get_color_blue.restypes = ctypes.c_double
libamino.aa_rx_geom_opt_get_color_green.argtypes = [ctypes.POINTER(geom_opt)]
libamino.aa_rx_geom_opt_get_color_green.restypes = ctypes.c_double

libamino.aa_rx_geom_opt_get_specular_red.argtypes = [ctypes.POINTER(geom_opt)]
libamino.aa_rx_geom_opt_get_specular_red.restypes = ctypes.c_double
libamino.aa_rx_geom_opt_get_specular_blue.argtypes = [ctypes.POINTER(geom_opt)]
libamino.aa_rx_geom_opt_get_specular_blue.restypes = ctypes.c_double
libamino.aa_rx_geom_opt_get_specular_green.argtypes = [ctypes.POINTER(geom_opt)]
libamino.aa_rx_geom_opt_get_specular_green.restypes = ctypes.c_double

libamino.aa_rx_geom_box.argtypes = [ctypes.POINTER(geom_opt), ctypes.POINTER(Vec3)]
libamino.aa_rx_geom_box.restype = ctypes.POINTER(geom)


libamino.aa_rx_geom_sphere.argtypes = [ctypes.POINTER(geom_opt), ctypes.c_double]
libamino.aa_rx_geom_sphere.restype = ctypes.POINTER(geom)

libamino.aa_rx_geom_cylinder.argtypes = [ctypes.POINTER(geom_opt),
                                         ctypes.c_double, ctypes.c_double]
libamino.aa_rx_geom_cylinder.restype = ctypes.POINTER(geom)

libamino.aa_rx_geom_cone.argtypes = [ctypes.POINTER(geom_opt),
                                     ctypes.c_double, ctypes.c_double, ctypes.c_double ]
libamino.aa_rx_geom_cone.restype = ctypes.POINTER(geom)


libamino.aa_rx_geom_grid.argtypes = [ctypes.POINTER(geom_opt),
                                     ctypes.POINTER(ctypes.c_double),
                                     ctypes.POINTER(ctypes.c_double),
                                     ctypes.c_double ]
libamino.aa_rx_geom_grid.restype = ctypes.POINTER(geom)


libamino.aa_rx_geom_torus.argtypes = [ctypes.POINTER(geom_opt),
                                     ctypes.c_double, ctypes.c_double, ctypes.c_double ]
libamino.aa_rx_geom_torus.restype = ctypes.POINTER(geom)


class SceneGraph:
    """A scene graph"""
    __slots__ = ['ptr']

    def __init__(self):
        self.ptr = libamino.aa_rx_sg_create()
    def __del__(self):
        libamino.aa_rx_sg_destroy(self.ptr)

    def add_frame_fixed(self,parent,name,
                        tf=QuatTrans.identity(),
                        geom=None ):
        """Add a fixed frame to the scene"""
        E = QuatTrans.ensure(tf)
        libamino.aa_rx_sg_add_frame_fixed(self.ptr, parent, name,
                                          E.quat, E.trans)
        self.attach_geom(name,geom)


    def add_frame_revolute(self,parent,name,
                           tf=QuatTrans.identity(),
                           config_name=None,
                           axis=(0, 0, 1),
                           offset=0,
                           geom=None ):
        """Add a revolute frame to the scene"""
        E = QuatTrans.ensure(tf)
        if config_name is None: config_name = name
        libamino.aa_rx_sg_add_frame_revolute(self.ptr, parent, name,
                                             E.quat, E.trans,
                                             config_name, Vec3.ensure(axis), offset)
        self.attach_geom(name,geom)

    def add_frame_prismatic(self,parent,name,
                            tf=QuatTrans.identity(),
                            config_name=None,
                            axis=(0, 0, 1),
                            offset=0,
                            geom=None ):
        """Add a prismatic frame to the scene"""
        E = QuatTrans.ensure(tf)
        if config_name is None: config_name = name
        libamino.aa_rx_sg_add_frame_prismatic(self.ptr, parent, name,
                                              E.quat, E.trans,
                                              config_name, Vec3.ensure(axis), offset)
        self.attach_geom(name,geom)

    def attach_geom(self,name,geom):
        """Attach geometry to the named frame"""
        if( isinstance(geom,Geom) and geom.ptr ):
            libamino.aa_rx_geom_attach(self.ptr, name, geom.ptr)
        elif geom is None:
            pass
        elif isinstance(geom, list) or isinstance(geom, tuple):
            for elt in geom:
                self.attach_geom(name,elt)
        else:
            raise Exception()


    def get_tf_abs(self,q):
        if isinstance(q,dict) :
            q = self.config_vector(q)
        T = DMat.create(7, self.frame_count())
        libamino.aa_rx_sg_fill_tf_abs(self.ptr, q, T)
        return T

    def load(self, filename, name, root=""):
        r = libamino.aa_rx_dl_sg_at(filename,name,self.ptr,root)
        return self

    def init(self):
        libamino.aa_rx_sg_init(self.ptr)
        return self

    def config_count(self):
        return libamino.aa_rx_sg_config_count(self.ptr)

    def frame_count(self):
        return libamino.aa_rx_sg_frame_count(self.ptr)

    def config_id(self,name):
        return libamino.aa_rx_sg_config_id(self.ptr,name)

    def frame_id(self,name):
        return libamino.aa_rx_sg_frame_id(self.ptr,name)

    def ensure_config_id(self,name_or_id):
        if isinstance(name_or_id,basestring):
            return self.config_id(name_or_id)
        else:
            return name_or_id

    def ensure_frame_id(self,name_or_id):
        if isinstance(name_or_id,basestring):
            return self.frame_id(name_or_id)
        elif name_or_id >= self.frame_count:
            raise IndexError("Invalid frame id: %d" % name_or_id)
        else:
            return name_or_id

    def ensure_frame_id_actual(self,name_or_id):
        r = self.ensure_frame_id(name_or_id)
        if r < 0:
            raise IndexError("Not an actual frame")
        return r

    def ensure_frame_name(self,name_or_id):
        if isinstance(name_or_id,basestring):
            return name_or_id
        elif name_or_id is None:
            return FRAME_NONE
        else:
            return libamino.aa_rx_sg_frame_name(self.ptr,name_or_id)

    def config_vector(self,config,vector=None):
        if( isinstance(config,dict) ):
            if vector is None:
                vector = DVec.create(self.config_count())
                vector.set(0)
            elif len(vector) != self.config_count():
                raise IndexError()
            for key in config:
                vector[ self.config_id(key) ] = config[key]
            return vector
        elif vector is None:
            return DVec.ensure(config)
        else:
            vector.copy_from(config)
            return vector

    def chain(self,root,tip):
        return SubSceneGraph.chain(self,root,tip)

libamino.aa_rx_sg_create.argtypes = []
libamino.aa_rx_sg_create.restype = ctypes.POINTER(sg)
libamino.aa_rx_sg_destroy.argtypes = [ctypes.POINTER(sg) ]
libamino.aa_rx_sg_init.argtypes = [ctypes.POINTER(sg) ]

libamino.aa_rx_sg_config_count.argtypes = [ctypes.POINTER(sg) ]
libamino.aa_rx_sg_config_count.restype = ctypes.c_size_t

libamino.aa_rx_sg_frame_count.argtypes = [ctypes.POINTER(sg) ]
libamino.aa_rx_sg_frame_count.restype = ctypes.c_size_t

libamino.aa_rx_sg_config_id.argtypes = [ctypes.POINTER(sg), ctypes.c_char_p ]
libamino.aa_rx_sg_config_id.restype = ctypes.c_int

libamino.aa_rx_sg_frame_id.argtypes = [ctypes.POINTER(sg), ctypes.c_char_p ]
libamino.aa_rx_sg_frame_id.restype = ctypes.c_int

libamino.aa_rx_sg_frame_name.argtypes = [ctypes.POINTER(sg), ctypes.c_int]
libamino.aa_rx_sg_frame_name.restype = ctypes.c_char_p

libamino.aa_rx_sg_config_name.argtypes = [ctypes.POINTER(sg), ctypes.c_int]
libamino.aa_rx_sg_config_name.restype = ctypes.c_char_p

libamino.aa_rx_dl_sg_at.argtypes = [ ctypes.c_char_p, ctypes.c_char_p,
                                    ctypes.POINTER(sg), ctypes.c_char_p]
libamino.aa_rx_dl_sg_at.restype = ctypes.POINTER(sg)

libamino.aa_rx_sg_add_frame_fixed.argtypes = [ctypes.POINTER(sg),
                                              ctypes.c_char_p, ctypes.c_char_p,
                                              ctypes.POINTER(Quat), ctypes.POINTER(Vec3)]

libamino.aa_rx_sg_add_frame_revolute.argtypes = [ctypes.POINTER(sg),
                                                 ctypes.c_char_p, ctypes.c_char_p,
                                                 ctypes.POINTER(Quat), ctypes.POINTER(Vec3),
                                                 ctypes.c_char_p, ctypes.POINTER(Vec3),
                                                 ctypes.c_double]

libamino.aa_rx_sg_add_frame_prismatic.argtypes = [ctypes.POINTER(sg),
                                                  ctypes.c_char_p, ctypes.c_char_p,
                                                  ctypes.POINTER(Quat), ctypes.POINTER(Vec3),
                                                  ctypes.c_char_p, ctypes.POINTER(Vec3),
                                                  ctypes.c_double]

libamino.aa_rx_geom_attach.argtypes = [ctypes.POINTER(sg), ctypes.c_char_p,
                                       ctypes.POINTER(geom)]

libamino.aa_rx_sg_fill_tf_abs.argtypes = [ctypes.POINTER(sg), ctypes.POINTER(DVec),
                                          ctypes.POINTER(DMat)]





class SubSceneGraph:
    __slots__ = ['ptr', 'scenegraph']

    def __init__(self,scenegraph,ptr):
        self.scenegraph = scenegraph
        self.ptr = ptr

    def __del__(self):
        libamino.aa_rx_sg_sub_destroy(self.ptr)

    def jacobian(self, tf):
        rows = ctypes.c_size_t()
        cols = ctypes.c_size_t()
        libamino.aa_rx_sg_sub_jacobian_size(self.ptr,
                                            ctypes.byref(rows), ctypes.byref(cols))
        J = DMat.create(rows.value,cols.value)
        libamino.aa_rx_sg_sub_jacobian(self.ptr,
                                       tf.cols(), tf.data(), tf.ld(),
                                       J.data(), J.ld())
        return J

    def config_count(self):
        return libamino.aa_rx_sg_sub_config_count(self.ptr)

    def config_id(self,key):
        return libamino.aa_rx_sg_sub_config_count(self.ptr)

    def scatter_config(self,v_sub, v_all=None):
        if( v_all is None ):
            v_all = DVec.create(self.scenegraph.config_count())
        libamino.aa_rx_sg_sub_config_scatter(self.ptr, v_sub, v_all)
        return v_all

    def gather_config(self,v_all):
        v_sub = DVec.create(self.config_count())
        libamino.aa_rx_sg_sub_config_gather(self.ptr,v_all, v_sub)
        return v_sub

    def end_effector_id(self):
        return libamino.aa_rx_sg_sub_frame_ee(self.ptr)

    def config_vector(self,config,vector=None):
        # TODO: convert dictionary
        if vector is None:
            return DVec.ensure(config)
        else:
            vector.copy_from(config)
            return vector


    @staticmethod
    def chain(scenegraph,root,tip):
        root = scenegraph.ensure_frame_id(root)
        tip = scenegraph.ensure_frame_id(tip)
        ptr = libamino.aa_rx_sg_chain_create(scenegraph.ptr, root, tip)
        return SubSceneGraph( scenegraph, ptr )

libamino.aa_rx_sg_sub_destroy.argtypes = [ctypes.POINTER(sg_sub)]

libamino.aa_rx_sg_sub_jacobian_size.argtypes = [ctypes.POINTER(sg_sub),
                                                ctypes.POINTER(ctypes.c_size_t),
                                                ctypes.POINTER(ctypes.c_size_t) ]

libamino.aa_rx_sg_sub_jacobian.argtypes = [ctypes.POINTER(sg_sub),
                                           ctypes.c_size_t, ctypes.POINTER(ctypes.c_double),
                                           ctypes.c_size_t,
                                           ctypes.POINTER(ctypes.c_double),
                                           ctypes.c_size_t]

libamino.aa_rx_sg_sub_config_count.argtypes = [ctypes.POINTER(sg_sub) ]
libamino.aa_rx_sg_sub_config_count.restype = ctypes.c_size_t

libamino.aa_rx_sg_sub_config_scatter.argtypes = [ctypes.POINTER(sg_sub),
                                                 ctypes.POINTER(DVec), ctypes.POINTER(DVec) ]

libamino.aa_rx_sg_sub_config_gather.argtypes = [ctypes.POINTER(sg_sub),
                                                ctypes.POINTER(DVec), ctypes.POINTER(DVec) ]


libamino.aa_rx_sg_chain_create.argtypes = [ctypes.POINTER(sg), ctypes.c_int, ctypes.c_int]
libamino.aa_rx_sg_chain_create.restype = ctypes.POINTER(sg_sub)

libamino.aa_rx_sg_sub_frame_ee.argtypes = [ctypes.POINTER(sg_sub)]
libamino.aa_rx_sg_sub_frame_ee.restype = ctypes.c_int

libamino.aa_rx_sg_sub_frame_ee.argtypes = [ctypes.POINTER(sg_sub)]
libamino.aa_rx_sg_sub_frame_ee.restype = ctypes.c_int
