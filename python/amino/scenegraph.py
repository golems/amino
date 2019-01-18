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
    def __init__(self):
        self.ptr = libamino.aa_rx_geom_opt_create()
    def __del__(self):
        libamino.aa_rx_geom_opt_destroy(self.ptr)

    @staticmethod
    def ensure(thing):
        if isinstance(thing, GeomOpt):
            return thing
        elif thing is None:
            return GeomOpt()
        else:
            return Exception()

class Geom(object):
    __slots__ = ['ptr']
    def __init__(self,ptr):
        self.ptr = ptr

    @staticmethod
    def Box(opt, dimension):
        o = GeomOpt.ensure(opt)
        return Geom(libamino.aa_rx_geom_box(o.ptr, Vec3.ensure(dimension)))

libamino.aa_rx_geom_opt_create.argtypes = []
libamino.aa_rx_geom_opt_create.restype = ctypes.POINTER(geom_opt)
libamino.aa_rx_geom_opt_destroy.argtypes = [ctypes.POINTER(geom_opt) ]

libamino.aa_rx_geom_box.argtypes = [ctypes.POINTER(geom_opt), ctypes.POINTER(Vec3)]
libamino.aa_rx_geom_box.restype = ctypes.POINTER(geom)


class SceneGraph:
    __slots__ = ['ptr']

    def __init__(self):
        self.ptr = libamino.aa_rx_sg_create()
    def __del__(self):
        libamino.aa_rx_sg_destroy(self.ptr)

    def add_frame_fixed(self,parent,name,tf):
        if(isinstance(tf,tuple)):
            h,v = tf
        else:
            h = tf.rotation()
            v = tf.translation()

        libamino.aa_rx_sg_add_frame_fixed( self.ptr, parent, name,
                                           Quat.ensure(h), Vec3.ensure(v) )

    def attach_geom(self,name,geom):
        if( geom.ptr ):
            libamino.aa_rx_geom_attach(self.ptr, name, geom.ptr)
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
        else:
            return name_or_id

    def config_vector(self, config_dict, vector=None):
        if vector is None:
            vector = DVec.create(self.config_count())
            vector.set(0)
        for key in config_dict:
            vector[ self.config_id(key) ] = config_dict[key]
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

libamino.aa_rx_dl_sg_at.argtypes = [ ctypes.c_char_p, ctypes.c_char_p,
                                    ctypes.POINTER(sg), ctypes.c_char_p]
libamino.aa_rx_dl_sg_at.restype = ctypes.POINTER(sg)


libamino.aa_rx_sg_add_frame_fixed.argtypes = [ctypes.POINTER(sg),
                                              ctypes.c_char_p, ctypes.c_char_p,
                                              ctypes.POINTER(Quat), ctypes.POINTER(Vec3)]

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


    def scatter_config(self,v_sub, v_all=None):
        if( v_all is None ):
            v_all = DVec.create(self.scenegraph.config_count())
        libamino.aa_rx_sg_sub_config_scatter(self.ptr, v_sub, v_all)
        return v_all

    def gather_config(self,v_all):
        v_sub = DVec.create(self.config_count())
        libamino.aa_rx_sg_sub_config_gather(self.ptr,v_all, v_sub)
        return v_sub

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
