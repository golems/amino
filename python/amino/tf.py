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



import ctypes
from lib import libamino

from math import sqrt

########
## TF ##
########

class Vec3(ctypes.Structure):
    _fields_ = [ ("x", ctypes.c_double),
                 ("y", ctypes.c_double),
                 ("z", ctypes.c_double) ]

    def __init__(self,v):
        if v is None:
            pass
        else:
            self.copy_from(v)

    @staticmethod
    def ensure(thing):
        if( isinstance(thing,Vec3) ):
            return thing
        else:
            return Vec3(thing)

    def copy_from(self,v):
        if( len(v) != 3 ):
            raise IndexError()
        self.x = v[0]
        self.y = v[1]
        self.z = v[2]

    def ssd(self,other):
        return libamino.aa_tf_vssd(self, Vec3.ensure(other))

    def nrm2(self):
        return libamino.aa_tf_vnorm(self)

    def __eq__(self,other):
        return (0 == self.ssd(other))
    def __ne__(self,other):
        return (0 != self.ssd(other))

    def __getitem__(self, key):
        if key == 0:
            return self.x
        elif key == 1:
            return self.y
        elif key == 2:
            return self.z
        else:
            raise IndexError(key)

    def __setitem__(self, key, item):
        if key == 0:
            self.x = item
        elif key == 1:
            self.y = item
        elif key == 2:
            self.z = item
        else:
            raise IndexError(key)

    def cross(self,other):
        r = Vec3(None)
        libamino.aa_tf_cross(self,Vec3.ensure(other),r)
        return r

    def dot(self,other):
        return libamino.aa_tf_vdot(self,Vec3.ensure(other))

    def __len__(self):
        return 3

    def __str__(self):
        return 'Vec3(%f, %f, %f)' % (self.x, self.y, self.z)


class XAngle(ctypes.Structure):
    _fields_ = [ ("value", ctypes.c_double) ]

class YAngle(ctypes.Structure):
    _fields_ = [ ("value", ctypes.c_double) ]

class ZAngle(ctypes.Structure):
    _fields_ = [ ("value", ctypes.c_double) ]

class Quat(ctypes.Structure):
    _fields_ = [ ("x", ctypes.c_double),
                 ("y", ctypes.c_double),
                 ("z", ctypes.c_double),
                 ("w", ctypes.c_double), ]

    def __init__(self, v):
        if v is None:
            pass
        else:
            self.conv(v)

    def conv_from(self,v):
        if isinstance(v, Quat):
            self.x = v.x
            self.y = v.y
            self.z = v.z
            self.w = v.w
        elif isinstance(v, XAngle):
            libamino.aa_tf_xangle2quat(v.value,self)
        elif isinstance(v, YAngle):
            libamino.aa_tf_yangle2quat(v.value,self)
        elif isinstance(v, ZAngle):
            libamino.aa_tf_zangle2quat(v.value,self)
        elif isinstance(v, RotMat):
            libamino.aa_tf_rotmat2quat(v,self)
        elif type(v) == int or type(v) == float:
            self.x = 0
            self.y = 0
            self.z = 0
            self.w = v
        elif isinstance(v, Vec3):
            self.x = v.x
            self.y = v.y
            self.z = v.z
            self.w = 0
        elif isinstance(v, list) or isinstance(v,tuple):
            if( len(v) != 4 ):
                raise IndexError(key)
            self.x = v[0]
            self.y = v[1]
            self.z = v[2]
            self.w = v[3]
        else:
            raise Exception('Invalid argument')
        return self

    def vector(self):
        return Vec3(self.x,self.y,self.z)

    def scalar(self):
        return self.w

    def ssd(self,other):
        if( len(other) != 4 ):
            raise IndexError()
        return sqrt( (self.x - other[0])**2 +
                     (self.y - other[1])**2 +
                     (self.z - other[2])**2 +
                     (self.w - other[3])**2 )

    def __add__(self,other):
        h = Quat(None)
        libamino.aa_tf_qadd(self,other,h)
        return h

    def __sub__(self,other):
        h = Quat(None)
        libamino.aa_tf_qsub(self,other,h)
        return h

    def __mul__(self,other):
        h = Quat(None)
        libamino.aa_tf_qmul(self,other,h)
        return h

    def __getitem__(self, key):
        if key == 0:
            return self.x
        elif key == 1:
            return self.y
        elif key == 2:
            return self.z
        elif key == 3:
            return self.w
        else:
            raise IndexError(key)

    def __setitem__(self, key, item):
        if key == 0:
            self.x = item
        elif key == 1:
            self.y = item
        elif key == 2:
            self.z = item
        elif key == 3:
            self.z = item
        else:
            raise IndexError(key)

    def __len__(self):
        return 4

    def conj(self):
        h = Quat(None)
        libamino.aa_tf_qconj(self,h)
        return h

    def inv(self):
        h = Quat(None)
        libamino.aa_tf_qinv(self,h)
        return h

    def exp(self):
        h = Quat(None)
        libamino.aa_tf_qexp(self,h)
        return h

    def ln(self):
        h = Quat(None)
        libamino.aa_tf_qln(self,h)
        return h


    def __str__(self):
        return 'Quat(%f, %f, %f, %f)' % (self.x, self.y, self.z, self.w)


class RotMat(ctypes.Structure):
    _fields_ = [ ("cx", Vec3),
                 ("cy", Vec3),
                 ("cz", Vec3) ]

    def __init__(self, v):
        if v is None:
            pass
        elif isinstance(v, Quat):
            libamino.aa_tf_quat2rotmat(v,self)
        elif isinstance(v, RotMat):
            self.cx = v.cx
            self.cy = v.cy
            self.cz = v.cz
        elif isinstance(v, XAngle):
            libamino.aa_tf_xangle2rotmat(v.value,self)
        elif isinstance(v, YAngle):
            libamino.aa_tf_yangle2rotmat(v.value,self)
        elif isinstance(v, ZAngle):
            libamino.aa_tf_zangle2rotmat(v.value,self)
        elif 1 == v:
            self.cx = Vec3(1,0,0)
            self.cy = Vec3(0,1,0)
            self.cz = Vec3(0,0,1)
        else:
            raise Exception('Invalid argument')

class TfMat(ctypes.Structure):
    _fields_ = [ ("R", RotMat),
                 ("v", Vec3) ]

    def __init__(self, v):
        if isinstance(v,tuple):
            self.R = RotMat(v[0])
            self.v = Vec3(v[1])
        else:
            raise Exception('Invalid argument')

    def rotation(self):
        return self.R

    def translation(self):
        return self.v

class DualQuat(ctypes.Structure):
    _fields_ = [ ("real", Quat),
                 ("dual", Quat) ]

    def __init__(self, v):
        if v is None:
            pass
        elif isinstance(v, list):
            self.real.x = v[0]
            self.real.y = v[1]
            self.real.z = v[2]
            self.real.w = v[3]
            self.dual.x = v[4]
            self.dual.y = v[5]
            self.dual.z = v[6]
            self.dual.w = v[7]
        elif isinstance(v,tuple):
            libamino.aa_tf_qv2duqu(Quat(v[0]),Vec3(v[1]),self)
        else:
            raise Exception('Invalid argument')

    def __str__(self):
        return 'DualQuat(%s, %s)' % (self.real, self.dual)

    def rotation(self):
        return self.real

    def translation(self):
        v = Vec3(None)
        libamino.aa_tf_duqu_trans(self,v)
        return v

class QuatTrans(ctypes.Structure):
    _fields_ = [ ("quat", Quat),
                 ("trans", Vec3) ]

    def __init__(self, v):
        if v is None:
            pass
        elif isinstance(v, list):
            self.quat.x = v[0]
            self.quat.y = v[1]
            self.quat.z = v[2]
            self.quat.w = v[3]
            self.trans.x = v[4]
            self.trans.y = v[5]
            self.trans.z = v[6]
        elif isinstance(v,tuple):
            self.quat = Quat(v[0])
            self.trans = Vec3(v[1])
        else:
            raise Exception('Invalid argument')

    def __str__(self):
        return 'QuatTrans(%s, %s)' % (self.quat, self.trans)

    def rotation(self):
        return self.quat

    def translation(self):
        return self.trans


###################
## LIBRARY CALLS ##
###################

libamino.aa_tf_xangle2quat.argtypes = [ctypes.c_double, ctypes.POINTER(Quat)]
libamino.aa_tf_yangle2quat.argtypes = [ctypes.c_double, ctypes.POINTER(Quat)]
libamino.aa_tf_zangle2quat.argtypes = [ctypes.c_double, ctypes.POINTER(Quat)]

libamino.aa_tf_xangle2rotmat.argtypes = [ctypes.c_double, ctypes.POINTER(RotMat)]
libamino.aa_tf_yangle2rotmat.argtypes = [ctypes.c_double, ctypes.POINTER(RotMat)]
libamino.aa_tf_zangle2rotmat.argtypes = [ctypes.c_double, ctypes.POINTER(RotMat)]

libamino.aa_tf_quat2rotmat.argtypes = [ctypes.POINTER(Quat), ctypes.POINTER(RotMat)]
libamino.aa_tf_rotmat2quat.argtypes = [ctypes.POINTER(RotMat), ctypes.POINTER(Quat)]

libamino.aa_tf_qv2duqu.argtypes = [ ctypes.POINTER(Quat),
                                    ctypes.POINTER(Vec3),
                                    ctypes.POINTER(DualQuat)]

libamino.aa_tf_duqu2qv.argtypes = [ ctypes.POINTER(DualQuat),
                                    ctypes.POINTER(Quat),
                                    ctypes.POINTER(Vec3) ]

libamino.aa_tf_duqu2qv.argtypes = [ ctypes.POINTER(DualQuat),
                                    ctypes.POINTER(Quat),
                                    ctypes.POINTER(Vec3) ]

libamino.aa_tf_duqu_trans.argtypes = [ ctypes.POINTER(DualQuat), ctypes.POINTER(Vec3) ]

libamino.aa_tf_duqu_trans.argtypes = [ ctypes.POINTER(DualQuat), ctypes.POINTER(Vec3) ]


# vector functions
libamino.aa_tf_vssd.argtypes = [ ctypes.POINTER(Vec3), ctypes.POINTER(Vec3)]
libamino.aa_tf_vssd.restype = ctypes.c_double

libamino.aa_tf_vdot.argtypes = [ ctypes.POINTER(Vec3), ctypes.POINTER(Vec3)]
libamino.aa_tf_vdot.restype = ctypes.c_double

libamino.aa_tf_vnorm.argtypes = [ ctypes.POINTER(Vec3) ]
libamino.aa_tf_vnorm.restype = ctypes.c_double

libamino.aa_tf_cross.argtypes = [ ctypes.POINTER(Vec3), ctypes.POINTER(Vec3),
                                   ctypes.POINTER(Vec3) ]

# quaternion functions
libamino.aa_tf_qmul.argtypes = [ ctypes.POINTER(Quat), ctypes.POINTER(Quat), ctypes.POINTER(Quat) ]
libamino.aa_tf_qadd.argtypes = [ ctypes.POINTER(Quat), ctypes.POINTER(Quat), ctypes.POINTER(Quat) ]
libamino.aa_tf_qsub.argtypes = [ ctypes.POINTER(Quat), ctypes.POINTER(Quat), ctypes.POINTER(Quat) ]
libamino.aa_tf_qinv.argtypes = [ ctypes.POINTER(Quat), ctypes.POINTER(Quat) ]
libamino.aa_tf_qconj.argtypes = [ ctypes.POINTER(Quat), ctypes.POINTER(Quat) ]
libamino.aa_tf_qexp.argtypes = [ ctypes.POINTER(Quat), ctypes.POINTER(Quat) ]
libamino.aa_tf_qln.argtypes = [ ctypes.POINTER(Quat), ctypes.POINTER(Quat) ]
