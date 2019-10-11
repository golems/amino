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
## @file tf.py Rotations and Transformations
##



import ctypes
from lib import libamino

from math import sqrt

from mixin import VecMixin, CopyEltsMixin

from mat import DVec, is_scalar

def ensure(thing,desired_type):
    """If thing is not of desired_type, construct a new desired_type from thing"""
    return thing if isinstance(thing,desired_type) else desired_type(thing)

class Vec3(ctypes.Structure,VecMixin):
    """Class for length-3 vectors"""

    _fields_ = [ ("x", ctypes.c_double),
                 ("y", ctypes.c_double),
                 ("z", ctypes.c_double) ]

    def __init__(self,v):
        """Construct a Vec3 object

        * If v is None, the object is unitialized.
        * If v is a list, the list elements are copied to the object.
        """
        if v is not None:
            self.copy_from(v)

    def copy_from(self,other):
        self._copy_elts(other)
        return self

    def zero(self):
        """Set to zero."""
        self.x = 0
        self.y = 0
        self.z = 0

    @staticmethod
    def ensure(thing):
        """Ensure thing is a Vec3.  If it's not, convert it."""
        return ensure(thing,Vec3)

    @staticmethod
    def identity():
        return Vec3([0,0,0])

    def ssd(self,other):
        """Sum-square-differences of self and other."""
        return libamino.aa_tf_vssd(self, Vec3.ensure(other))

    def nrm2(self):
        """2-norm (Euclidean) of self"""
        return libamino.aa_tf_vnorm(self)

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
        """Compute the cross product"""
        r = Vec3(None)
        libamino.aa_tf_cross(self,Vec3.ensure(other),r)
        return r

    def dot(self,other):
        """Compute the dot product"""
        return libamino.aa_tf_vdot(self,Vec3.ensure(other))

    def __len__(self):
        """Returns 3"""
        return 3

    def __str__(self):
        return 'Vec3(%f, %f, %f)' % (self.x, self.y, self.z)

    def __iadd__(self,other):
        if isinstance(other,Vec3):
            self.x += other.x
            self.y += other.y
            self.z += other.z
            return self
        elif is_scalar(other):
            self.x += other
            self.y += other
            self.z += other
            return self
        else:
            return self.__iadd__(Vec3(other))

    def __add__(self,other):
        """Add a scalar or vector to self"""
        return Vec3(self).__iadd__(other)

    def __radd__(self,other):
        """Add a scalar or vector to self"""
        return Vec3(self).__iadd__(other)


    def __isub__(self,other):
        if isinstance(other,Vec3):
            self.x -= other.x
            self.y -= other.y
            self.z -= other.z
        elif is_scalar(other):
            self.x -= other
            self.y -= other
            self.z -= other
        else:
            self.__isub(Vec3(other))

    def __sub__(self,other):
        """Add a scalar or vector to self"""
        return Vec3(self).__isub__(other)

    def __rsub__(self,other):
        """Add a scalar or vector to self"""
        return Vec3(self).__isub__(other)

    def to_quat(self,h):
        """Convert to a quaternion and copy to h.

        This functions sets the vector (xyz) part of h.  The scalar
        part (w) of h will be zero

        """
        h.x = self.x
        h.y = self.y
        h.z = self.z
        h.w = self.w

class XAngle(ctypes.Structure):
    """Class for rotation about the X axis"""
    _fields_ = [ ("value", ctypes.c_double) ]

    def to_quat(self,h):
        """Convert to a quaternion and store in h"""
        libamino.aa_tf_xangle2quat(self.value, h)
    def to_rotmat(self,r):
        """Convert to a rotation matrix and store in r"""
        libamino.aa_tf_xangle2rotmat(self.value, r)
    def to_axang(self,a):
        """Convert to an axis-angle and store in a"""
        libamino.aa_tf_xangle2axang(self.value, a)

    def to_eulerzyx(self,e):
        """Convert to an Euler XYZ"""
        e.x = self.value
        e.y = 0
        e.z = 0

    def __str__(self):
        return 'XAngle(%f)' % (self.value)

class YAngle(ctypes.Structure):
    """Class for rotation about the Y axis"""
    _fields_ = [ ("value", ctypes.c_double) ]

    def to_quat(self,h):
        """Convert to a quaternion and store in h"""
        libamino.aa_tf_yangle2quat(self.value, h)
    def to_rotmat(self,r):
        """Convert to a rotation matrix and store in r"""
        libamino.aa_tf_yangle2rotmat(self.value, r)
    def to_axang(self,a):
        """Convert to an axis-angle and store in a"""
        libamino.aa_tf_yangle2axang(self.value, a)

    def to_eulerzyx(self,e):
        """Convert to an Euler XYZ and store in e"""
        e.x = 0
        e.y = self.value
        e.z = 0

    def __str__(self):
        return 'YAngle(%f)' % (self.value)

class ZAngle(ctypes.Structure):
    """Class for rotation about the Z axis"""
    _fields_ = [ ("value", ctypes.c_double) ]

    def to_quat(self,h):
        """Convert to a quaternion and store in h"""
        libamino.aa_tf_zangle2quat(self.value, h)
    def to_rotmat(self,r):
        """Convert to a rotation matrix and store in r"""
        libamino.aa_tf_zangle2rotmat(self.value, r)
    def to_axang(self,a):
        """Convert to an axis-angle and store in a"""
        libamino.aa_tf_zangle2axang(self.value, a)

    def to_eulerzyx(self,e):
        """Convert to an Euler XYZ and store in e"""
        e.x = 0
        e.y = 0
        e.z = self.value

    def __str__(self):
        return 'ZAngle(%f)' % (self.value)

class EulerZYX(ctypes.Structure):
    _fields_ = [ ("z", ctypes.c_double),
                 ("y", ctypes.c_double),
                 ("x", ctypes.c_double) ]
    def __init__(self, v):
        if v is None:
            pass
        elif isinstance(v, list) or isinstance(v,tuple):
            if( 3 != len(v) ):
                raise IndexError()
            self.x = v[0]
            self.y = v[1]
            self.z = v[2]
        else:
            v.to_eulerzyx(self)

    def to_quat(self,h):
        """Convert to a quaternion"""
        libamino.aa_tf_eulerzyx2quat(self.x,self.y,self.z,h)
    def to_rotmat(self,r):
        """Convert to a rotation matrix and store in r"""
        libamino.aa_tf_eulerzyx2rotmat(self.x,self.y,self.z,r)

def EulerRPY(v):
    """Construct a Roll-Pitch-Yaw Euler angler.

    The actual object returned is an EulerZYX

    """

    if isinstance(v, list) or isinstance(v,tuple):
        if( 3 != len(v) ):
            raise IndexError()
        else:
            return EulerZYX( [v[2], v[1], v[0]] )
    else:
        return EulerZYX(v)

class AxAng(ctypes.Structure):
    """3D rotation about an arbitrary axis"""
    _fields_ = [ ("axis", Vec3),
                 ("angle", ctypes.c_double) ]
    def __init__(self, v):
        if v is not None:
            self.conv_from(v)

    def conv_from(self,v):
        if isinstance(v, list):
            if( 4 != len(v) ):
                raise IndexError()
            libamino.aa_tf_axang_make(v[0], v[1], v[2], v[3], self)
            # self.axis.x = v[0]
            # self.axis.y = v[1]
            # self.axis.z = v[2]
            # self.angle  = v[3]
        else:
            v.to_axang(self)

    def normalize(self):
        """Ensure the axis is a unit vector"""
        libamino.aa_tf_axang_normalize(self)
        return self

    def to_quat(self,h):
        """Convert to a quaternion"""
        libamino.aa_tf_axang2quat(self,h)
    def to_rotmat(self,r):
        """Convert to a rotation matrix and store in r"""
        libamino.aa_tf_axang2rotmat(self,r)
    def to_axang(self,a):
        """Convert to an axis-angle and store in a"""
        a.axis.copy_from(self.axis)
        a.angle = self.angle

    def __invert__(self):
        """Return the inverse"""
        a = AxAng(None)
        a.axis = self.axis
        a.angle = -self.angle
        return a

    def rotate(self,p):
        """Rotate a point by self"""
        q = Vec3(None)
        libamino.aa_tf_axang_rot(self,Vec3.ensure(p),q)
        return q

    def __str__(self):
        axis = self.axis
        angle = self.angle
        return 'AxAng(%f, %f, %f, %f)' % (axis.x, axis.y, axis.z, angle)

class Quat(ctypes.Structure,VecMixin):
    """Class for quaternions"""
    _fields_ = [ ("x", ctypes.c_double),
                 ("y", ctypes.c_double),
                 ("z", ctypes.c_double),
                 ("w", ctypes.c_double), ]

    def __init__(self, v):
        """Construct a Quat object

        * If v is None, the object is unitialized.
        * If v is a list, the list elements are copied to the object.
        * If v is an int or float, the scalar (w) of the object is set
          and the vector (xyz) is zero
        * If v is a Vec3, the vector (xyz) of the object is set
          and the scalar (w) is zero
        * Else v is converted to a Quat
        """
        if v is not None:
            self.conv_from(v)

    def conv_from(self,v):
        if type(v) == int or type(v) == float:
            self.x = 0
            self.y = 0
            self.z = 0
            self.w = v
        elif isinstance(v, list) or isinstance(v,tuple):
            self._copy_elts(v)
        else:
            v.to_quat(self)
        return self

    @staticmethod
    def identity():
        return Quat(1)

    @staticmethod
    def ensure(thing):
        """Ensure thing is a Quat.  If it's not, convert it."""
        return ensure(thing,Quat)


    def to_quat(self, h):
        """Convert (copy) to a quaternion"""
        h.x = self.x
        h.y = self.y
        h.z = self.z
        h.w = self.w

    def to_rotmat(self, r):
        """Convert to a rotation matrix and store in r"""
        libamino.aa_tf_quat2rotmat(self,r)

    def to_axang(self,a):
        """Convert to an axis-angle and store in a"""
        libamino.aa_tf_quat2axang(self, a)

    def vector(self):
        """Return the vector (xyz) part"""
        return Vec3(self.x,self.y,self.z)

    def scalar(self):
        """Return the scalar (w) part"""
        return self.w

    def ssd(self,other):
        """Sum-square-differences of self and other."""
        return libamino.aa_tf_qssd(self, Quat.ensure(other))

    def nrm2(self):
        """2-norm (Euclidean) of self"""
        return libamino.aa_tf_qnorm(self)

    def __add__(self,other):
        """Add quaternion and another object"""
        h = Quat(None)
        libamino.aa_tf_qadd(self,Quat.ensure(other),h)
        return h

    def __sub__(self,other):
        """Subtract quaternion and another object"""
        h = Quat(None)
        libamino.aa_tf_qsub(self,Quat.ensure(other),h)
        return h

    def __rsub__(self,other):
        h = Quat(None)
        libamino.aa_tf_qsub(Quat.ensure(other),self,h)
        return h

    def scal(self,alpha):
        """Scale this object by alpha (in-place)"""
        libamino.aa_tf_qscal(self,alpha)
        return self

    def normalize(self):
        """Normalize this object, i.e., divide by the magnitude."""
        libamino.aa_tf_qnormalize(self)
        return self

    def minimize(self):
        libamino.aa_tf_qminimize(self)
        return self

    def __rmul__(self,other):
        if type(other) == int or type(other) == float:
            return Quat(self).scal(other)
        else:
            h = Quat(None)
            libamino.aa_tf_qmul(Quat.ensure(other),self,h)
            return h

    def __mul__(self,other):
        """Multiple quaternion and another object"""
        if type(other) == int or type(other) == float:
            return Quat(self).scal(other)
        else:
            h = Quat(None)
            libamino.aa_tf_qmul(self,Quat.ensure(other),h)
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
            self.w = item
        else:
            raise IndexError(key)

    def __len__(self):
        return 4

    def conj(self):
        """Return the conjugate"""
        h = Quat(None)
        libamino.aa_tf_qconj(self,h)
        return h

    def __invert__(self):
        """Return the inverse"""
        h = Quat(None)
        libamino.aa_tf_qinv(self,h)
        return h

    def exp(self):
        """Return the exponential"""
        h = Quat(None)
        libamino.aa_tf_qexp(self,h)
        return h

    def ln(self):
        """Return the natural logarithm"""
        h = Quat(None)
        libamino.aa_tf_qln(self,h)
        return h

    def rotate(self,p):
        """Rotate a point by this quaternion"""
        q = Vec3(None)
        libamino.aa_tf_qrot(self,Vec3.ensure(p),q)
        return q

    def __str__(self):
        return 'Quat(%f, %f, %f, %f)' % (self.x, self.y, self.z, self.w)


class RotMat(ctypes.Structure):
    """Class for rotation matrices"""
    _fields_ = [ ("cx", Vec3),
                 ("cy", Vec3),
                 ("cz", Vec3) ]

    def __init__(self, v):
        """Construct a Quat object

        * If v is None, the object is unitialized.
        * If v is 1, the object is the identity rotation matrix.
        * Else v is converted to a rotation matrix.
        """
        if v is not None:
            self.conv_from(v)

    def conv_from(self,v):
        if 1 == v:
            self.cx = Vec3(1,0,0)
            self.cy = Vec3(0,1,0)
            self.cz = Vec3(0,0,1)
        else:
            v.to_rotmat(self)

    def rotate(self,p):
        """Rotate a point."""
        q = Vec3(None)
        libamino.aa_tf_rotmat_rot(self,Vec3.ensure(p),q)
        return q

    @staticmethod
    def ensure(thing):
        """Ensure thing is a RotMat.  If it's not, convert it."""
        return ensure(thing,RotMat)

    def __mul__(self,other):
        """Chain two matrices"""
        r = RotMat(None)
        libamino.aa_tf_rotmat_mul(self,RotMat.ensure(other),r)
        return r

    def to_quat(self,h):
        """Convert to a quaternion and store in h"""
        libamino.aa_tf_rotmat2quat(self,h)
    def to_rotmat(self,r):
        """Convert (copy) to a rotation matrix and store in r"""
        r.cx = self.cx
        r.cy = self.cy
        r.cz = self.cz
    def to_axang(self,a):
        """Convert to an axis-angle and store in a"""
        libamino.aa_tf_rotmat2axang(self, a)


    def ln(self):
        """Return the natural logarithm"""
        h = Vec3(None)
        libamino.aa_tf_rotmat_lnv(self,h)
        return h

    def __invert__(self):
        """Return the inverse"""
        r = RotMat(None)
        libamino.aa_tf_rotmat_inv2(self,r)
        return r

    def __getitem__(self, key):
        i,j = key
        if j == 0:
            return self.cx[i]
        elif j == 1:
            return self.cy[i]
        elif j == 2:
            return self.cz[i]
        else:
            raise IndexError(key)

    def __setitem__(self, key, item):
        i,j = key
        if j == 0:
            self.cx[i] = item
        elif j == 1:
            self.cy[i] = item
        elif j == 2:
            self.cz[i] = item
        else:
            raise IndexError(key)



    @staticmethod
    def row_matrix(args):
        m = len(args)
        A = RotMat(None)
        if m != 3:
            raise  IndexError()
        for i in range(0,m):
            n = len(args[i])
            if n != 3:
                raise  IndexError()

            A.row_vec(i).copy_from(args[i])
        return A

    def __str__(self):
        s = "RotMat.row_matrix(["
        for i in range(0,3):
            if i == 0:
                s += "["
            else:
                s += ",\n                   ["
            for j in range(0,3):
                if j == 0:
                    s += "%f" % self[i,j]
                else:
                    s += ", %f" % self[i,j]
            s += "]"
        s += "])"
        return s

class TfMat(ctypes.Structure):
    """Class for transformation matrices"""
    _fields_ = [ ("R", RotMat),
                 ("v", Vec3) ]

    def __init__(self, arg):
        if arg is not None:
            self.conv_from(arg)

    def conv_from(self,arg):
        if isinstance(arg,tuple):
            R,v = arg
            self.R.conv_from(R)
            self.v.copy_from(v)
        else:
            self.conv_from( (arg.rotation(), arg.translation()) )

    def __mul__(self,other):
        """Chain two TF matrices"""
        r = TfMat(None)
        libamino.aa_tf_tfmat_mul(self,other,r)
        return r

    def __invert__(self):
        """Return the inverse"""
        h = TfMat(None)
        libamino.aa_tf_tfmat_inv2(self,h)
        return h

    def transform(self,p):
        """Chain two TF matrices"""
        q = Vec3(None)
        libamino.aa_tf_tfmat_tf(self,Vec3.ensure(p),q)
        return q

    def rotation(self):
        """Return the rotation part"""
        return self.R

    def translation(self):
        """Return the translation part"""
        return self.v

    def __getitem__(self, key):
        i,j = key
        if j < 3:
            return self.R[key]
        elif j == 3:
            return self.v[i]
        else:
            raise IndexError(key)

    def __setitem__(self, key, item):
        i,j = key
        if j < 3:
            self.R[key] = item
        elif j == 3:
            self.v[i] = item
        else:
            raise IndexError(key)

    @staticmethod
    def ensure(thing):
        """Ensure thing is a TfMat.  If it's not, convert it."""
        return ensure(thing,TfMat)

    @staticmethod
    def row_matrix(args):
        m = len(args)
        A = TfMat(None)
        if m != 3:
            raise  IndexError()
        for i in range(0,m):
            n = len(args[i])
            if n != 4:
                raise  IndexError()
            for j in range(0,n):
                A[i,j] = args[i][j]

        return A

    def __str__(self):
        s = "TfMat.row_matrix(["
        for i in range(0,3):
            if i == 0:
                s += "["
            else:
                s += ",\n                  ["
            for j in range(0,4):
                if j == 0:
                    s += "%f" % self[i,j]
                else:
                    s += ", %f" % self[i,j]
            s += "]"
        s += "])"
        # Omit the bottom, constant row
        return s

    def __len__(self):
        return 12

class DualQuat(ctypes.Structure,CopyEltsMixin):
    """Class for Dual Number Quaternions"""
    _fields_ = [ ("real", Quat),
                 ("dual", Quat) ]


    def __init__(self, arg):
        if arg is not None:
            self.conv_from(arg)

    def conv_from(self,arg):
        if isinstance(arg,tuple):
            h,v = arg
            libamino.aa_tf_qv2duqu(Quat.ensure(h), Vec3.ensure(v),
                                   self)
        elif isinstance(arg, list):
            self._copy_elts(arg)
        else:
            self.conv_from( (arg.rotation(),arg.translation()) )

    def __mul__(self,other):
        """Chain two Dual Quaternions"""
        r = DualQuat(None)
        libamino.aa_tf_duqu_mul(self,other,r)
        return r

    def conj(self):
        """Return the conjugate"""
        h = DualQuat(None)
        libamino.aa_tf_duqu_conj(self,h)
        return h

    def __invert__(self):
        """Return the inverse"""
        # TODO: should we use the actual inverse?
        return self.conj()

    def ln(self):
        """Return the dual quaternion logarithm"""
        h = DualQuat(None)
        libamino.aa_tf_duqu_ln(self,h)
        return h

    @staticmethod
    def identity():
        return DualQuat( (Quat.identity(), Vec3.identity()) )

    @staticmethod
    def ensure(thing):
        """Ensure thing is a DualQuat.  If it's not, convert it."""
        return ensure(thing,DualQuat)

    def norm_parts(self):
        """Real and dual parts 2-norm (Euclidean)"""
        r = ctypes.c_double()
        d = ctypes.c_double()
        libamino.aa_tf_duqu_norm(self, ctypes.byref(r), ctypes.byref(d))
        return (r.value,d.value)

    def transform(self,p):
        """Transform point p"""
        q = Vec3(None)
        libamino.aa_tf_duqu_tf(self,Vec3.ensure(p),q)
        return q

    def __str__(self):
        return 'DualQuat(%s, %s)' % (self.real, self.dual)

    def __getitem__(self, key):
        if key == 0:
            return self.real.x
        elif key == 1:
            return self.real.y
        elif key == 2:
            return self.real.z
        elif key == 3:
            return self.real.w
        elif key == 4:
            return self.dual.x
        elif key == 5:
            return self.dual.y
        elif key == 6:
            return self.dual.z
        elif key == 7:
            return self.dual.w
        else:
            raise IndexError(key)

    def __setitem__(self, key, item):
        if key == 0:
            self.real.x = item
        elif key == 1:
            self.real.y = item
        elif key == 2:
            self.real.z = item
        elif key == 3:
            self.real.w = item
        elif key == 4:
            self.dual.x = item
        elif key == 5:
            self.dual.y = item
        elif key == 6:
            self.dual.z = item
        elif key == 7:
            self.dual.w = item
        else:
            raise IndexError(key)

    def rotation(self):
        """Convert to a rotation matrix and store in r"""
        return self.real

    def translation(self):
        """Return the translation"""
        v = Vec3(None)
        libamino.aa_tf_duqu_trans(self,v)
        return v

class QuatTrans(ctypes.Structure,CopyEltsMixin):
    """Class for Quaternion-Translation"""
    _fields_ = [ ("quat", Quat),
                 ("trans", Vec3) ]

    def __init__(self, arg=None):
        if arg is not None:
            self.conv_from(arg)

    def conv_from(self,arg):
        if isinstance(arg,tuple):
            (R,v) = arg
            self.quat.conv_from(R)
            self.trans.copy_from(v)
        elif isinstance(arg, list):
            self._copy_elts(arg)
        else:
            self.conv_from( (arg.rotation(),arg.translation()) )

    @staticmethod
    def identity():
        return QuatTrans( (Quat.identity(), Vec3.identity()) )

    @staticmethod
    def ensure(thing):
        """Ensure thing is a QuatTrans.  If it's not, convert it."""
        return ensure(thing,QuatTrans)

    def __mul__(self,other):
        """Chain two Dual Quaternions"""
        r = QuatTrans(None)
        libamino.aa_tf_qutr_mul(self,other,r)
        return r

    def conj(self):
        """Return the conjugate"""
        h = QuatTrans(None)
        libamino.aa_tf_qutr_conj(self,h)
        return h

    def __invert__(self):
        """Return the inverse (same as conjugate)"""
        return self.conj()

    def transform(self,p):
        """Chain two Quaternion-translations"""
        q = Vec3(None)
        libamino.aa_tf_qutr_tf(self,Vec3.ensure(p),q)
        return q

    def __str__(self):
        return 'QuatTrans(%s, %s)' % (self.quat, self.trans)

    def __getitem__(self, key):
        if key == 0:
            return self.quat.x
        elif key == 1:
            return self.quat.y
        elif key == 2:
            return self.quat.z
        elif key == 3:
            return self.quat.w
        elif key == 4:
            return self.trans.x
        elif key == 5:
            return self.trans.y
        elif key == 6:
            return self.trans.z
        else:
            raise IndexError(key)

    def __setitem__(self, key, item):
        if key == 0:
            self.quat.x = item
        elif key == 1:
            self.quat.y = item
        elif key == 2:
            self.quat.z = item
        elif key == 3:
            self.quat.w = item
        elif key == 4:
            self.trans.x = item
        elif key == 5:
            self.trans.y = item
        elif key == 6:
            self.trans.z = item
        else:
            raise IndexError(key)

    def rotation(self):
        """Return the rotation part"""
        return self.quat

    def translation(self):
        """Return the translation part"""
        return self.trans

    def __len__(self):
        return 7

#--------------#
# Differential #
#--------------#

class TfVec(ctypes.Structure):
    _fields_ = [ ("trans", Vec3),
                 ("rot", Vec3) ]

    def _first(self):
        return self.trans
    def _second(self):
        return self.rot

    def set_translation(self,x):
        self.trans.copy_from(x)

    def set_rotation(self,x):
        self.rot.copy_from(x)

    def copy_from(self,x):
        self.set_rotation(x.rot)
        self.set_translation(x.trans)

    def zero(self):
        """Set to zero."""
        self.rot.zero()
        self.trans.zero()

    def __len__(self):
        """Number of elements in self"""
        return 6

    def __getitem__(self, key):
        if key < 0:
            raise IndexError(key)
        elif key < 3:
            return self._first()[key]
        elif key < 6:
            return self._second()[key-3]
        else:
            raise IndexError(key)

    def __setitem__(self, key, item):
        if key < 0:
            raise IndexError(key)
        elif key < 3:
            self._first()[key] = item
        elif key < 6:
            self._second()[key-3] = item
        else:
            raise IndexError(key)

    def to_dvec(self, vec=DVec(6)):
        vec.copy_from(self)
        return vec

    def from_dvec(self, vec):
        vec.copy_to(self)


class TfVel(TfVec):
    """A rotational and translational velocity."""
    def __init__(self):
        pass

class Twist(TfVec):
    """A twist velocity."""
    def __init__(self):
        pass

#---------------#
# LIBRARY CALLS #
#---------------#

libamino.aa_tf_xangle2quat.argtypes = [ctypes.c_double, ctypes.POINTER(Quat)]
libamino.aa_tf_yangle2quat.argtypes = [ctypes.c_double, ctypes.POINTER(Quat)]
libamino.aa_tf_zangle2quat.argtypes = [ctypes.c_double, ctypes.POINTER(Quat)]

libamino.aa_tf_xangle2rotmat.argtypes = [ctypes.c_double, ctypes.POINTER(RotMat)]
libamino.aa_tf_yangle2rotmat.argtypes = [ctypes.c_double, ctypes.POINTER(RotMat)]
libamino.aa_tf_zangle2rotmat.argtypes = [ctypes.c_double, ctypes.POINTER(RotMat)]

libamino.aa_tf_xangle2axang.argtypes = [ctypes.c_double, ctypes.POINTER(AxAng)]
libamino.aa_tf_yangle2axang.argtypes = [ctypes.c_double, ctypes.POINTER(AxAng)]
libamino.aa_tf_zangle2axang.argtypes = [ctypes.c_double, ctypes.POINTER(AxAng)]

libamino.aa_tf_eulerzyx2rotmat.argtypes = [ ctypes.c_double, ctypes.c_double, ctypes.c_double,
                                            ctypes.POINTER(RotMat) ]

libamino.aa_tf_eulerzyx2quat.argtypes = [ ctypes.c_double, ctypes.c_double, ctypes.c_double,
                                          ctypes.POINTER(Quat) ]


libamino.aa_tf_quat2rotmat.argtypes = [ctypes.POINTER(Quat), ctypes.POINTER(RotMat)]
libamino.aa_tf_rotmat2quat.argtypes = [ctypes.POINTER(RotMat), ctypes.POINTER(Quat)]

libamino.aa_tf_axang2quat.argtypes = [ ctypes.POINTER(AxAng), ctypes.POINTER(Quat) ]
libamino.aa_tf_quat2axang.argtypes = [ ctypes.POINTER(Quat), ctypes.POINTER(AxAng) ]

libamino.aa_tf_axang2rotmat.argtypes = [ ctypes.POINTER(AxAng), ctypes.POINTER(RotMat) ]
libamino.aa_tf_rotmat2axang.argtypes = [ ctypes.POINTER(RotMat), ctypes.POINTER(AxAng) ]

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

# axang
libamino.aa_tf_axang_make.argtypes = [ ctypes.c_double, ctypes.c_double,
                                       ctypes.c_double, ctypes.c_double,
                                       ctypes.POINTER(AxAng) ]

libamino.aa_tf_axang_normalize.argtypes = [ ctypes.POINTER(AxAng) ]

libamino.aa_tf_axang_rot.argtypes = [ ctypes.POINTER(AxAng), ctypes.POINTER(Vec3), ctypes.POINTER(Vec3)]

# vector functions
libamino.aa_tf_vssd.argtypes = [ ctypes.POINTER(Vec3), ctypes.POINTER(Vec3)]
libamino.aa_tf_vssd.restype = ctypes.c_double


libamino.aa_tf_vnorm.argtypes = [ ctypes.POINTER(Vec3) ]
libamino.aa_tf_vnorm.restype = ctypes.c_double

libamino.aa_tf_vdot.argtypes = [ ctypes.POINTER(Vec3), ctypes.POINTER(Vec3)]
libamino.aa_tf_vdot.restype = ctypes.c_double

libamino.aa_tf_cross.argtypes = [ ctypes.POINTER(Vec3), ctypes.POINTER(Vec3),
                                   ctypes.POINTER(Vec3) ]

# quaternion functions

libamino.aa_tf_qssd.argtypes = [ ctypes.POINTER(Quat), ctypes.POINTER(Quat)]
libamino.aa_tf_qssd.restype = ctypes.c_double

libamino.aa_tf_qminimize.argtypes = [ ctypes.POINTER(Quat) ]

libamino.aa_tf_qnorm.argtypes = [ ctypes.POINTER(Quat) ]
libamino.aa_tf_qnorm.restype = ctypes.c_double

libamino.aa_tf_qscal.argtypes = [ ctypes.POINTER(Quat), ctypes.c_double ]

libamino.aa_tf_qnormalize.argtypes = [ ctypes.POINTER(Quat) ]

libamino.aa_tf_qmul.argtypes = [ ctypes.POINTER(Quat), ctypes.POINTER(Quat), ctypes.POINTER(Quat) ]
libamino.aa_tf_qadd.argtypes = [ ctypes.POINTER(Quat), ctypes.POINTER(Quat), ctypes.POINTER(Quat) ]
libamino.aa_tf_qsub.argtypes = [ ctypes.POINTER(Quat), ctypes.POINTER(Quat), ctypes.POINTER(Quat) ]
libamino.aa_tf_qinv.argtypes = [ ctypes.POINTER(Quat), ctypes.POINTER(Quat) ]
libamino.aa_tf_qconj.argtypes = [ ctypes.POINTER(Quat), ctypes.POINTER(Quat) ]
libamino.aa_tf_qexp.argtypes = [ ctypes.POINTER(Quat), ctypes.POINTER(Quat) ]
libamino.aa_tf_qln.argtypes = [ ctypes.POINTER(Quat), ctypes.POINTER(Quat) ]

libamino.aa_tf_qrot.argtypes = [ ctypes.POINTER(Quat), ctypes.POINTER(Vec3), ctypes.POINTER(Vec3)]


# rotation matrix

libamino.aa_tf_rotmat_rot.argtypes = [ctypes.POINTER(RotMat), ctypes.POINTER(Vec3),
                                      ctypes.POINTER(Vec3)]

libamino.aa_tf_rotmat_mul.argtypes = [ctypes.POINTER(RotMat), ctypes.POINTER(RotMat), ctypes.POINTER(RotMat)]

libamino.aa_tf_rotmat_inv2.argtypes = [ ctypes.POINTER(RotMat), ctypes.POINTER(RotMat) ]

libamino.aa_tf_rotmat_lnv.argtypes = [ ctypes.POINTER(RotMat), ctypes.POINTER(Vec3) ]

# Transforms
libamino.aa_tf_tfmat_tf.argtypes = [ ctypes.POINTER(TfMat),
                                     ctypes.POINTER(Vec3),
                                     ctypes.POINTER(Vec3) ]

libamino.aa_tf_qutr_tf.argtypes = [ ctypes.POINTER(QuatTrans),
                                     ctypes.POINTER(Vec3),
                                     ctypes.POINTER(Vec3) ]

libamino.aa_tf_duqu_tf.argtypes = [ ctypes.POINTER(DualQuat),
                                    ctypes.POINTER(Vec3),
                                    ctypes.POINTER(Vec3) ]

libamino.aa_tf_duqu_ln.argtypes = [ ctypes.POINTER(DualQuat), ctypes.POINTER(DualQuat) ]

libamino.aa_tf_duqu_norm.argtypes = [ ctypes.POINTER(DualQuat),
                                      ctypes.POINTER(ctypes.c_double) ,
                                      ctypes.POINTER(ctypes.c_double) ]


libamino.aa_tf_tfmat_mul.argtypes = [ ctypes.POINTER(TfMat), ctypes.POINTER(TfMat), ctypes.POINTER(TfMat) ]

libamino.aa_tf_duqu_mul.argtypes = [ ctypes.POINTER(DualQuat), ctypes.POINTER(DualQuat), ctypes.POINTER(DualQuat) ]

libamino.aa_tf_qutr_mul.argtypes = [ ctypes.POINTER(QuatTrans), ctypes.POINTER(QuatTrans), ctypes.POINTER(QuatTrans) ]

libamino.aa_tf_tfmat_inv2.argtypes = [ ctypes.POINTER(TfMat), ctypes.POINTER(TfMat)]

libamino.aa_tf_duqu_conj.argtypes = [ ctypes.POINTER(DualQuat), ctypes.POINTER(DualQuat)]

libamino.aa_tf_qutr_conj.argtypes = [ ctypes.POINTER(QuatTrans), ctypes.POINTER(QuatTrans)]

libamino.aa_tf_qutr_conj.argtypes = [ ctypes.POINTER(QuatTrans), ctypes.POINTER(QuatTrans)]
