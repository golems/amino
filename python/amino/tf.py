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
"""Euclidean transformation representations."""

import ctypes
from amino.lib import libamino
from amino.mixin import VecMixin, CopyEltsMixin, MatMixin
from amino.mat import DVec, is_scalar
from amino.util import ensure


class Vec3(ctypes.Structure, VecMixin):
    """Length-3 vectors"""

    _fields_ = [("x", ctypes.c_double), ("y", ctypes.c_double),
                ("z", ctypes.c_double)]

    def __init__(self, arg=None):
        """Constructs a Vec3 object

        * If arg is None, the object is unitialized.
        * If arg is a list, the list elements are copied to the object.
        """
        if arg is None:
            super(Vec3, self).__init__(0, 0, 0)
        elif len(arg) == 3:
            super(Vec3, self).__init__(arg[0], arg[1], arg[2])
        else:
            raise IndexError()

    def zero(self):
        """Sets to zero."""
        self.x = 0
        self.y = 0
        self.z = 0

    @staticmethod
    def ensure(thing):
        """Ensures thing is a Vec3.  If it's not, convert it."""
        return ensure(thing, Vec3)

    @staticmethod
    def identity():
        """Returns the identity Vec3."""
        return Vec3([0, 0, 0])

    def ssd(self, other):
        """Sum-square-differences of self and other."""
        return libamino.aa_tf_vssd(self, Vec3.ensure(other))

    def nrm2(self):
        """2-norm (Euclidean) of self"""
        return libamino.aa_tf_vnorm(self)

    def __getitem__(self, key):
        if key == 0:
            return self.x
        if key == 1:
            return self.y
        if key == 2:
            return self.z
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

    def cross(self, other):
        """Computes the cross product"""
        r = Vec3()
        libamino.aa_tf_cross(self, Vec3.ensure(other), r)
        return r

    def dot(self, other):
        """Computes the dot product"""
        return libamino.aa_tf_vdot(self, Vec3.ensure(other))

    def __len__(self):
        return 3

    def __str__(self):
        return 'Vec3(%f, %f, %f)' % (self.x, self.y, self.z)

    def __iadd__(self, other):
        if isinstance(other, Vec3):
            self.x += other.x
            self.y += other.y
            self.z += other.z
            return self

        if is_scalar(other):
            self.x += other
            self.y += other
            self.z += other
            return self

        # try to convert and then add
        return self.__iadd__(Vec3(other))

    def __add__(self, other):
        return Vec3(self).__iadd__(other)

    def __radd__(self, other):
        return Vec3(self).__iadd__(other)

    def __isub__(self, other):
        if isinstance(other, Vec3):
            self.x -= other.x
            self.y -= other.y
            self.z -= other.z
            return self

        if is_scalar(other):
            self.x -= other
            self.y -= other
            self.z -= other
            return self

        return self.__isub__(Vec3(other))

    def __sub__(self, other):
        """Subtracts a scalar or vector from self"""

        return Vec3(self).__isub__(other)

    def __rsub__(self, other):
        """Subtracts self or vector from self"""
        if is_scalar(other):
            v = Vec3()
            v.x = other - self.x
            v.y = other - self.y
            v.z = other - self.z
            return v

        return Vec3(other).__isub__(self)

    def __imul__(self, other):
        if is_scalar(other):
            self.x *= other
            self.y *= other
            self.z *= other
            return self
        raise TypeError()

    def __mul__(self, other):
        v = Vec3(self)
        v *= other
        return v

    def __rmul__(self, other):
        return self * other

    def __itruediv__(self, other):
        if is_scalar(other):
            self.x /= other
            self.y /= other
            self.z /= other
            return self
        raise TypeError()

    def __truediv__(self, other):
        return Vec3(self).__itruediv__(other)

    def to_quat(self, h):
        """Converts to a quaternion and copies to h.

        This functions sets the vector (xyz) part of h.  The scalar
        part (w) of h will be zero

        """
        h.x = self.x
        h.y = self.y
        h.z = self.z
        h.w = 0


class XAngle(ctypes.Structure):
    """Rotation about the X axis"""
    _fields_ = [("value", ctypes.c_double)]

    def to_quat(self, h):
        """Converts to a quaternion and store in h"""
        libamino.aa_tf_xangle2quat(self.value, h)

    def to_rotmat(self, r):
        """Converts to a rotation matrix and store in r"""
        libamino.aa_tf_xangle2rotmat(self.value, r)

    def to_axang(self, a):
        """Converts to an axis-angle and store in a"""
        libamino.aa_tf_xangle2axang(self.value, a)

    def to_eulerzyx(self, e):
        """Converts to an Euler XYZ"""
        e.x = self.value
        e.y = 0
        e.z = 0

    def __str__(self):
        return 'XAngle(%f)' % (self.value)


class YAngle(ctypes.Structure):
    """Rotation about the Y axis"""
    _fields_ = [("value", ctypes.c_double)]

    def to_quat(self, h):
        """Converts to a quaternion and store in h"""
        libamino.aa_tf_yangle2quat(self.value, h)

    def to_rotmat(self, r):
        """Converts to a rotation matrix and store in r"""
        libamino.aa_tf_yangle2rotmat(self.value, r)

    def to_axang(self, a):
        """Converts to an axis-angle and store in a"""
        libamino.aa_tf_yangle2axang(self.value, a)

    def to_eulerzyx(self, e):
        """Converts to an Euler XYZ and store in e"""
        e.x = 0
        e.y = self.value
        e.z = 0

    def __str__(self):
        return 'YAngle(%f)' % (self.value)


class ZAngle(ctypes.Structure):
    """Rotation about the Z axis"""
    _fields_ = [("value", ctypes.c_double)]

    def to_quat(self, h):
        """Converts to a quaternion and store in h"""
        libamino.aa_tf_zangle2quat(self.value, h)

    def to_rotmat(self, r):
        """Converts to a rotation matrix and store in r"""
        libamino.aa_tf_zangle2rotmat(self.value, r)

    def to_axang(self, a):
        """Converts to an axis-angle and store in a"""
        libamino.aa_tf_zangle2axang(self.value, a)

    def to_eulerzyx(self, e):
        """Converts to an Euler XYZ and store in e"""
        e.x = 0
        e.y = 0
        e.z = self.value

    def __str__(self):
        return 'ZAngle(%f)' % (self.value)


class EulerZYX(ctypes.Structure):
    """Z-Y-X Euler angles"""
    _fields_ = [("_z", ctypes.c_double), ("_y", ctypes.c_double),
                ("_x", ctypes.c_double)]

    def __init__(self, arg=None):
        if arg is None:
            super(EulerZYX, self).__init__(0, 0, 0)
        elif isinstance(arg, (list, tuple)):
            if len(arg) != 3:
                raise IndexError()
            super(EulerZYX, self).__init__(arg[0], arg[1], arg[2])
        else:
            arg.to_eulerzyx(self)

    def to_quat(self, h):
        """Converts to a quaternion"""
        libamino.aa_tf_eulerzyx2quat(self._z, self._y, self._x, h)

    def to_rotmat(self, r):
        """Converts to a rotation matrix and store in r"""
        libamino.aa_tf_eulerzyx2rotmat(self._z, self._y, self._x, r)

    @property
    def x(self):
        """X-axis rotation."""
        return self._x

    @x.setter
    def x(self, value):
        self._x = value

    @property
    def y(self):
        """Y-axis rotation."""
        return self._y

    @y.setter
    def y(self, value):
        self._y = value

    @property
    def z(self):
        """Z-axis rotation."""
        return self._z

    @z.setter
    def z(self, value):
        self._z = value


class EulerRPY(EulerZYX):
    """Roll-Pitch-Yaw Euler angles"""

    def __init__(self, arg=None):
        if isinstance(arg, (list, tuple)):
            if len(arg) != 3:
                raise IndexError()
            arg = (arg[2], arg[1], arg[0])
        super(EulerRPY, self).__init__(arg)

    @property
    def r(self):
        """Roll."""
        return self._x

    @r.setter
    def r(self, value):
        self._x = value

    @property
    def p(self):
        """Pitch."""
        return self._y

    @p.setter
    def p(self, value):
        self._y = value

    @property
    def y(self):
        """Yaw."""
        return self._z

    @y.setter
    def y(self, value):
        self._z = value

    def __str__(self):
        return 'EulerRPY((%f, %f, %f))' % (self.r, self.p, self.y)


class AxAng(ctypes.Structure):
    """3D rotation about an arbitrary axis"""
    _fields_ = [("axis", Vec3), ("angle", ctypes.c_double)]

    def __init__(self, arg=None):
        if arg is None:
            self.set_identity()
        else:
            self.conv_from(arg)

    def set_identity(self):
        """Set to identity"""
        self.axis.copy_from((0, 0, 1))
        self.angle = 0

    def conv_from(self, src):
        """Converts src into an AxAng."""
        if isinstance(src, list):
            if len(src) != 4:
                raise IndexError()
            libamino.aa_tf_axang_make(src[0], src[1], src[2], src[3], self)
        else:
            src.to_axang(self)

    def normalize(self):
        """Ensures the axis is a unit vector"""
        libamino.aa_tf_axang_normalize(self)
        return self

    def to_quat(self, h):
        """Converts to a quaternion"""
        libamino.aa_tf_axang2quat(self, h)

    def to_rotmat(self, r):
        """Converts to a rotation matrix and store in r"""
        libamino.aa_tf_axang2rotmat(self, r)

    def to_axang(self, a):
        """Converts to an axis-angle and store in a"""
        a.axis.copy_from(self.axis)
        a.angle = self.angle

    def __invert__(self):
        """Returns the inverse"""
        a = AxAng()
        a.axis = self.axis
        a.angle = -self.angle
        return a

    def rotate(self, p):
        """Rotate a point by self"""
        q = Vec3()
        libamino.aa_tf_axang_rot(self, Vec3.ensure(p), q)
        return q

    def __str__(self):
        axis = self.axis
        angle = self.angle
        return 'AxAng(%f, %f, %f, %f)' % (axis.x, axis.y, axis.z, angle)


class Quat(ctypes.Structure, VecMixin):
    """Class for quaternions"""
    _fields_ = [
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
        ("z", ctypes.c_double),
        ("w", ctypes.c_double),
    ]

    def __init__(self, arg=None):
        """Construct a Quat object

        * If arg is None, the object is unitialized.
        * If arg is a list, the list elements are copied to the object.
        * If arg is an int or float, the scalar (w) of the object is set
          and the vector (xyz) is zero
        * If arg is a Vec3, the vector (xyz) of the object is set
          and the scalar (w) is zero
        * Else arg is converted to a Quat
        """
        if arg is None:
            self.set_identity()
        else:
            self.conv_from(arg)

    def conv_from(self, src):
        """Converts src to a quaternion."""
        if is_scalar(src):
            self.x = 0
            self.y = 0
            self.z = 0
            self.w = src
        elif isinstance(src, (list, tuple)):
            self.copy_from(src)
        else:
            src.to_quat(self)
        return self

    @staticmethod
    def identity():
        """Returns the identity quaternion."""
        return Quat(1)

    @staticmethod
    def ensure(thing):
        """Ensures thing is a Quat.  If it's not, convert it."""
        return ensure(thing, Quat)

    def to_quat(self, h):
        """Converts (copy) to a quaternion"""
        h.x = self.x
        h.y = self.y
        h.z = self.z
        h.w = self.w

    def to_rotmat(self, r):
        """Converts to a rotation matrix and store in r"""
        libamino.aa_tf_quat2rotmat(self, r)

    def to_axang(self, a):
        """Converts to an axis-angle and store in a"""
        libamino.aa_tf_quat2axang(self, a)

    def to_eulerzyx(self):
        """Converts to an euler zyx angle representation"""
        ang = (ctypes.c_double * 3)()
        libamino.aa_tf_quat2eulerzyx(self, ang)
        x = [ang[0], ang[1], ang[2]]
        return EulerZYX(x)

    def vector(self):
        """Returns the vector (xyz) part"""
        return Vec3([self.x, self.y, self.z])

    def scalar(self):
        """Returns the scalar (w) part"""
        return self.w

    def ssd(self, other):
        """Sum-square-differences of self and other."""
        return libamino.aa_tf_qssd(self, Quat.ensure(other))

    def nrm2(self):
        """2-norm (Euclidean) of self."""
        return libamino.aa_tf_qnorm(self)

    # Do not define addition/subtraction for scalars.  The meaning is
    # non-obvious since we could either add/sub every quaternion
    # element or only the w (quaternion scalar) element.
    def __iadd__(self, other):
        libamino.aa_tf_qiadd(self, Quat.ensure(other))
        return self

    def __isub__(self, other):
        libamino.aa_tf_qisub(self, Quat.ensure(other))
        return self

    def __add__(self, other):
        h = Quat()
        libamino.aa_tf_qadd(self, Quat.ensure(other), h)
        return h

    def __radd__(self, other):
        return self + other

    def __sub__(self, other):
        h = Quat()
        libamino.aa_tf_qsub(self, Quat.ensure(other), h)
        return h

    def __rsub__(self, other):
        h = Quat()
        libamino.aa_tf_qsub(Quat.ensure(other), self, h)
        return h

    def normalize(self):
        """Normalize this object, i.e., divide by the magnitude."""
        libamino.aa_tf_qnormalize(self)
        return self

    def minimize(self):
        """Converts to the minimal quaternion for the represented rotation."""
        libamino.aa_tf_qminimize(self)
        return self

    def set_identity(self):
        """Set to identity"""
        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 1

    def zero(self):
        """Set to zero"""
        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 0

    def __imul__(self, other):
        if is_scalar(other):
            libamino.aa_tf_qscal(self, other)
        else:
            raise TypeError()
        return self

    def __rmul__(self, other):
        if is_scalar(other):
            return Quat(self).__imul__(other)

        h = Quat()
        libamino.aa_tf_qmul(Quat.ensure(other), self, h)
        return h

    def __mul__(self, other):
        if is_scalar(other):
            return Quat(self).__imul__(other)

        h = Quat()
        libamino.aa_tf_qmul(self, Quat.ensure(other), h)
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
        """Returns the conjugate"""
        h = Quat()
        libamino.aa_tf_qconj(self, h)
        return h

    def __invert__(self):
        """Returns the inverse"""
        h = Quat()
        libamino.aa_tf_qinv(self, h)
        return h

    def exp(self):
        """Returns the exponential"""
        h = Quat()
        libamino.aa_tf_qexp(self, h)
        return h

    def ln(self):
        """Returns the natural logarithm"""
        h = Quat()
        libamino.aa_tf_qln(self, h)
        return h

    def rotate(self, p):
        """Rotate a point by this quaternion"""
        q = Vec3()
        libamino.aa_tf_qrot(self, Vec3.ensure(p), q)
        return q

    def __str__(self):
        return 'Quat(%f, %f, %f, %f)' % (self.x, self.y, self.z, self.w)


class RotMat(ctypes.Structure, MatMixin):
    """Class for rotation matrices"""
    _fields_ = [("cx", Vec3), ("cy", Vec3), ("cz", Vec3)]

    def __init__(self, arg=None):
        """Construct a RotMat object

        * If arg is None, the object is unitialized.
        * If arg is 1, the object is the identity rotation matrix.
        * Else arg is converted to a rotation matrix.
        """
        if arg is None:
            self.set_identity()
        else:
            self.conv_from(arg)

    def conv_from(self, src):
        """Converts src to a rotation matrix."""
        if src == 1:
            self.set_identity()
        else:
            src.to_rotmat(self)

    def set_identity(self):
        """Set to identity"""
        self.cx.copy_from((1, 0, 0))
        self.cy.copy_from((0, 1, 0))
        self.cz.copy_from((0, 0, 1))

    def rotate(self, p):
        """Rotate a point."""
        q = Vec3(None)
        libamino.aa_tf_rotmat_rot(self, Vec3.ensure(p), q)
        return q

    @staticmethod
    def ensure(thing):
        """Ensures thing is a RotMat.  If it's not, convert it."""
        return ensure(thing, RotMat)

    def __mul__(self, other):
        """Chains two matrices"""
        r = RotMat()
        libamino.aa_tf_rotmat_mul(self, RotMat.ensure(other), r)
        return r

    def to_quat(self, h):
        """Converts to a quaternion and store in h"""
        libamino.aa_tf_rotmat2quat(self, h)

    def to_rotmat(self, r):
        """Converts (copy) to a rotation matrix and store in r"""
        r.cx = self.cx
        r.cy = self.cy
        r.cz = self.cz

    def to_axang(self, a):
        """Converts to an axis-angle and store in a"""
        libamino.aa_tf_rotmat2axang(self, a)

    def ln(self):
        """Returns the natural logarithm"""
        h = Vec3()
        libamino.aa_tf_rotmat_lnv(self, h)
        return h

    def __invert__(self):
        """Returns the inverse"""
        r = RotMat()
        libamino.aa_tf_rotmat_inv2(self, r)
        return r

    def __getitem__(self, key):
        i, j = key
        if j == 0:
            return self.cx[i]
        if j == 1:
            return self.cy[i]
        if j == 2:
            return self.cz[i]
        raise IndexError(key)

    def __setitem__(self, key, item):
        i, j = key
        if j == 0:
            self.cx[i] = item
        elif j == 1:
            self.cy[i] = item
        elif j == 2:
            self.cz[i] = item
        else:
            raise IndexError(key)

    @property
    def rows(self):
        """Number of rows."""
        return 3;

    @property
    def cols(self):
        """Number of columns."""
        return 3;

    @staticmethod
    def row_matrix(args):
        """Constructs rotation matrix from rows in args."""
        if len(args) != 3:
            raise IndexError()
        A = RotMat()
        for i in range(0, 3):
            if len(args[i]) != 3:
                raise IndexError()
            for j in range(0, 3):
                A[i, j] = args[i][j]
        return A

    @staticmethod
    def col_matrix(args):
        """Constructs rotation matrix from columns in args."""
        if len(args) != 3:
            raise IndexError()
        A = RotMat()
        for j in range(0, 3):
            if len(args[j]) != 3:
                raise IndexError()
            for i in range(0, 3):
                A[i, j] = args[j][i]
        return A

    def __str__(self):
        return self._str_helper("RotMat.row_matrix")

    def isclose(self, other, tol=1e-9):
        """Returns True if self is within tol rotation angle to other."""
        other = RotMat.ensure(other)
        v = (self * ~other).ln().nrm2()
        return v <= tol


class TfMat(ctypes.Structure, MatMixin):
    """Class for transformation matrices"""
    _fields_ = [("R", RotMat), ("v", Vec3)]

    def __init__(self, arg=None):
        if arg is None:
            self.set_identity()
        else:
            self.conv_from(arg)

    def conv_from(self, src):
        """Converts src to a transformation matrix."""
        if isinstance(src, tuple):
            R, v = src
            self.R.conv_from(R)
            self.v.copy_from(v)
        else:
            self.conv_from((src.rotation, src.translation))

    def __mul__(self, other):
        """Chains two TF matrices"""
        r = TfMat()
        libamino.aa_tf_tfmat_mul(self, other, r)
        return r

    def set_identity(self):
        """Set to identity"""
        self.R.set_identity()
        self.v.copy_from((0, 0, 0))

    def __invert__(self):
        """Returns the inverse"""
        h = TfMat()
        libamino.aa_tf_tfmat_inv2(self, h)
        return h

    def transform(self, p):
        """Chains two TF matrices"""
        q = Vec3()
        libamino.aa_tf_tfmat_tf(self, Vec3.ensure(p), q)
        return q

    @property
    def rotation(self):
        """Rotation part"""
        return self.R

    @rotation.setter
    def rotation(self, value):
        self.R.conv_from(value)

    @property
    def translation(self):
        """Translation part"""
        return self.v

    @translation.setter
    def translation(self, value):
        return self.v.copy_from(value)

    def __getitem__(self, key):
        i, j = key
        if j < 3:
            return self.R[key]
        elif j == 3:
            return self.v[i]
        else:
            raise IndexError(key)

    def __setitem__(self, key, item):
        i, j = key
        if j < 3:
            self.R[key] = item
        elif j == 3:
            self.v[i] = item
        else:
            raise IndexError(key)

    @staticmethod
    def ensure(thing):
        """Ensures thing is a TfMat.  If it's not, convert it."""
        return ensure(thing, TfMat)

    @staticmethod
    def row_matrix(args):
        """Constructs transformation matrix from rows in args."""
        m = len(args)
        A = TfMat()
        if m != 3:
            raise IndexError()
        for i in range(0, m):
            n = len(args[i])
            if n != 4:
                raise IndexError()
            for j in range(0, n):
                A[i, j] = args[i][j]

        return A

    def __str__(self):
        return self._str_helper("TfMat.row_matrix", 3, 3)

    def __len__(self):
        return 12


class DualQuat(ctypes.Structure, CopyEltsMixin):
    """Class for Dual Number Quaternions"""
    _fields_ = [("real", Quat), ("dual", Quat)]

    def __init__(self, arg=None):
        if arg is None:
            self.set_identity()
        else:
            self.conv_from(arg)

    def set_identity(self):
        """Set to identity"""
        self.real.set_identity()
        self.dual.zero()

    def conv_from(self, src):
        """Converts src to a Dual Quaternion."""
        if isinstance(src, tuple):
            h, v = src
            libamino.aa_tf_qv2duqu(Quat.ensure(h), Vec3.ensure(v), self)
        elif isinstance(src, list):
            self.copy_from(src)
        else:
            self.conv_from((src.rotation, src.translation))

    def __mul__(self, other):
        """Chains two Dual Quaternions"""
        r = DualQuat()
        libamino.aa_tf_duqu_mul(self, other, r)
        return r

    def conj(self):
        """Returns the conjugate"""
        h = DualQuat()
        libamino.aa_tf_duqu_conj(self, h)
        return h

    def __invert__(self):
        """Returns the inverse"""
        # TODO: should we use the actual inverse?
        return self.conj()

    def ln(self):
        """Returns the dual quaternion logarithm"""
        h = DualQuat()
        libamino.aa_tf_duqu_ln(self, h)
        return h

    @staticmethod
    def identity():
        """Returns the identity dual quaternion."""
        return DualQuat((Quat.identity(), Vec3.identity()))

    @staticmethod
    def ensure(thing):
        """Ensures thing is a DualQuat.  If it's not, convert it."""
        return ensure(thing, DualQuat)

    def norm_parts(self):
        """Real and dual parts 2-norm (Euclidean)"""
        r = ctypes.c_double()
        d = ctypes.c_double()
        libamino.aa_tf_duqu_norm(self, ctypes.byref(r), ctypes.byref(d))
        return (r.value, d.value)

    def transform(self, p):
        """Transform point p"""
        q = Vec3()
        libamino.aa_tf_duqu_tf(self, Vec3.ensure(p), q)
        return q

    def __str__(self):
        return 'DualQuat(%s, %s)' % (self.real, self.dual)

    def __getitem__(self, key):
        if key == 0:
            v = self.real.x
        elif key == 1:
            v = self.real.y
        elif key == 2:
            v = self.real.z
        elif key == 3:
            v = self.real.w
        elif key == 4:
            v = self.dual.x
        elif key == 5:
            v = self.dual.y
        elif key == 6:
            v = self.dual.z
        elif key == 7:
            v = self.dual.w
        else:
            raise IndexError(key)
        return v

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

    @property
    def rotation(self):
        """Rotation part"""
        return Quat(self.real)

    @property
    def translation(self):
        """Translation"""
        v = Vec3()
        libamino.aa_tf_duqu_trans(self, v)
        return v

    @rotation.setter
    def rotation(self, value):
        h = Quat(value)
        v = self.translation
        libamino.aa_tf_qv2duqu(h, v, self)

    @translation.setter
    def translation(self, value):
        h = self.rotation
        v = Vec3(value)
        libamino.aa_tf_qv2duqu(h, v, self)


class QuatTrans(ctypes.Structure, CopyEltsMixin):
    """Class for Quaternion-Translation"""
    _fields_ = [("quat", Quat), ("trans", Vec3)]

    def __init__(self, arg=None):
        if arg is None:
            self.set_identity()
        else:
            self.conv_from(arg)

    def set_identity(self):
        """Set to identity"""
        self.quat.set_identity()
        self.trans.copy_from((0, 0, 0))

    def conv_from(self, src):
        """Converts src to quaternion-translation."""
        if isinstance(src, tuple):
            (R, v) = src
            self.quat.conv_from(R)
            self.trans.copy_from(v)
        elif isinstance(src, list):
            self.copy_from(src)
        else:
            self.conv_from((src.rotation, src.translation))

    @staticmethod
    def identity():
        """Returns the identity QuatTrans."""
        return QuatTrans((Quat.identity(), Vec3.identity()))

    @staticmethod
    def ensure(thing):
        """Ensures thing is a QuatTrans.  If it's not, convert it."""
        return ensure(thing, QuatTrans)

    def __mul__(self, other):
        """Chains two Dual Quaternions"""
        r = QuatTrans()
        libamino.aa_tf_qutr_mul(self, other, r)
        return r

    def conj(self):
        """Returns the conjugate"""
        h = QuatTrans()
        libamino.aa_tf_qutr_conj(self, h)
        return h

    def __invert__(self):
        """Returns the inverse (same as conjugate)"""
        return self.conj()

    def transform(self, p):
        """Chains two Quaternion-translations"""
        q = Vec3()
        libamino.aa_tf_qutr_tf(self, Vec3.ensure(p), q)
        return q

    def __str__(self):
        return 'QuatTrans(%s, %s)' % (self.quat, self.trans)

    def __getitem__(self, key):
        if key == 0:
            v = self.quat.x
        elif key == 1:
            v = self.quat.y
        elif key == 2:
            v = self.quat.z
        elif key == 3:
            v = self.quat.w
        elif key == 4:
            v = self.trans.x
        elif key == 5:
            v = self.trans.y
        elif key == 6:
            v = self.trans.z
        else:
            raise IndexError(key)
        return v

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

    @property
    def rotation(self):
        """Rotation part"""
        return self.quat

    @rotation.setter
    def rotation(self, value):
        return self.quat.conv_from(value)

    @property
    def translation(self):
        """Translation"""
        return self.trans

    @translation.setter
    def translation(self, value):
        return self.trans.copy_from(value)

    def __len__(self):
        return 7


#--------------#
# Differential #
#--------------#
class TfVec(ctypes.Structure):
    """six-element vector with rotational and translational parts."""
    _fields_ = [("trans", Vec3), ("rot", Vec3)]

    def _first(self):
        return self.trans

    def _second(self):
        return self.rot

    @property
    def rotational(self):
        """Rotational part"""
        return self.rot

    @property
    def translational(self):
        """Translational part"""
        return self.trans

    @translational.setter
    def translational(self, x):
        self.trans.copy_from(x)

    @rotational.setter
    def rotational(self, x):
        self.rot.copy_from(x)

    def copy_from(self, src):
        """Copies src into self."""
        self.rotational = src.rotational
        self.translational = src.translational

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
            return self._second()[key - 3]
        else:
            raise IndexError(key)

    def __setitem__(self, key, item):
        if key < 0:
            raise IndexError(key)
        elif key < 3:
            self._first()[key] = item
        elif key < 6:
            self._second()[key - 3] = item
        else:
            raise IndexError(key)

    def to_dvec(self, vec=DVec(6)):
        """Copies self to a vec."""
        vec.copy_from(self)
        return vec

    def from_dvec(self, vec):
        """Copies vec to self."""
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

libamino.aa_tf_xangle2rotmat.argtypes = [
    ctypes.c_double, ctypes.POINTER(RotMat)
]
libamino.aa_tf_yangle2rotmat.argtypes = [
    ctypes.c_double, ctypes.POINTER(RotMat)
]
libamino.aa_tf_zangle2rotmat.argtypes = [
    ctypes.c_double, ctypes.POINTER(RotMat)
]

libamino.aa_tf_xangle2axang.argtypes = [ctypes.c_double, ctypes.POINTER(AxAng)]
libamino.aa_tf_yangle2axang.argtypes = [ctypes.c_double, ctypes.POINTER(AxAng)]
libamino.aa_tf_zangle2axang.argtypes = [ctypes.c_double, ctypes.POINTER(AxAng)]

libamino.aa_tf_eulerzyx2rotmat.argtypes = [
    ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ctypes.POINTER(RotMat)
]

libamino.aa_tf_eulerzyx2quat.argtypes = [
    ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ctypes.POINTER(Quat)
]

libamino.aa_tf_quat2eulerzyx.argtypes = [ctypes.POINTER(Quat),
                                         ctypes.POINTER(ctypes.c_double)]

libamino.aa_tf_quat2rotmat.argtypes = [
    ctypes.POINTER(Quat), ctypes.POINTER(RotMat)
]
libamino.aa_tf_rotmat2quat.argtypes = [
    ctypes.POINTER(RotMat), ctypes.POINTER(Quat)
]

libamino.aa_tf_axang2quat.argtypes = [
    ctypes.POINTER(AxAng), ctypes.POINTER(Quat)
]
libamino.aa_tf_quat2axang.argtypes = [
    ctypes.POINTER(Quat), ctypes.POINTER(AxAng)
]

libamino.aa_tf_axang2rotmat.argtypes = [
    ctypes.POINTER(AxAng), ctypes.POINTER(RotMat)
]
libamino.aa_tf_rotmat2axang.argtypes = [
    ctypes.POINTER(RotMat), ctypes.POINTER(AxAng)
]

libamino.aa_tf_qv2duqu.argtypes = [
    ctypes.POINTER(Quat),
    ctypes.POINTER(Vec3),
    ctypes.POINTER(DualQuat)
]

libamino.aa_tf_duqu2qv.argtypes = [
    ctypes.POINTER(DualQuat),
    ctypes.POINTER(Quat),
    ctypes.POINTER(Vec3)
]

libamino.aa_tf_duqu2qv.argtypes = [
    ctypes.POINTER(DualQuat),
    ctypes.POINTER(Quat),
    ctypes.POINTER(Vec3)
]

libamino.aa_tf_duqu_trans.argtypes = [
    ctypes.POINTER(DualQuat), ctypes.POINTER(Vec3)
]

libamino.aa_tf_duqu_trans.argtypes = [
    ctypes.POINTER(DualQuat), ctypes.POINTER(Vec3)
]

# axang
libamino.aa_tf_axang_make.argtypes = [
    ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ctypes.POINTER(AxAng)
]

libamino.aa_tf_axang_normalize.argtypes = [ctypes.POINTER(AxAng)]

libamino.aa_tf_axang_rot.argtypes = [
    ctypes.POINTER(AxAng),
    ctypes.POINTER(Vec3),
    ctypes.POINTER(Vec3)
]

# vector functions
libamino.aa_tf_vssd.argtypes = [ctypes.POINTER(Vec3), ctypes.POINTER(Vec3)]
libamino.aa_tf_vssd.restype = ctypes.c_double

libamino.aa_tf_vnorm.argtypes = [ctypes.POINTER(Vec3)]
libamino.aa_tf_vnorm.restype = ctypes.c_double

libamino.aa_tf_vdot.argtypes = [ctypes.POINTER(Vec3), ctypes.POINTER(Vec3)]
libamino.aa_tf_vdot.restype = ctypes.c_double

libamino.aa_tf_cross.argtypes = [
    ctypes.POINTER(Vec3),
    ctypes.POINTER(Vec3),
    ctypes.POINTER(Vec3)
]

# quaternion functions

libamino.aa_tf_qssd.argtypes = [ctypes.POINTER(Quat), ctypes.POINTER(Quat)]
libamino.aa_tf_qssd.restype = ctypes.c_double

libamino.aa_tf_qminimize.argtypes = [ctypes.POINTER(Quat)]

libamino.aa_tf_qnorm.argtypes = [ctypes.POINTER(Quat)]
libamino.aa_tf_qnorm.restype = ctypes.c_double

libamino.aa_tf_qscal.argtypes = [ctypes.POINTER(Quat), ctypes.c_double]

libamino.aa_tf_qnormalize.argtypes = [ctypes.POINTER(Quat)]

libamino.aa_tf_qmul.argtypes = [
    ctypes.POINTER(Quat),
    ctypes.POINTER(Quat),
    ctypes.POINTER(Quat)
]
libamino.aa_tf_qadd.argtypes = [
    ctypes.POINTER(Quat),
    ctypes.POINTER(Quat),
    ctypes.POINTER(Quat)
]
libamino.aa_tf_qsub.argtypes = [
    ctypes.POINTER(Quat),
    ctypes.POINTER(Quat),
    ctypes.POINTER(Quat)
]
libamino.aa_tf_qiadd.argtypes = [ctypes.POINTER(Quat), ctypes.POINTER(Quat)]
libamino.aa_tf_qisub.argtypes = [ctypes.POINTER(Quat), ctypes.POINTER(Quat)]
libamino.aa_tf_qinv.argtypes = [ctypes.POINTER(Quat), ctypes.POINTER(Quat)]
libamino.aa_tf_qconj.argtypes = [ctypes.POINTER(Quat), ctypes.POINTER(Quat)]
libamino.aa_tf_qexp.argtypes = [ctypes.POINTER(Quat), ctypes.POINTER(Quat)]
libamino.aa_tf_qln.argtypes = [ctypes.POINTER(Quat), ctypes.POINTER(Quat)]

libamino.aa_tf_qrot.argtypes = [
    ctypes.POINTER(Quat),
    ctypes.POINTER(Vec3),
    ctypes.POINTER(Vec3)
]

# rotation matrix

libamino.aa_tf_rotmat_rot.argtypes = [
    ctypes.POINTER(RotMat),
    ctypes.POINTER(Vec3),
    ctypes.POINTER(Vec3)
]

libamino.aa_tf_rotmat_mul.argtypes = [
    ctypes.POINTER(RotMat),
    ctypes.POINTER(RotMat),
    ctypes.POINTER(RotMat)
]

libamino.aa_tf_rotmat_inv2.argtypes = [
    ctypes.POINTER(RotMat), ctypes.POINTER(RotMat)
]

libamino.aa_tf_rotmat_lnv.argtypes = [
    ctypes.POINTER(RotMat), ctypes.POINTER(Vec3)
]

# Transforms
libamino.aa_tf_tfmat_tf.argtypes = [
    ctypes.POINTER(TfMat),
    ctypes.POINTER(Vec3),
    ctypes.POINTER(Vec3)
]

libamino.aa_tf_qutr_tf.argtypes = [
    ctypes.POINTER(QuatTrans),
    ctypes.POINTER(Vec3),
    ctypes.POINTER(Vec3)
]

libamino.aa_tf_duqu_tf.argtypes = [
    ctypes.POINTER(DualQuat),
    ctypes.POINTER(Vec3),
    ctypes.POINTER(Vec3)
]

libamino.aa_tf_duqu_ln.argtypes = [
    ctypes.POINTER(DualQuat),
    ctypes.POINTER(DualQuat)
]

libamino.aa_tf_duqu_norm.argtypes = [
    ctypes.POINTER(DualQuat),
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double)
]

libamino.aa_tf_tfmat_mul.argtypes = [
    ctypes.POINTER(TfMat),
    ctypes.POINTER(TfMat),
    ctypes.POINTER(TfMat)
]

libamino.aa_tf_duqu_mul.argtypes = [
    ctypes.POINTER(DualQuat),
    ctypes.POINTER(DualQuat),
    ctypes.POINTER(DualQuat)
]

libamino.aa_tf_qutr_mul.argtypes = [
    ctypes.POINTER(QuatTrans),
    ctypes.POINTER(QuatTrans),
    ctypes.POINTER(QuatTrans)
]

libamino.aa_tf_tfmat_inv2.argtypes = [
    ctypes.POINTER(TfMat), ctypes.POINTER(TfMat)
]

libamino.aa_tf_duqu_conj.argtypes = [
    ctypes.POINTER(DualQuat),
    ctypes.POINTER(DualQuat)
]

libamino.aa_tf_qutr_conj.argtypes = [
    ctypes.POINTER(QuatTrans),
    ctypes.POINTER(QuatTrans)
]

libamino.aa_tf_qutr_conj.argtypes = [
    ctypes.POINTER(QuatTrans),
    ctypes.POINTER(QuatTrans)
]
