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
## @file mat.py Vectors and Matrices
##
"""Vectors and Matrices"""

import ctypes

from amino.lib import libamino
from amino.mixin import VecMixin, SSDEqMixin, DivCompatMixin, MatMixin
from amino.util import ensure, is_int, is_scalar

CBLAS_NO_TRANS = 111
CBLAS_TRANS = 112
CBLAS_CONJ_TRANS = 113


class DVec(ctypes.Structure, VecMixin):
    """Vector of double floats."""
    _fields_ = [("_size", ctypes.c_size_t),
                ("_data", ctypes.POINTER(ctypes.c_double)),
                ("_inc", ctypes.c_size_t)]

    @staticmethod
    def _alloc(size):
        return (ctypes.c_double * size)()

    def __init__(self, arg=None):
        """Constructs a vector.

        If arg is None -- Create an unitialized DVec
        If arg is a list -- Fill the DVec with list contents
        If arg is an int -- Create a DVec of that size
        """
        if arg is None:
            super().__init__(0, None, 0)
        elif is_int(arg):
            super().__init__(arg, DVec._alloc(arg), 1)
        else:
            n = len(arg)
            super().__init__(n, DVec._alloc(n), 1)
            self.copy_from(arg)

    @staticmethod
    def ensure(thing):
        """If thing is not a DVec, construct a DVec containging thing.

        Returns:
           thing if already a DVec, other a newly-constructed DVec.
        """
        return ensure(thing, DVec)

    def copy_from(self, src):
        """Copies the contents of thing into self"""
        if len(self) != len(src):
            raise IndexError()
        if isinstance(src, DVec):
            libamino.aa_dvec_copy(src, self)
        else:
            super().copy_from(src)
        return src

    def copy_to(self, dst):
        """Copies self to thing."""
        if len(self) != len(dst):
            raise IndexError()
        if isinstance(dst, DVec):
            libamino.aa_dvec_copy(self, dst)
        else:
            super().copy_to(dst)
        return dst

    def axpy(self, alpha, x):
        """self = alpha*x, where alpha is a scalar and x is a DVec"""
        libamino.aa_dvec_axpy(alpha, x, self)
        return self

    def zero(self):
        """Fills self with zeros"""
        libamino.aa_dvec_zero(self)
        return self

    def set(self, alpha):
        """Sets all elements of self to scalar alpha"""
        libamino.aa_dvec_set(self, alpha)
        return self

    def increment(self, alpha):
        """self = self + alpha, for scalar alpha"""
        libamino.aa_dvec_inc(alpha, self)
        return self

    def gemv(self, trans, alpha, A, x, beta):
        """General matrix-vector multiply.

        self = alpha*A*x + beta*self

        alpha -- a scalar
        A -- a matrix
        x -- a vector
        beta -- a scalar
        """
        if A.rows != len(self):
            raise IndexError()
        if A.cols != len(x):
            raise IndexError()
        libamino.aa_dmat_gemv(trans, alpha, A, x, beta, self)
        return self

    def ssd(self, other):
        """Sum of square differences"""
        return libamino.aa_dvec_ssd(self, DVec.ensure(other))

    def nrm2(self):
        """Euclidean norm of self"""
        return libamino.aa_dvec_nrm2(self)

    def __getitem__(self, key):
        """Returns an item or slice of self"""
        if isinstance(key, slice):
            step = 1 if key.step is None else key.step
            start = 0 if key.start is None else key.start
            stop = self._size if key.stop is None else key.stop
            x = DVec()
            libamino.aa_dvec_slice(self, start, stop, step, x)
            return x
        if key < 0 or key >= len(self):
            raise IndexError(key)
        return self._data[key * self._inc]

    def __setitem__(self, key, item):
        """Set an item of self"""
        if key < 0 or key >= len(self):
            raise IndexError(key)
        self._data[key * self._inc] = item

    def __len__(self):
        """Number of elements in self"""
        return self._size

    @property
    def inc(self):
        """Increment between elements."""
        return self._inc

    def __iadd__(self, other):
        """Add a scalar or vector to self"""
        if isinstance(other, DVec):
            return self.axpy(1, other)
        if isinstance(other, list):
            return self.axpy(1, DVec(other))
        if is_scalar(other):
            return self.increment(other)
        raise Exception('Invalid argument')

    def __add__(self, other):
        """Add a scalar or vector to self"""
        return DVec(self).__iadd__(other)

    def __radd__(self, other):
        """Add a scalar or vector to self"""
        return DVec(self).__iadd__(other)

    def __isub(self, other):
        """Subtract a scalar or vector from self"""
        if isinstance(other, DVec):
            return self.axpy(-1, other)
        if isinstance(other, list):
            return self.axpy(-1, DVec(other))
        if is_scalar(other):
            return self.increment(-other)
        raise Exception('Invalid argument')

    def __sub__(self, other):
        """Subtract a scalar or vector from self"""
        if isinstance(other, DVec):
            return DVec(self).axpy(-1, other)
        if isinstance(other, list):
            return self.__sub__(DVec(other))
        if is_scalar(other):
            return DVec(self).increment(-other)
        raise Exception('Invalid argument')

    def __rsub__(self, other):
        """Subtract a self from a scalar or vector"""
        if is_scalar(other):
            return self.__neg__().increment(other)
        if isinstance(other, list):
            return DVec(other).axpy(-1, self)
        raise Exception('Invalid argument')

    def __imul__(self, other):
        """Multiply self by a scalar"""
        libamino.aa_dvec_scal(other, self)
        return self

    def __neg__(self):
        """Negate self"""
        return DVec(self).__imul__(-1)

    def __mul__(self, other):
        """Multiply self by a scalar"""
        return DVec(self).__imul__(other)

    def __rmul__(self, other):
        """Multiply self by a scalar"""
        return DVec(self).__imul__(other)

    def __itruediv__(self, other):
        """Divide self by a scalar"""
        return self.__imul__(1.0 / other)

    def __truediv__(self, other):
        """Divide self by a scalar"""
        return DVec(self).__itruediv__(other)

    def __str__(self):
        string = "DVec(["
        i = 0
        n = len(self)
        while i < n - 1:
            string += "%f, " % self._data[self._inc * i]
            i = i + 1
        if i < n:
            string += "%f" % self._data[self._inc * i]
        string += "])"
        return string


class DMat(ctypes.Structure, SSDEqMixin, DivCompatMixin, MatMixin):
    """Matrix of double floats."""
    _fields_ = [("_rows", ctypes.c_size_t), ("_cols", ctypes.c_size_t),
                ("_data", ctypes.POINTER(ctypes.c_double)),
                ("_ld", ctypes.c_size_t)]

    @staticmethod
    def _alloc(rows, cols):
        return (ctypes.c_double * (rows * cols))()

    def __init__(self, arg=None):
        """Constructs a matrix."""
        if arg is None:
            super().__init__(0, 0, None, 0)
        elif isinstance(arg, tuple):
            rows, cols = arg
            data = DMat._alloc(rows, cols)
            super().__init__(rows, cols, data, rows)
        elif isinstance(arg, DMat):
            rows = arg.rows
            cols = arg.cols
            data = DMat._alloc(arg.rows, arg.cols)
            super().__init__(rows, cols, data, rows)
            self.copy_from(arg)
        else:
            raise Exception('Invalid argument')

    def copy_from(self, src):
        """Copies elements from src into self"""
        if self.rows != src.rows or self.cols != src.cols:
            raise IndexError()
        if isinstance(src, DMat):
            libamino.aa_dmat_copy(src, self)

    @property
    def rows(self):
        """Number of rows."""
        return self._rows

    @property
    def cols(self):
        """Number of columns."""
        return self._cols

    @property
    def ld(self):
        """Leading dimension of data."""
        return self._ld

    @property
    def data(self):
        """Pointer to data."""
        return self._data

    def __len__(self):
        return self._rows * self._cols

    def row_vec(self, i):
        """Returns the i'th row as a DVec."""
        self._check_row(i)
        v = DVec()
        libamino.aa_dmat_row_vec(self, i, v)
        return v

    def col_vec(self, j):
        """Returns the j'th column as a DVec."""
        self._check_col(j)
        v = DVec()
        libamino.aa_dmat_col_vec(self, j, v)
        return v

    def diag_vec(self):
        """Returns the diagonal as a DVec."""
        v = DVec()
        libamino.aa_dmat_diag_vec(self, v)
        return v

    def transpose(self):
        """Returns the tranpose of the matrix"""
        At = DMat((self._cols, self._rows))
        libamino.aa_dmat_trans(self, At)
        return At

    @staticmethod
    def row_matrix(args):
        """Creates a matrix from rows passed in args."""
        m = len(args)
        n = len(args[0]) if m > 0 else 0
        A = DMat((m, n))
        for i in range(0, m):
            A.row_vec(i).copy_from(args[i])
        return A

    @staticmethod
    def col_matrix(args):
        """Creates a matrix from columns passed in args."""
        n = len(args)
        m = len(args[0]) if n > 0 else 0
        A = DMat((m, n))
        for j in range(0, n):
            A.col_vec(j).copy_from(args[j])
        return A

    def ssd(self, other):
        """Returns Sum of square differences."""
        return libamino.aa_dmat_ssd(self, other)

    def nrm2(self):
        """Returns the Euclidean norm of self"""
        return libamino.aa_dmat_nrm2(self)

    def gemm(self, transA, transB, alpha, A, B, beta):
        """General matrix-matrix multiply.

        Thin wrapper of BLAS dgemm().

        self := alpha*op(A)*op(B) + beta*self

        Args:
            transA: whether to transpose A
            transB: whether to transpose B
            alpha: scale factor for A and B
            beta: scale factor for self
        """
        C = self
        if (A.rows != C.rows or A.cols != B.rows or B.cols != C.cols):
            raise IndexError()
        libamino.aa_dmat_gemm(transA, transB, alpha, A, B, beta, C)
        return C

    def pinv(self, tol=-1):
        """Returns the pseudoinverse.

        Args:
           tol: singular values less than tol are ignored.  Negative values use a default.
        """
        M = DMat((self._cols, self._rows))
        libamino.aa_dmat_pinv(self, tol, M)
        return M

    def inv(self):
        """Returns the inverse."""
        M = DMat(self)
        libamino.aa_dmat_inv(M)
        return M

    def _check_row(self, i):
        if (i < 0 or i >= self._rows):
            raise IndexError(i)

    def _check_col(self, j):
        if (j < 0 or j >= self._cols):
            raise IndexError(j)

    def _check_index(self, i, j):
        self._check_row(i)
        self._check_col(j)

    #def block(self, row_start, col_start, row_end, col_end):

    def __getitem__(self, key):
        i, j = key
        if isinstance(i, slice) and isinstance(j, slice):
            if not ((i.step is None) or i.step == 0):
                raise IndexError()
            if not ((j.step is None) or j.step == 0):
                raise IndexError()
            M = DMat()
            libamino.aa_dmat_block(self, i.start, j.start, i.stop, j.stop, M)
            return M
        self._check_index(i, j)
        return self._data[i + j * self._ld]

    def __setitem__(self, key, item):
        i, j = key
        self._check_index(i, j)
        self._data[i + j * self._ld] = item

    def __imul__(self, other):
        if not is_scalar(other):
            raise TypeError('Cannot scale matrix with %s' % type(other))
        libamino.aa_dmat_scal(self, other)
        return self

    def __neg__(self):
        """Negate self"""
        return DMat(self).__imul__(-1)

    def __mul__(self, other):
        if is_scalar(other):
            return DMat(self).__imul__(other)
        if isinstance(other, DVec):
            y = DVec(self._rows)
            y.gemv(CBLAS_NO_TRANS, 1, self, other, 0)
            return y
        if isinstance(other, list):
            return self * DVec(other)
        if isinstance(other, DMat):
            A, B = self, other
            C = DMat((A._rows, B._cols))
            return C.gemm(CBLAS_NO_TRANS, CBLAS_NO_TRANS, 1, A, B, 1)
        raise TypeError('Cannot multiply matrix with %s' % type(other))

    def __rmul__(self, other):
        if is_scalar(other):
            return DMat(self).__imul__(other)
        raise TypeError('Cannot multiply matrix with %s' % type(other))

    def __itruediv__(self, other):
        """Divide self by a scalar"""
        return self.__imul__(1.0 / other)

    def __truediv__(self, other):
        """Divide self by a scalar"""
        return DMat(self).__itruediv__(other)

    def __iadd__(self, other):
        if is_scalar(other):
            libamino.aa_dmat_inc(self, other)
            return self
        if isinstance(other, DMat):
            libamino.aa_dmat_axpy(1, other, self)
            return self
        raise TypeError('Cannot increment matrix with %s' % type(other))

    def __add__(self, other):
        if is_scalar(other) or isinstance(other, DMat):
            return DMat(self).__iadd__(other)
        raise TypeError('Cannot add matrix add %s' % type(other))

    def __radd__(self, other):
        if is_scalar(other):
            return DMat(self).__iadd__(other)
        raise TypeError('Cannot add matrix add %s' % type(other))

    def __isub__(self, other):
        if is_scalar(other):
            libamino.aa_dmat_inc(self, -other)
            return self
        if isinstance(other, DMat):
            libamino.aa_dmat_axpy(-1, other, self)
            return self
        raise TypeError('Cannot increment matrix with %s' % type(other))

    def __sub__(self, other):
        if is_scalar(other) or isinstance(other, DMat):
            return DMat(self).__isub__(other)
        raise TypeError('Cannot subtract matrix with %s' % type(other))

    def __rsub__(self, other):
        if is_scalar(other):
            return self.__neg__().__iadd__(other)
        raise TypeError('Cannot subtract matrix with %s' % type(other))

    def __str__(self):
        return self._str_helper("TfMat.row_matrix")


#---------------#
# LIBRARY CALLS #
#---------------#

## Blas 1
libamino.aa_dvec_axpy.argtypes = [
    ctypes.c_double,
    ctypes.POINTER(DVec),
    ctypes.POINTER(DVec)
]
libamino.aa_dvec_copy.argtypes = [ctypes.POINTER(DVec), ctypes.POINTER(DVec)]
libamino.aa_dvec_scal.argtypes = [ctypes.c_double, ctypes.POINTER(DVec)]
libamino.aa_dvec_inc.argtypes = [ctypes.c_double, ctypes.POINTER(DVec)]

libamino.aa_dvec_dot.argtypes = [ctypes.POINTER(DVec), ctypes.POINTER(DVec)]
libamino.aa_dvec_dot.restype = ctypes.c_double

libamino.aa_dvec_nrm2.argtypes = [ctypes.POINTER(DVec)]
libamino.aa_dvec_nrm2.restype = ctypes.c_double

libamino.aa_dvec_set.argtypes = [ctypes.POINTER(DVec), ctypes.c_double]

libamino.aa_dvec_zero.argtypes = [ctypes.POINTER(DVec)]

libamino.aa_dvec_slice.argtypes = [
    ctypes.POINTER(DVec), ctypes.c_size_t, ctypes.c_size_t, ctypes.c_size_t,
    ctypes.POINTER(DVec)
]

libamino.aa_dvec_ssd.argtypes = [ctypes.POINTER(DVec), ctypes.POINTER(DVec)]
libamino.aa_dvec_ssd.restype = ctypes.c_double

# Blas 2
libamino.aa_dmat_row_vec.argtypes = [
    ctypes.POINTER(DMat), ctypes.c_size_t,
    ctypes.POINTER(DVec)
]
libamino.aa_dmat_col_vec.argtypes = [
    ctypes.POINTER(DMat), ctypes.c_size_t,
    ctypes.POINTER(DVec)
]
libamino.aa_dmat_diag_vec.argtypes = [
    ctypes.POINTER(DMat), ctypes.POINTER(DVec)
]

libamino.aa_dmat_block.argtypes = [
    ctypes.POINTER(DMat), ctypes.c_size_t, ctypes.c_size_t, ctypes.c_size_t,
    ctypes.c_size_t,
    ctypes.POINTER(DMat)
]

libamino.aa_dmat_gemv.argtypes = [
    ctypes.c_int, ctypes.c_double,
    ctypes.POINTER(DMat),
    ctypes.POINTER(DVec), ctypes.c_double,
    ctypes.POINTER(DVec)
]

# Blas 3
libamino.aa_dmat_gemm.argtypes = [
    ctypes.c_int, ctypes.c_int, ctypes.c_double,
    ctypes.POINTER(DMat),
    ctypes.POINTER(DMat), ctypes.c_double,
    ctypes.POINTER(DMat)
]

# Matrix functions
libamino.aa_dmat_ssd.argtypes = [ctypes.POINTER(DMat), ctypes.POINTER(DMat)]
libamino.aa_dmat_ssd.restype = ctypes.c_double

libamino.aa_dmat_scal.argtypes = [ctypes.POINTER(DMat), ctypes.c_double]
libamino.aa_dmat_inc.argtypes = [ctypes.POINTER(DMat), ctypes.c_double]
libamino.aa_dmat_axpy.argtypes = [
    ctypes.c_double,
    ctypes.POINTER(DMat),
    ctypes.POINTER(DMat)
]

libamino.aa_dmat_nrm2.argtypes = [ctypes.POINTER(DMat)]
libamino.aa_dmat_nrm2.restype = ctypes.c_double

libamino.aa_dmat_trans.argtypes = [ctypes.POINTER(DMat), ctypes.POINTER(DMat)]

libamino.aa_dmat_pinv.argtypes = [
    ctypes.POINTER(DMat), ctypes.c_double,
    ctypes.POINTER(DMat)
]
libamino.aa_dmat_inv.argtypes = [ctypes.POINTER(DMat)]

libamino.aa_dmat_copy.argtypes = [ctypes.POINTER(DMat), ctypes.POINTER(DMat)]
