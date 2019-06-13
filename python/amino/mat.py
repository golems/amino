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


import ctypes

from lib import libamino

from mixin import VecMixin,SSDEqMixin

CblasNoTrans   = 111
CblasTrans     = 112
CblasConjTrans = 113

class DVec(ctypes.Structure,VecMixin):
    _fields_ = [ ("_size", ctypes.c_size_t),
                 ("_data", ctypes.POINTER(ctypes.c_double)),
                 ("_inc", ctypes.c_size_t) ]

    def __init__(self,arg):
        """Construct a vector.

        If arg is None -- Create an unitialized DVec
        If arg is a list -- Fill the DVec with list contents
        If arg is an int -- Create a DVec of that size
        """
        if( arg is None ):
            pass
        elif isinstance(arg,list) or isinstance(arg,DVec):
            l = len(arg)
            self._allocate(l)
            self.copy_from(arg)
        elif type(arg) == int :
            self._allocate(arg)
        else:
            raise Exception('Invalid argument')

    def _view(self,size,data,inc):
        self._size = size
        self._data = data
        self._inc = inc

    def _slice(self, start, stop, step):
        if( stop > self._size or stop < start or start < 0 ):
            raise IndexError();
        x = DVec(None)
        if step is None:
            step = 1
        libamino.aa_dvec_slice(self,start,stop,step,x)
        return x

    @staticmethod
    def ensure(thing):
        """If thing is not a DVec, construct a DVec containging thing."""
        if( isinstance(thing,DVec) ):
            return thing
        else:
            return DVec(thing)

    @staticmethod
    def create(size):
        """Create a DVec of length size"""
        x = DVec(None)
        x._allocate(size)
        return x

    def copy_from(self,thing):
        """Copy the contents of thing into self"""
        ls = len(self)
        if( ls != len(thing) ):
            raise IndexError()
        elif isinstance(thing,DVec):
            libamino.aa_dvec_copy(thing,self)
        else:
            for i in range(0,ls):
                self[i] = thing[i]
        return thing

    def copy_to(self,thing):
        """Copy self to thing."""
        ls = len(self)
        if( ls != len(thing) ):
            raise IndexError()
        elif isinstance(thing,DVec):
            libamino.aa_dvec_copy(self,thing)
        else:
            for i in range(0,ls):
                thing[i] = self[i]
        return thing

    def _allocate(self,size):
        d = (ctypes.c_double * size)()
        self._view( size,
                    ctypes.cast(d, ctypes.POINTER(ctypes.c_double)),
                    1)

    def axpy(self,alpha,x):
        """self = alpha*x, where alpha is a scalar and x is a DVec"""
        libamino.aa_dvec_axpy(alpha,x,self)
        return self


    def zero(self):
        """Fill self with zeros"""
        libamino.aa_dvec_zero(self)
        return self

    def set(self,alpha):
        """Set all elements of self to scalar alpha"""
        libamino.aa_dvec_set(self,alpha)
        return self

    def increment(self,alpha):
        """self = self + alpha, for scalar alpha"""
        libamino.aa_dvec_inc(alpha,self)
        return self

    def gemv(self,trans,alpha,A,x,beta):
        """General matrix-vector multiply.

        self = alpha*A*x + beta*self

        alpha -- a scalar
        A -- a matrix
        x -- a vector
        beta -- a scalar
        """
        if  A.rows() != len(self):
            raise IndexError()
        if  A.cols() != len(x):
            raise IndexError()
        libamino.aa_dmat_gemv(trans,alpha,A,x,beta,self)
        return self

    def ssd(self,other):
        """Sum of square differences"""
        return libamino.aa_dvec_ssd(self,DVec.ensure(other))

    def nrm2(self):
        """Euclidean norm of self"""
        return libamino.aa_dvec_nrm2(self)

    def __getitem__(self, key):
        """Return an item or slice of self"""
        if isinstance(key,slice):
            return self._slice(key.start,key.stop,key.step)
        elif 0 <= key and key < len(self):
            return self._data[ key * self._inc ]
        else:
            raise IndexError(key)

    def __setitem__(self, key, item):
        """Set an item of self"""
        if 0 <= key and key < len(self):
            self._data[ key * self._inc ] = item
        else:
            raise IndexError(key)

    def __len__(self):
        """Number of elements in self"""
        return self._size

    def inc(self):
        """Increment between elements in self"""
        return self._inc

    def __iadd__(self,other):
        """Add a scalar or vector to self"""
        if isinstance(other,DVec):
            return self.axpy(1,other)
        elif isinstance(other,list):
            return self.axpy(1,DVec(other))
        elif type(other) == int or type(other) == float:
            return self.increment(other)
        else:
            raise Exception('Invalid argument')

    def __add__(self,other):
        """Add a scalar or vector to self"""
        return DVec(self).__iadd__(other)

    def __radd__(self,other):
        """Add a scalar or vector to self"""
        return DVec(self).__iadd__(other)

    def __isub(self,other):
        """Subtract a scalar or vector from self"""
        if isinstance(other,DVec):
            return self.axpy(-1,other)
        if isinstance(other,list):
            return self.axpy(-1,DVec(other))
        elif type(other) == int or type(other) == float:
            return self.increment(-other)
        else:
            raise Exception('Invalid argument')

    def __sub__(self,other):
        """Subtract a scalar or vector from self"""
        if isinstance(other,DVec):
            return DVec(self).axpy(-1,other)
        elif isinstance(other,list):
            return self.__sub__(DVec(other))
        elif type(other) == int or type(other) == float:
            return DVec(self).increment(-other)
        else:
            raise Exception('Invalid argument')

    def __rsub__(self,other):
        """Subtract a self from a scalar or vector"""
        if type(other) == int or type(other) == float:
            return self.__neg__().increment(other)
        elif isinstance(other,list):
            return DVec(other).axpy(-1,self)
        else:
            raise Exception('Invalid argument')

    def __imul__(self,other):
        """Multiply self by a scalar"""
        libamino.aa_dvec_scal(other,self)
        return self

    def __neg__(self):
        """Negate self"""
        return DVec(self).__imul__(-1)

    def __mul__(self,other):
        """Multiply self by a scalar"""
        return DVec(self).__imul__(other)

    def __rmul__(self,other):
        """Multiply self by a scalar"""
        return DVec(self).__imul__(other)

    def __idiv__(self,other):
        """Divide self by a scalar"""
        return self.__imul__( 1.0 / other )

    def __div__(self,other):
        """Divide self by a scalar"""
        return DVec(self).__idiv__(other)

    def __str__(self):
        s = "DVec(["
        i = 0
        n = len(self)
        while i < n - 1:
            s += "%f, " % self._data[self._inc * i]
            i = i+1
        if( i < n ):
            s += "%f" % self._data[self._inc * i]
        s += "])"
        return s

class DMat(ctypes.Structure,SSDEqMixin):
    _fields_ = [ ("_rows", ctypes.c_size_t),
                 ("_cols", ctypes.c_size_t),
                 ("_data", ctypes.POINTER(ctypes.c_double)),
                 ("_ld", ctypes.c_size_t) ]

    def __init__(self,arg=None):
        if arg is None:
            pass
        elif isinstance(arg,tuple):
            rows,cols = arg
            self._allocate(rows,cols)
        elif isinstance(arg,DMat):
            self._allocate(arg.rows(),arg.cols())
            self.copy_from(arg)
        else:
            raise Exception('Invalid argument')

    def copy_from(self,other):
        if self.rows() != other.rows() or self.cols() != other.cols():
            raise IndexError()
        if isinstance(other,DMat):
            libamino.aa_dmat_copy(other,self)

    def _allocate(self,rows,cols,ld=None):
        self._rows = rows
        self._cols = cols
        if ld is None:
            self._ld = rows
        else:
            self._ld = ld
        d = (ctypes.c_double * (rows*cols))()
        self._data = ctypes.cast(d, ctypes.POINTER(ctypes.c_double))

    @staticmethod
    def create(rows,cols):
        x = DMat(None)
        x._allocate(rows,cols)
        return x

    def rows(self):
        return self._rows
    def cols(self):
        return self._cols
    def ld(self):
        return self._ld
    def data(self):
        return self._data
    def __len__(self):
        return self.rows()*self.cols()

    def row_vec(self,i):
        self._check_row(i)
        v = DVec(None)
        libamino.aa_dmat_row_vec(self,i,v)
        return v

    def col_vec(self,j):
        self._check_col(j)
        v = DVec(None)
        libamino.aa_dmat_col_vec(self,j,v)
        return v

    def diag_vec(self):
        v = DVec(None)
        libamino.aa_dmat_diag_vec(self,v)
        return v

    def block(self,row_start,col_start,row_end,col_end):
        M = DMat(None)
        libamino.aa_dmat_block(self,row_start,col_start,
                               row_end,col_end,
                               M)
        return M

    def transpose(self):
        """Return the tranpose of the matrix"""
        At = DMat.create(self.cols(),self.rows())
        libamino.aa_dmat_trans(self,At)
        return At

    def t(self):
        """Synonym for self.transpose()"""
        return self.transpose()

    @staticmethod
    def row_matrix(args):
        m = len(args)
        n = len(args[0]) if m > 0 else 0
        A = DMat.create(m,n)
        for i in range(0,m):
            A.row_vec(i).copy_from(args[i])
        return A

    @staticmethod
    def col_matrix(args):
        n = len(args)
        m = len(args[0]) if n > 0 else 0
        A = DMat.create(m,n)
        for j in range(0,n):
            A.col_vec(j).copy_from(args[j])
        return A

    def ssd(self,other):
        return libamino.aa_dmat_ssd(self,other)

    def nrm2(self):
        return libamino.aa_dmat_nrm2(self)

    def gemm(self,transA,transB,alpha,A,B,beta):
        C = self
        if A.rows() != C.rows():
            raise IndexError()
        if A.cols() != B.rows():
            raise IndexError()
        if B.cols() != C.cols():
            raise IndexError()
        libamino.aa_dmat_gemm(transA,transB,alpha,A,B,beta,C)
        return C

    def pinv(self, tol=-1):
        M = DMat.create( self.cols(), self.rows() )
        libamino.aa_dmat_pinv(self,tol,M)
        return M

    def inv(self):
        M = DMat(self)
        libamino.aa_dmat_inv(M)
        return M

    def _check_row(self, i):
        if( i < 0 or i >= self._rows ):
            raise IndexError(i)
    def _check_col(self, j):
        if( j < 0 or j >= self._cols ):
            raise IndexError(j)
    def _check_index(self, i, j):
        self._check_row(i)
        self._check_col(j)

    def __getitem__(self, key):
        i,j = key
        if isinstance(i,slice) and isinstance(j,slice):
            if not ( (i.step is None) or i.step == 0 ):
                raise IndexError()
            if not ( (j.step is None) or j.step == 0 ):
                raise IndexError()
            return self.block(i.start,j.start,i.stop,j.stop)
        else:
            self._check_index(i,j)
            return self._data[ i + j*self._ld]

    def __setitem__(self, key, item):
        i,j = key
        self._check_index(i,j)
        self._data[ i + j*self._ld] = item

    def __imul__(self, other):
        if isinstance(other,int) or isinstance(other,float):
            libamino.aa_dmat_scal(self,other)
            return self
        else:
            raise TypeError('Cannot cale matrix with %s'%type(other))

    def __neg__(self):
        """Negate self"""
        return DMat(self).__imul__(-1)

    def __mul__(self, other):
        if isinstance(other,int) or isinstance(other,float):
            return DMat(self).__imul__(other)
        elif isinstance(other,DVec) :
            y = DVec.create(self.rows())
            y.gemv(CblasNoTrans,1,self,other,0)
            return y
        elif isinstance(other,list) :
            return self * DVec(other)
        elif isinstance(other,DMat) :
            A,B = self,other
            C = DMat.create(A.rows(), B.cols())
            return C.gemm(CblasNoTrans,CblasNoTrans,1,A,B,1)
        else:
            raise TypeError('Cannot multiply matrix with %s'%type(other))

    def __rmul__(self, other):
        if isinstance(other,int) or isinstance(other,float):
            return DMat(self).__imul__(other)
        else:
            raise TypeError('Cannot multiply matrix with %s'%type(other))

    def __idiv__(self,other):
        """Divide self by a scalar"""
        return self.__imul__( 1.0 / other )

    def __div__(self,other):
        """Divide self by a scalar"""
        return DMat(self).__idiv__(other)

    def __iadd__(self, other):
        if isinstance(other,int) or isinstance(other,float):
            libamino.aa_dmat_inc(self,other)
            return self
        if isinstance(other,DMat):
            libamino.aa_dmat_axpy(1,other,self)
            return self
        else:
            raise TypeError('Cannot increment matrix with %s'%type(other))

    def __add__(self, other):
        if isinstance(other,int) or isinstance(other,float) or isinstance(other,DMat):
            return DMat(self).__iadd__(other)
        else:
            raise TypeError('Cannot add matrix add %s'%type(other))

    def __radd__(self, other):
        if isinstance(other,int) or isinstance(other,float):
            return DMat(self).__iadd__(other)
        else:
            raise TypeError('Cannot add matrix add %s'%type(other))

    def __isub__(self, other):
        if isinstance(other,int) or isinstance(other,float):
            libamino.aa_dmat_inc(self,-other)
            return self
        if isinstance(other,DMat):
            libamino.aa_dmat_axpy(-1,other,self)
            return self
        else:
            raise TypeError('Cannot increment matrix with %s'%type(other))

    def __sub__(self, other):
        if isinstance(other,int) or isinstance(other,float) or isinstance(other,DMat):
            return DMat(self).__isub__(other)
        else:
            raise TypeError('Cannot subtract matrix with %s'%type(other))

    def __rsub__(self, other):
        if isinstance(other,int) or isinstance(other,float):
            return self.__neg__().__iadd__(other)
        else:
            raise TypeError('Cannot subtract matrix with %s'%type(other))

    def __str__(self):
        m = self.rows()
        n = self.cols()
        s = "DMat.row_matrix("
        for i in range(0,m):
            if i == 0:
                s += "["
            else:
                s += ",\n                ["
            for j in range(0,n):
                if j == 0:
                    s += "%f" % self[i,j]
                else:
                    s += ", %f" % self[i,j]
            s += "]"
        s += ")"
        return s

#---------------#
# LIBRARY CALLS #
#---------------#

## Blas 1
libamino.aa_dvec_axpy.argtypes = [ctypes.c_double, ctypes.POINTER(DVec), ctypes.POINTER(DVec)]
libamino.aa_dvec_copy.argtypes = [ctypes.POINTER(DVec), ctypes.POINTER(DVec)]
libamino.aa_dvec_scal.argtypes = [ctypes.c_double, ctypes.POINTER(DVec)]
libamino.aa_dvec_inc.argtypes = [ctypes.c_double, ctypes.POINTER(DVec)]

libamino.aa_dvec_dot.argtypes = [ctypes.POINTER(DVec), ctypes.POINTER(DVec)]
libamino.aa_dvec_dot.restype = ctypes.c_double

libamino.aa_dvec_nrm2.argtypes = [ctypes.POINTER(DVec)]
libamino.aa_dvec_nrm2.restype = ctypes.c_double

libamino.aa_dvec_set.argtypes = [ctypes.POINTER(DVec), ctypes.c_double]


libamino.aa_dvec_zero.argtypes = [ctypes.POINTER(DVec)]

libamino.aa_dvec_slice.argtypes = [ ctypes.POINTER(DVec),
                                    ctypes.c_size_t, ctypes.c_size_t, ctypes.c_size_t,
                                    ctypes.POINTER(DVec) ]

libamino.aa_dvec_ssd.argtypes = [ctypes.POINTER(DVec),ctypes.POINTER(DVec)]
libamino.aa_dvec_ssd.restype = ctypes.c_double

# Blas 2
libamino.aa_dmat_row_vec.argtypes = [ ctypes.POINTER(DMat), ctypes.c_size_t,
                                      ctypes.POINTER(DVec) ]
libamino.aa_dmat_col_vec.argtypes = [ ctypes.POINTER(DMat), ctypes.c_size_t,
                                      ctypes.POINTER(DVec) ]
libamino.aa_dmat_diag_vec.argtypes = [ ctypes.POINTER(DMat), ctypes.POINTER(DVec) ]

libamino.aa_dmat_block.argtypes = [ ctypes.POINTER(DMat),
                                    ctypes.c_size_t, ctypes.c_size_t,
                                    ctypes.c_size_t, ctypes.c_size_t,
                                    ctypes.POINTER(DMat) ]

libamino.aa_dmat_gemv.argtypes = [ ctypes.c_int,
                                  ctypes.c_double, ctypes.POINTER(DMat), ctypes.POINTER(DVec),
                                  ctypes.c_double, ctypes.POINTER(DVec) ]


# Blas 3
libamino.aa_dmat_gemm.argtypes = [ ctypes.c_int, ctypes.c_int,
                                  ctypes.c_double, ctypes.POINTER(DMat), ctypes.POINTER(DMat),
                                  ctypes.c_double, ctypes.POINTER(DMat) ]

# Matrix functions
libamino.aa_dmat_ssd.argtypes = [ctypes.POINTER(DMat),ctypes.POINTER(DMat)]
libamino.aa_dmat_ssd.restype = ctypes.c_double

libamino.aa_dmat_scal.argtypes = [ctypes.POINTER(DMat),ctypes.c_double]
libamino.aa_dmat_inc.argtypes = [ctypes.POINTER(DMat),ctypes.c_double]
libamino.aa_dmat_axpy.argtypes = [ ctypes.c_double, ctypes.POINTER(DMat), ctypes.POINTER(DMat)]

libamino.aa_dmat_nrm2.argtypes = [ctypes.POINTER(DMat)]
libamino.aa_dmat_nrm2.restype = ctypes.c_double

libamino.aa_dmat_trans.argtypes = [ctypes.POINTER(DMat),ctypes.POINTER(DMat)]

libamino.aa_dmat_pinv.argtypes = [ctypes.POINTER(DMat),
                                  ctypes.c_double,
                                  ctypes.POINTER(DMat)]
libamino.aa_dmat_inv.argtypes = [ctypes.POINTER(DMat)]

libamino.aa_dmat_copy.argtypes = [ctypes.POINTER(DMat),ctypes.POINTER(DMat)]
