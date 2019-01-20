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
        if( isinstance(thing,DVec) ):
            return thing
        else:
            return DVec(thing)

    @staticmethod
    def create(size):
        x = DVec(None)
        x._allocate(size)
        return x

    def copy_from(self,thing):
        ls = len(self)
        if( ls != len(thing) ):
            raise IndexError()
        elif isinstance(thing,DVec):
            libamino.aa_lb_dcopy(thing,self)
        else:
            for i in range(0,ls):
                self[i] = thing[i]
        return thing

    def copy_to(self,thing):
        ls = len(self)
        if( ls != len(thing) ):
            raise IndexError()
        elif isinstance(thing,DVec):
            libamino.aa_lb_dcopy(self,thing)
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
        libamino.aa_lb_daxpy(alpha,x,self)
        return self

    def scal(self,alpha):
        libamino.aa_lb_dscal(alpha,self)
        return self

    def negate(self):
        self.scal(-1)
        return self

    def __neg__(self):
        return DVec(self).negate()

    def zero(self):
        libamino.aa_dvec_zero(self)
        return self

    def set(self,alpha):
        libamino.aa_dvec_set(self,alpha)
        return self

    def increment(self,alpha):
        libamino.aa_lb_dinc(alpha,self)
        return self

    def gemv(self,trans,alpha,A,x,beta):
        if  A.rows() != len(self):
            raise IndexError()
        if  A.cols() != len(x):
            raise IndexError()
        libamino.aa_lb_dgemv(trans,alpha,A,x,beta,self)
        return self

    def ssd(self,other):
        return libamino.aa_dvec_ssd(self,DVec.ensure(other))

    def nrm2(self):
        return libamino.aa_lb_dnrm2(self)

    def __getitem__(self, key):
        if 0 <= key and key < len(self):
            return self._data[ key * self._inc ]
        elif isinstance(key,slice):
            return self._slice(key.start,key.stop,key.step)
        else:
            raise IndexError(key)

    def __setitem__(self, key, item):
        if 0 <= key and key < len(self):
            self._data[ key * self._inc ] = item
        else:
            raise IndexError(key)

    def __len__(self):
        return self._size

    def _mulop(self,other):
        tp = type(other)
        if tp == int or tp == float:
            return DVec(self).scal(other)
        else:
            raise Exception('Invalid argument')

    def __mul__(self,other):
        return self._mulop(other)

    def __rmul__(self,other):
        return self._mulop(other)

    def _addop(self,other):
        if isinstance(other,DVec) or isinstance(other,list):
            return DVec(other).axpy(1,self)
        elif type(other) == int or type(other) == float:
            return DVec(self).increment(other)
        else:
            raise Exception('Invalid argument')

    def __add__(self,other):
        return self._addop(other)

    def __iadd__(self,other):
        if isinstance(other,DVec):
            return self.axpy(1,other)
        if isinstance(other,list):
            return self.axpy(1,DVec(other))
        elif type(other) == int or type(other) == float:
            return self.increment(other)
        else:
            raise Exception('Invalid argument')

    def __isub(self,other):
        if isinstance(other,DVec):
            return self.axpy(-1,other)
        if isinstance(other,list):
            return self.axpy(-1,DVec(other))
        elif type(other) == int or type(other) == float:
            return self.increment(-other)
        else:
            raise Exception('Invalid argument')

    def __sub__(self,other):
        if isinstance(other,DVec):
            return DVec(self).axpy(-1,other)
        elif isinstance(other,list):
            return self - DVec(other)
        elif type(other) == int or type(other) == float:
            return DVec(self).increment(-other)
        else:
            raise Exception('Invalid argument')

    def __rsub__(self,other):
        if type(other) == int or type(other) == float:
            return (-self).increment(other)
        elif isinstance(other,list):
            return DVec(other).axpy(-1,self)
        else:
            raise Exception('Invalid argument')

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

    def transpose(self):
        At = DMat.create(self.cols(),self.rows())
        libamino.aa_dmat_trans(self,At)
        return At

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
        libamino.aa_lb_dgemm(transA,transB,alpha,A,B,beta,C)
        return C

    def pinv(self, tol):
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
        self._check_index(i,j)
        return self._data[ i + j*self._ld]

    def __setitem__(self, key, item):
        i,j = key
        self._check_index(i,j)
        self._data[ i + j*self._ld] = item

    def __mul__(self, other):
        if isinstance(other,DVec) :
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
libamino.aa_lb_daxpy.argtypes = [ctypes.c_double, ctypes.POINTER(DVec), ctypes.POINTER(DVec)]
libamino.aa_lb_dcopy.argtypes = [ctypes.POINTER(DVec), ctypes.POINTER(DVec)]
libamino.aa_lb_dscal.argtypes = [ctypes.c_double, ctypes.POINTER(DVec)]
libamino.aa_lb_dinc.argtypes = [ctypes.c_double, ctypes.POINTER(DVec)]

libamino.aa_lb_ddot.argtypes = [ctypes.POINTER(DVec), ctypes.POINTER(DVec)]
libamino.aa_lb_ddot.restype = ctypes.c_double

libamino.aa_lb_dnrm2.argtypes = [ctypes.POINTER(DVec)]
libamino.aa_lb_dnrm2.restype = ctypes.c_double

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

libamino.aa_lb_dgemv.argtypes = [ ctypes.c_int,
                                  ctypes.c_double, ctypes.POINTER(DMat), ctypes.POINTER(DVec),
                                  ctypes.c_double, ctypes.POINTER(DVec) ]


# Blas 3
libamino.aa_lb_dgemm.argtypes = [ ctypes.c_int, ctypes.c_int,
                                  ctypes.c_double, ctypes.POINTER(DMat), ctypes.POINTER(DMat),
                                  ctypes.c_double, ctypes.POINTER(DMat) ]

# Matrix functions
libamino.aa_dmat_ssd.argtypes = [ctypes.POINTER(DMat),ctypes.POINTER(DMat)]
libamino.aa_dmat_ssd.restype = ctypes.c_double

libamino.aa_dmat_nrm2.argtypes = [ctypes.POINTER(DMat)]
libamino.aa_dmat_nrm2.restype = ctypes.c_double

libamino.aa_dmat_trans.argtypes = [ctypes.POINTER(DMat),ctypes.POINTER(DMat)]

libamino.aa_dmat_pinv.argtypes = [ctypes.POINTER(DMat),
                                  ctypes.c_double,
                                  ctypes.POINTER(DMat)]
libamino.aa_dmat_inv.argtypes = [ctypes.POINTER(DMat)]

libamino.aa_dmat_copy.argtypes = [ctypes.POINTER(DMat),ctypes.POINTER(DMat)]
