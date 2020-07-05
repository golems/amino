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

# Object must provide .ssd(other), .nrm2(), .__setitem__()
"""Helper mixins."""


class SSDEqMixin:
    """Equality test mixin using sum-square-differences."""

    def __eq__(self, other):
        return self.ssd(other) == 0

    def __ne__(self, other):
        return self.ssd(other) != 0

    def isclose(self, other, rel_tol=1e-09, abs_tol=0.0):
        """Returns true if object is close to other."""
        a = self
        b = other
        na = a.nrm2()
        nb = b.nrm2()
        d = a.ssd(b)
        return d <= max(rel_tol * max(na, nb), abs_tol)


class CopyEltsMixin:
    """Copy Elements mixin."""

    def copy_from(self, src):
        """Copy elements from src to self"""
        n = len(self)
        if n != len(src):
            raise IndexError()
        for i in range(0, n):
            self[i] = src[i]
        return self

    def copy_to(self, dst):
        """Copy elements from self to dst"""
        n = len(self)
        if n != len(dst):
            raise IndexError()
        for i in range(0, n):
            dst[i] = self[i]
        return self


class DivCompatMixin:
    """Mixin for compatibility division operator."""
    def __div__(self, other):
        return self.__truediv__(other)

    def __idiv__(self, other):
        """Divide self by a scalar"""
        return self.__itruediv__(other)


class VecMixin(CopyEltsMixin, SSDEqMixin, DivCompatMixin):
    """Mixin for vector-like objects."""

    def __radd__(self, other):
        return self + other


class MatMixin:
    """Mixin for matrix-like objects."""
    def _str_helper(self, name, m=None, n=None):
        if m is None:
            m = self.rows
        if n is None:
            n = self.cols
        s = "%s([" % name
        spaces = " "*len(s)
        newrow = ",\n%s[" % spaces
        for i in range(0, m):
            if i == 0:
                s += "["
            else:
                s += newrow
            for j in range(0, n):
                if j == 0:
                    s += "%f" % self[i, j]
                else:
                    s += ", %f" % self[i, j]
            s += "]"
        s += "])"
        return s
