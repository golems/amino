#!/usr/bin/env python
#
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


import unittest

from amino import *

import math

class TestVec0(unittest.TestCase):
    def test_create(self):
        v = DVec.create(2)
        self.assertEqual(len(v),2)


    def test_copy_from(self):
        v = DVec.create(2)
        lst = [11,17]
        v.copy_from(lst)
        self.assertEqual(v[0], lst[0])
        self.assertEqual(v[1], lst[1])

    def test_copy_to(self):
        lst = [11,17]
        v = DVec(lst)
        self.assertEqual(v[0], lst[0])
        self.assertEqual(v[1], lst[1])
        ll = [0,0]
        v.copy_to(ll)
        self.assertEqual(ll[0], lst[0])
        self.assertEqual(ll[1], lst[1])


    def test_getsetitem(self):
        v = DVec.create(3)
        v[0] = 3
        v[1] = 5
        v[2] = 7
        self.assertEqual(v[0], 3.0)
        self.assertEqual(v[1], 5.0)
        self.assertEqual(v[2], 7.0)

        self.assertRaises( IndexError, lambda : v[3] )

    def test_nrm2(self):
        l = [1,2,3]
        x = DVec(l)
        self.assertTrue( abs(x.nrm2() -
                             math.sqrt(l[0]**2 + l[1]**2 + l[2]**2))
                         < 1e-6 )

    def test_ssd(self):
        v = DVec([3,5,7])
        v[0] = 3
        v[1] = 5
        v[2] = 7
        self.assertEqual(v.ssd([3,5,7]), 0)
        self.assertEqual(v.ssd([2,5,7]), 1)
        self.assertEqual(v.ssd([2,6,7]), 2)
        self.assertEqual(v.ssd([1,5,7]), 4)

    def test_scal(self):
        v = DVec([3,5,7])
        self.assertEqual((v*2), [6,10,14] )
        self.assertEqual((2*v), [6,10,14] )

    def test_sadd(self):
        v = DVec([3,5,7])
        self.assertEqual((v+2), [5,7,9])
        self.assertEqual((2+v), [5,7,9])

    def test_neg(self):
        v = DVec([3,5,7])
        self.assertEqual((-v), [-3,-5,-7])

    def test_subb(self):
        v = DVec([3,5,7])
        self.assertEqual((v-2), [1,3,5])
        self.assertEqual((2-v), [-1,-3,-5])

    def test_vadd(self):
        v = DVec([3,5,7])
        self.assertEqual( (v + [1,2,3]), [4,7,10] )
        self.assertEqual( ([1,2,3] + v), [4,7,10] )

    def test_vsub(self):
        v = DVec([3,5,7])
        self.assertEqual( (v - [1,2,3]), [2,3,4] )
        self.assertEqual( ([1,2,3] - v), [-2,-3,-4] )

    def test_set(self):
        v = DVec([3,5,7])
        v.set(1)
        self.assertEqual( v, [1,1,1])
        v.zero()
        self.assertEqual(v, [0,0,0])
        self.assertNotEqual(v, [0,0,1])

    #def test_getsetitem(self):

class TestMat(unittest.TestCase):
    def test_getsetitem(self):
        A = DMat.create(2,2 )
        A[0,0] = 1
        A[1,0] = 2
        A[0,1] = 3
        A[1,1] = 4

        self.assertEqual(A[0,0], 1)
        self.assertEqual(A[1,0], 2)
        self.assertEqual(A[0,1], 3)
        self.assertEqual(A[1,1], 4)

        self.assertEqual(A.col_vec(0)[0], 1)
        self.assertEqual(A.col_vec(0)[1], 2)
        self.assertEqual(A.col_vec(1)[0], 3)
        self.assertEqual(A.col_vec(1)[1], 4)

        self.assertEqual(A.row_vec(0)[0], 1)
        self.assertEqual(A.row_vec(0)[1], 3)
        self.assertEqual(A.row_vec(1)[0], 2)
        self.assertEqual(A.row_vec(1)[1], 4)

    def test_rowmatrix(self):
        A = DMat.row_matrix( [ [1,2], [3,4] ] )
        self.assertEqual(A[0,0], 1)
        self.assertEqual(A[0,1], 2)
        self.assertEqual(A[1,0], 3)
        self.assertEqual(A[1,1], 4)

        B = DMat.col_matrix( [ [1,3], [2,4] ] )
        self.assertEqual(A.ssd(B), 0)
        B[0,0] = B[0,0]+1
        self.assertEqual(A.ssd(B), 1)

    def test_colmatrix(self):
        A = DMat.col_matrix( [ [1,2], [3,4] ] )
        self.assertEqual(A[0,0], 1)
        self.assertEqual(A[1,0], 2)
        self.assertEqual(A[0,1], 3)
        self.assertEqual(A[1,1], 4)

        B = DMat.row_matrix([ [1,3], [2,4] ])
        self.assertEqual(A.ssd(B), 0)
        B[0,0] = B[0,0]+1
        self.assertEqual(A.ssd(B), 1)

    def test_transpose(self):
        cols = [ [1,2], [3,4], [5,6] ]
        A = DMat.col_matrix(cols)
        At = A.transpose()

        self.assertEqual(At.row_vec(0).ssd(cols[0]), 0)
        self.assertEqual(At.row_vec(1).ssd(cols[1]), 0)
        self.assertEqual(At.row_vec(2).ssd(cols[2]), 0)

        Atx = DMat.row_matrix(cols)
        self.assertEqual(At.ssd(Atx), 0)


class TestGemv(unittest.TestCase):
    def test_gemv(self):
        A = DMat.col_matrix( [ [1,2], [3,4] ] )
        y = A * DVec([5,6])
        self.assertEqual(y.ssd([23,34]), 0)

        y = A * [1,2]
        self.assertEqual(y.ssd([7,10]), 0)

class TestGemm(unittest.TestCase):
    def test_gemm2x2(self):
        A = DMat.col_matrix( [[1,2], [3,4]] )
        B = DMat.col_matrix( [[5,6], [7,8]] )
        AB = DMat.col_matrix

        y = A * DVec([5,6])
        self.assertEqual(y.ssd([23,34]), 0)


    def test_gemm2x3(self):
        A = DMat.row_matrix( [[1,2], [3,4], [5,6]] )
        B = DMat.row_matrix( [[7,8,9], [10,11,12]] )

        AB =  DMat.row_matrix( [[27,  30,  33],
                                [61,  68,  75],
                                [95, 106, 117]] )
        self.assertEqual(AB.ssd(A*B), 0)

        BA = DMat.row_matrix( [[76, 100], [103, 136]] )
        self.assertEqual(BA.ssd(B*A), 0)

if __name__ == '__main__':
    unittest.main()
