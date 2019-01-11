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

class TestVec0(unittest.TestCase):

    def test_getsetitem(self):
        v = DVec.create(3)
        v[0] = 3
        v[1] = 5
        v[2] = 7
        self.assertEqual(v[0], 3.0)
        self.assertEqual(v[1], 5.0)
        self.assertEqual(v[2], 7.0)

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
        self.assertEqual((v*2).ssd([6,10,14]), 0)
        self.assertEqual((2*v).ssd([6,10,14]), 0)

    def test_sadd(self):
        v = DVec([3,5,7])
        self.assertEqual((v+2).ssd([5,7,9]), 0)
        self.assertEqual((2+v).ssd([5,7,9]), 0)

    def test_neg(self):
        v = DVec([3,5,7])
        self.assertEqual((-v).ssd([-3,-5,-7]), 0)

    def test_subb(self):
        v = DVec([3,5,7])
        self.assertEqual((v-2).ssd([1,3,5]), 0)
        self.assertEqual((2-v).ssd([-1,-3,-5]), 0)

    def test_vadd(self):
        v = DVec([3,5,7])
        self.assertEqual( (v + [1,2,3]).ssd( [4,7,10] ), 0 )
        self.assertEqual( ([1,2,3] + v).ssd( [4,7,10] ), 0 )

    def test_vsub(self):
        v = DVec([3,5,7])
        self.assertEqual( (v - [1,2,3]).ssd( [2,3,4] ), 0 )
        self.assertEqual( ([1,2,3] - v).ssd( [-2,-3,-4] ), 0 )

    def test_set(self):
        v = DVec([3,5,7])
        v.set(1)
        self.assertEqual( v.ssd([1,1,1]), 0)
        v.zero()
        self.assertEqual( v.ssd([0,0,0]), 0)

    #def test_getsetitem(self):


if __name__ == '__main__':
    unittest.main()
