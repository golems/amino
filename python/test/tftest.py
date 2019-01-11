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

from math import sqrt

class TestVec3(unittest.TestCase):
    def test_create(self):
        v = Vec3([3,5,7])

        self.assertEqual( len(v), 3 )

        self.assertEqual( v[0], 3 )
        self.assertEqual( v[1], 5 )
        self.assertEqual( v[2], 7 )

        self.assertRaises(IndexError, lambda : v[3])

    def test_nrm(self):
        v = Vec3([1,2,3])
        self.assertTrue( abs(v.nrm2() - sqrt(1+4+9)) < 1e-6 )

    def test_eq(self):
        a = Vec3([1,2,3])
        b = Vec3([1,2,3])
        c = Vec3([2,2,3])
        self.assertTrue( a == b )
        self.assertFalse( a == c )
        self.assertTrue( a != c )
        self.assertFalse( a != b )

        self.assertEqual(a, a)
        self.assertEqual(a, b)
        self.assertNotEqual(a, c)

        self.assertTrue(a == [1,2,3])
        self.assertFalse(a == [1,2,5])
        self.assertTrue([1,2,3] == a)
        self.assertFalse([1,2,4] == a)

        self.assertFalse(a != [1,2,3])
        self.assertTrue(a != [1,2,5])
        self.assertFalse([1,2,3] != a)
        self.assertTrue([1,2,4] != a)

    def test_dot_cross(self):
        v = Vec3([3,5,7])
        l = [17,19,23]
        self.assertEqual( v.dot(l), 307 )
        self.assertEqual( v.cross(l), [-18,50,-28] )
        self.assertTrue( abs(v.ssd(l) - 25.456) < 1e-2 )


class TestQuat(unittest.TestCase):
    def test_create(self):
        h = Quat([1,2,3,4])

        self.assertEqual( len(h), 4 )

        self.assertEqual( h[0], 1 )
        self.assertEqual( h[1], 2 )
        self.assertEqual( h[2], 3 )
        self.assertEqual( h[3], 4 )

        self.assertRaises(IndexError, lambda : h[4])

    def test_norm(self):
        h = Quat([1,2,3,4])
        self.assertTrue( abs(h.nrm2() - 5.4772) < 1e-3 )

    def test_eq(self):
        a = Quat([1,2,3,4])
        l = [1,2,3,4]
        p = [1,2,3,5]
        self.assertTrue(  a == l )
        self.assertFalse( a != l )
        self.assertTrue(  a != p )
        self.assertFalse( a == p )

        self.assertTrue(  l == a )
        self.assertFalse( l != a )
        self.assertTrue(  p != a )
        self.assertFalse( p == a )

    def test_mul(self):
        a = Quat([3,5,7,11])
        b = [13,17,19,23]
        ab = [188, 336, 356, -4]
        ba = [236, 268, 384, -4]

        self.assertEqual(    a*b, ab )
        self.assertNotEqual( a*b, ba )
        self.assertEqual(    b*a, ba )
        self.assertNotEqual( b*a, ab )

    def test_add(self):
        a = Quat([3,5,7,11])
        b = [13,17,19,23]
        apb = [16, 22, 26, 34]
        x = [16, 22, 26, 35]

        self.assertEqual(    a+b, apb )
        self.assertNotEqual( a+b, x )
        self.assertEqual(    b+a, apb )
        self.assertNotEqual( b+a, x )

    def test_sub(self):
        a = Quat([3,5,7,11])
        b = [13,17,19,23]
        amb = [-10, -12, -12, -12]
        bma = [10, 12, 12, 12]

        self.assertEqual(    a-b, amb )
        self.assertNotEqual( a-b, bma )
        self.assertEqual(    b-a, bma )
        self.assertNotEqual( b-a, amb )

    def test_scal(self):
        a = Quat([3,5,7,11])
        a2 = [6,10,14,22]
        self.assertEqual( a*2, a2 )
        self.assertEqual( 2*a, a2 )

    def test_exp(self):
        a = Quat([3,5,7,11])
        e = [6096.087976319468,
             10160.146627199112,
             14224.205278078758,
             -56940.26661544057]
        l = [0.22777632879620358,
             0.37962721466033933,
             0.5314781005244751,
             2.659059996922108]

        self.assertTrue( a.exp().ssd(e) < 1e-3 )
        self.assertTrue( a.ln().ssd(l) < 1e-3 )

    def test_primary(self):
        v = 1
        x = [0.479425538604203, 0.0, 0.0, 0.8775825618903728]
        y = [0.0, 0.479425538604203, 0.0, 0.8775825618903728]
        z = [0.0, 0.0, 0.479425538604203, 0.8775825618903728]
        self.assertTrue( Quat(XAngle(v)).ssd(x) < 1e-3 )
        self.assertTrue( Quat(YAngle(v)).ssd(y) < 1e-3 )
        self.assertTrue( Quat(ZAngle(v)).ssd(z) < 1e-3 )

if __name__ == '__main__':
    unittest.main()
