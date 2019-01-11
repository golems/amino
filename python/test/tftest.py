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

if __name__ == '__main__':
    unittest.main()
