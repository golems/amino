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

from math import sqrt, pi


class TestVec3(unittest.TestCase):
    def test_create(self):
        v = Vec3([3, 5, 7])

        self.assertEqual(len(v), 3)

        self.assertEqual(v[0], 3)
        self.assertEqual(v[1], 5)
        self.assertEqual(v[2], 7)

        self.assertRaises(IndexError, lambda: v[3])

    def test_set(self):
        v = Vec3()
        v[0] = 3
        v[1] = 6
        v[2] = 9

        self.assertEqual(v[0], 3)
        self.assertEqual(v[1], 6)
        self.assertEqual(v[2], 9)

        def helper():
            v[3] = 10

        self.assertRaises(IndexError, helper)

    def test_nrm(self):
        v = Vec3([1, 2, 3])
        self.assertTrue(abs(v.nrm2() - sqrt(1 + 4 + 9)) < 1e-6)

    def test_eq(self):
        a = Vec3([1, 2, 3])
        b = Vec3([1, 2, 3])
        c = Vec3([2, 2, 3])
        self.assertTrue(a == b)
        self.assertFalse(a == c)
        self.assertTrue(a != c)
        self.assertFalse(a != b)

        self.assertEqual(a, a)
        self.assertEqual(a, b)
        self.assertNotEqual(a, c)

        self.assertTrue(a == [1, 2, 3])
        self.assertFalse(a == [1, 2, 5])
        self.assertTrue([1, 2, 3] == a)
        self.assertFalse([1, 2, 4] == a)

        self.assertFalse(a != [1, 2, 3])
        self.assertTrue(a != [1, 2, 5])
        self.assertFalse([1, 2, 3] != a)
        self.assertTrue([1, 2, 4] != a)

    def test_dot_cross(self):
        v = Vec3([3, 5, 7])
        l = [17, 19, 23]
        self.assertEqual(v.dot(l), 307)
        self.assertEqual(v.cross(l), [-18, 50, -28])
        self.assertTrue(abs(v.ssd(l) - 25.456) < 1e-2)

    def test_scal_pm(self):
        x = Vec3([3, 5, 7])
        a = 2
        xpa = [5, 7, 9]
        xma = [1, 3, 5]
        amx = [-1, -3, -5]

        self.assertEqual(x + a, xpa)
        self.assertEqual(a + x, xpa)

        self.assertEqual(x - a, xma)
        self.assertEqual(a - x, amx)

    def test_vec_pm(self):
        x = [3, 5, 7]
        y = [1, 2, 3]
        xpy = [4, 7, 10]
        xmy = [2, 3, 4]

        self.assertEqual(Vec3(x) + Vec3(y), xpy)
        self.assertEqual(Vec3(x) + y, xpy)
        self.assertEqual(x + Vec3(y), xpy)

        self.assertEqual(Vec3(x) - Vec3(y), xmy)
        self.assertEqual(Vec3(x) - y, xmy)
        self.assertEqual(x - Vec3(y), xmy)

    def test_vec_mul(self):
        x = Vec3([2, 4, 8])
        a = 2
        x_mul_a = [4, 8, 16]
        x_div_a = [1, 2, 4]

        self.assertEqual(a * x, x_mul_a)
        self.assertEqual(x * a, x_mul_a)

        self.assertEqual(x / a, x_div_a)


class TestQuat(unittest.TestCase):
    def test_create(self):
        h = Quat([1, 2, 3, 4])

        self.assertEqual(len(h), 4)

        self.assertEqual(h[0], 1)
        self.assertEqual(h[1], 2)
        self.assertEqual(h[2], 3)
        self.assertEqual(h[3], 4)

        self.assertRaises(IndexError, lambda: h[4])

    def test_norm(self):
        h = Quat([1, 2, 3, 4])
        self.assertTrue(abs(h.nrm2() - 5.4772) < 1e-3)

    def test_eq(self):
        a = Quat([1, 2, 3, 4])
        l = [1, 2, 3, 4]
        p = [1, 2, 3, 5]
        self.assertTrue(a == l)
        self.assertFalse(a != l)
        self.assertTrue(a != p)
        self.assertFalse(a == p)

        self.assertTrue(l == a)
        self.assertFalse(l != a)
        self.assertTrue(p != a)
        self.assertFalse(p == a)

    def test_mul(self):
        a = Quat([3, 5, 7, 11])
        b = [13, 17, 19, 23]
        ab = [188, 336, 356, -4]
        ba = [236, 268, 384, -4]

        self.assertEqual(a * b, ab)
        self.assertNotEqual(a * b, ba)
        self.assertEqual(b * a, ba)
        self.assertNotEqual(b * a, ab)

    def test_add(self):
        a = Quat([3, 5, 7, 11])
        b = [13, 17, 19, 23]
        apb = [16, 22, 26, 34]
        x = [16, 22, 26, 35]

        self.assertEqual(a + b, apb)
        self.assertNotEqual(a + b, x)
        self.assertEqual(b + a, apb)
        self.assertNotEqual(b + a, x)

    def test_iadd(self):
        ao = [3, 5, 7, 11]
        b = [13, 17, 19, 23]
        apb = [16, 22, 26, 34]
        x = [16, 22, 26, 35]

        a = Quat(ao)
        a += b
        self.assertEqual(a, apb)
        self.assertNotEqual(a, x)

    def test_sub(self):
        a = Quat([3, 5, 7, 11])
        b = [13, 17, 19, 23]
        amb = [-10, -12, -12, -12]
        bma = [10, 12, 12, 12]

        self.assertEqual(a - b, amb)
        self.assertNotEqual(a - b, bma)
        self.assertEqual(b - a, bma)
        self.assertNotEqual(b - a, amb)

    def test_isub(self):
        ao = [3, 5, 7, 11]
        b = [13, 17, 19, 23]
        amb = [-10, -12, -12, -12]
        bma = [10, 12, 12, 12]

        a = Quat(ao)
        a -= b
        self.assertEqual(a, amb)
        self.assertNotEqual(a, bma)

    def test_scal(self):
        a = Quat([3, 5, 7, 11])
        a2 = [6, 10, 14, 22]
        self.assertEqual(a * 2, a2)
        self.assertEqual(2 * a, a2)

    def test_iscal(self):
        ao = [3, 5, 7, 11]
        a2 = [6, 10, 14, 22]
        a = Quat(ao)
        a *= 2
        self.assertEqual(a, a2)
        self.assertNotEqual(a, ao)

    def test_inv(self):
        a = Quat([3, 5, 7, 11])
        b = ~a
        be = Quat([
            -0.014705882352941176, -0.024509803921568627, -0.03431372549019608,
            0.05392156862745098
        ])
        self.assertTrue(b.isclose(be))
        self.assertTrue(a.isclose(~b))

        hi = Quat.identity()
        self.assertTrue((a * b).isclose(hi))
        self.assertTrue((b * a).isclose(hi))

    def test_exp(self):
        a = Quat([3, 5, 7, 11])
        e = Quat([
            6096.087976319468, 10160.146627199112, 14224.205278078758,
            -56940.26661544057
        ])
        l = Quat([
            0.22777632879620358, 0.37962721466033933, 0.5314781005244751,
            2.659059996922108
        ])

        self.assertTrue(a.exp().isclose(e))
        self.assertTrue(a.ln().isclose(l))

    def test_primary(self):
        v = 1
        x = Quat([0.479425538604203, 0.0, 0.0, 0.8775825618903728])
        y = Quat([0.0, 0.479425538604203, 0.0, 0.8775825618903728])
        z = Quat([0.0, 0.0, 0.479425538604203, 0.8775825618903728])
        self.assertTrue(Quat(XAngle(v)).isclose(x))
        self.assertTrue(Quat(YAngle(v)).isclose(y))
        self.assertTrue(Quat(ZAngle(v)).isclose(z))

    def test_euler(self):
        e = EulerRPY([1, 2, 3])
        self.assertTrue(1 == e.r)
        self.assertTrue(2 == e.p)
        self.assertTrue(3 == e.y)

        h = Quat(e)
        he = Quat([
            -0.7182870182434113, 0.31062245106570396, 0.44443511344300074,
            0.4359528440735657
        ])

        self.assertTrue(h.isclose(he))


class TestRotMat(unittest.TestCase):
    def cmprot(self, R, h, v):
        Rv = R.rotate(v)
        hv = h.rotate(v)
        self.assertTrue(Rv.isclose(hv))

        Rvi = (~R).rotate(Rv)
        hvi = (~h).rotate(hv)

        self.assertTrue(Rvi.isclose(v))
        self.assertTrue(hvi.isclose(v))
        self.assertTrue(hvi.isclose(Rvi))

        self.assertTrue(RotMat(R).rotate(v).isclose(Rv))
        self.assertTrue(RotMat(h).rotate(v).isclose(Rv))

        self.assertTrue(Quat(R).rotate(v).isclose(Rv))
        self.assertTrue(Quat(h).rotate(v).isclose(Rv))

    def isclose_rq(self, thing):
        self.assertTrue(RotMat(thing), Quat(thing))

    def cmprq(self, thing, v):
        self.cmprot(RotMat(thing), Quat(thing), v)
        self.isclose_rq(thing)

    def cmp2(self, thing1, thing2, v):
        r1 = RotMat(thing1)
        h1 = Quat(thing1)
        r2 = RotMat(thing2)
        h2 = Quat(thing2)

        self.cmprot(r1 * r2, h1 * h2, v)

        R = r1 * RotMat(h2)
        h = h1 * Quat(r2)
        self.cmprot(R, h, v)

    def test_princ(self):
        v = Vec3([1, 2, 3])
        self.cmprq(XAngle(1), v)
        self.cmprq(YAngle(1), v)
        self.cmprq(ZAngle(1), v)

    def test_euler(self):
        e = EulerZYX([pi / 2, 0, 0])
        Re = RotMat(e)
        R = RotMat.col_matrix([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        self.assertTrue(Re.isclose(R))

    def test_princ2(self):
        v = Vec3([1, 2, 3])
        self.cmp2(XAngle(1), YAngle(2), v)
        self.cmp2(ZAngle(1), XAngle(2), v)


class TestAxAng(unittest.TestCase):
    def test_princ(self):
        a = 0.42
        ax = AxAng(XAngle(a))
        ay = AxAng(YAngle(a))
        az = AxAng(ZAngle(a))

        qx = Quat(XAngle(a))
        qy = Quat(YAngle(a))
        qz = Quat(ZAngle(a))

        self.assertTrue(qx.isclose(Quat(ax)))
        self.assertTrue(qy.isclose(Quat(ay)))
        self.assertTrue(qz.isclose(Quat(az)))

    def test_conv(self):
        aa = AxAng([1, 2, 3, 4])
        q = Quat(aa)
        R = RotMat(aa)
        qR = Quat(R)
        q.minimize()
        qR.minimize()
        self.assertTrue(q.isclose(qR))


class TestTf(unittest.TestCase):
    def cmptfs(self, T, S, E, p):
        p = Vec3.ensure(p)
        q = T.transform(p)

        self.assertTrue(q.isclose(S.transform(p)))
        self.assertTrue(q.isclose(E.transform(p)))

        self.assertTrue((~T).transform(q).isclose(p))
        self.assertTrue((~E).transform(q).isclose(p))
        self.assertTrue(E.conj().transform(q).isclose(p))
        self.assertTrue(S.conj().transform(q).isclose(p))

        self.assertTrue(TfMat(T).transform(p).isclose(q))
        self.assertTrue(TfMat(S).transform(p).isclose(q))
        self.assertTrue(TfMat(E).transform(p).isclose(q))

        self.assertTrue(DualQuat(T).transform(p).isclose(q))
        self.assertTrue(DualQuat(S).transform(p).isclose(q))
        self.assertTrue(DualQuat(E).transform(p).isclose(q))

        self.assertTrue(QuatTrans(T).transform(p).isclose(q))
        self.assertTrue(QuatTrans(S).transform(p).isclose(q))
        self.assertTrue(QuatTrans(E).transform(p).isclose(q))

    def cmptf(self, rot, trans, p):
        self.cmptfs(
            TfMat((rot, trans)), DualQuat((rot, trans)), QuatTrans((rot,
                                                                    trans)), p)

    def cmpmul(self, tf1, tf2, p):
        self.cmptfs(
            TfMat(tf1) * TfMat(tf2),
            DualQuat(tf1) * DualQuat(tf2),
            QuatTrans(tf1) * QuatTrans(tf2), p)

    def test_principal(self):
        self.cmptf(XAngle(.25), [1, 2, 3], [3, 7, 11])
        self.cmptf(YAngle(.5), [1, 2, 3], [3, 7, 11])
        self.cmptf(ZAngle(1), [1, 2, 3], [3, 7, 11])

    def test_mul(self):
        self.cmpmul((XAngle(.25), [1, 2, 3]), (YAngle(.5), [1, 2, 3]),
                    [3, 7, 11])

    def cmp_prop(self, rot, trans):
        T = TfMat()
        S = DualQuat()
        E = QuatTrans()

        T.rotation = rot
        T.translation = trans

        E.rotation = rot
        E.translation = trans

        S.translation = trans
        S.rotation = rot

        self.cmptfs(T, S, E, [1, 2, 3])

    def test_prop(self):
        self.cmp_prop(XAngle(pi / 2), (1, 2, 3))
        self.cmp_prop(YAngle(pi / 2), (1, 2, 3))
        self.cmp_prop(ZAngle(pi / 2), (1, 2, 3))


if __name__ == '__main__':
    unittest.main()
