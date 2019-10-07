#!/usr/bin/env python

from amino import Vec3, XAngle, YAngle, ZAngle, AxAng, EulerRPY, Quat, RotMat
from math import pi

def h1(name):
    """Print a level 1 heading"""
    print ""
    print "{:^16}".format(name)
    print "{:=^16}".format('')

def h2(name):
    """Print a level 2 heading"""
    print ""
    print "{:^16}".format(name)
    print "{:-^16}".format('')


def check_equiv(a,b):
    tol = 1e-6
    rel = (a * ~b)
    assert rel.ln().nrm2() < tol

def check_equiv_vec(a,b):
    tol = 1e-6
    assert a.ssd(b) < tol


h1("A Rotation")
ax = XAngle(pi/2)
print ax

h1("Conversions")
Ax = AxAng(ax)
qx = Quat(ax)
Rx = RotMat(ax)

check_equiv(qx, Quat(Ax))
check_equiv(qx, Quat(Rx))
check_equiv(Rx, RotMat(qx))
check_equiv(Rx, RotMat(Ax))

print Ax
print qx
print Rx

h1("Rotate")
p = Vec3([1,2,3])
pa = Ax.rotate(p)
pq = qx.rotate(p)
pR = Rx.rotate(p)

check_equiv_vec(pa, pq)
check_equiv_vec(pR, pq)

print pa
print pq
print pR


h1("Inverse")
qxi = ~qx
Rxi = ~Rx
Axi = ~Ax

check_equiv( qxi, Quat(Rxi) )
check_equiv( qxi, Quat(Axi) )
check_equiv( Rxi, RotMat(Axi) )
check_equiv( Rxi, RotMat(qxi) )

print Axi
print qxi
print Rxi

h1("Inv. Rot.")

pai = Axi.rotate(pa)
pqi = Axi.rotate(pq)
pRi = Axi.rotate(pR)

check_equiv_vec(p, pai)
check_equiv_vec(p, pqi)
check_equiv_vec(p, pRi)

print Axi.rotate(pa)
print qxi.rotate(pq)
print Rxi.rotate(pR)

h1("Chain")
ay = YAngle(pi/2)
qxy = qx * ay
Rxy = Rx * ay
print qxy
print Rxy

check_equiv( qxy, Quat(Rxy) )
check_equiv( Rxy, RotMat(qxy) )

h1("Chained Rotate")
pq = qxy.rotate(p)
pR = Rxy.rotate(p)
print pq
print pR

check_equiv_vec(pq, pR)
