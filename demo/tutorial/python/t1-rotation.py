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

h1("A Rotation")
ax = XAngle(pi/2)
print ax

h1("Conversions")
Ax = AxAng(ax)
qx = Quat(ax)
Rx = RotMat(ax)
print Ax
print qx
print Rx

h1("Rotate")
p = [1,2,3]
pa = Ax.rotate(p)
pq = qx.rotate(p)
pR = Rx.rotate(p)
print pa
print pq
print pR

h1("Inverse")
qxi = ~qx
Rxi = ~Rx
Axi = ~Ax
print Axi
print qxi
print Rxi

h1("Inv. Rot.")
print Axi.rotate(pa)
print qxi.rotate(pq)
print Rxi.rotate(pR)

h1("Chain")
ay = YAngle(pi/2)
qxy = qx * ay
Rxy = Rx * ay
print qxy
print Rxy

h1("Chained Rotate")
pq = qxy.rotate(p)
pR = Rxy.rotate(p)
print pq
print pR
