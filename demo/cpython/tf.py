#!/usr/bin/env python

from amino import *


print Quat(XAngle(3.14)) * Quat(YAngle(3.14))

# print Quat((1,2,3,4))

# print Quat(RotMat(YAngle(3.14)))
# print Quat(RotMat(RotMat(YAngle(3.14))))

# print TfMat( (XAngle(3.14), (0,0,1)) ).translation()

# print aa.RotMat(aa.YAngle(3.14)).cx
# print aa.RotMat(aa.YAngle(3.14)).cy
# print aa.RotMat(aa.YAngle(3.14)).cz

# print aa.RotMat(1).cx
# print aa.RotMat(1).cy
# print aa.RotMat(1).cz


print "end"
