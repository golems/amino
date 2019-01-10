#!/usr/bin/env python

from amino import *


A = DMat( (2,2) )
A[0,0] = 1
A[1,0] = 2
A[0,1] = 3
A[1,1] = 4

x = DVec( [1,2] )

y = A*x


print y

# print m[0,0]
# print m[1,0]
# print m[0,1]
# print m[1,1]

# print "c0: " + str(m.col_vec(0))
# print "c1: " + str(m.col_vec(1))

# print "r0: " + str(m.row_vec(0))
# print "r1: " + str(m.row_vec(1))

print "end"
