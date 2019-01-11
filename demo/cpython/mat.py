#!/usr/bin/env python

from amino import *

# v = DVec.create(2)
# v.copy_from([1,2])

# print v

print "Row"
A = DMat.row_matrix( [1,2],
                     [3,4] )

print A.col_vec(0)
print A.col_vec(1)
print A[0,0]
print A[1,0]
print A[0,1]
print A[1,1]

print A

print "Col"
A = DMat.col_matrix( [1,2],
                     [3,4] )
print A.col_vec(0)
print A.col_vec(1)
print A[0,0]
print A[1,0]
print A[0,1]
print A[1,1]

print A

# A = DMat( (2,2) )
# A[0,0] = 1
# A[1,0] = 2
# A[0,1] = 3
# A[1,1] = 4

# x = DVec( [1,2] )

# y = A*x


# print y


# print "c0: " + str(m.col_vec(0))
# print "c1: " + str(m.col_vec(1))

# print "r0: " + str(m.row_vec(0))
# print "r1: " + str(m.row_vec(1))

print "end"
