#!/usr/bin/env python

from amino import DVec, DMat

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


# Create a 2x3 matrix
A = DMat.row_matrix([[1,2,3],[4,5,6]])
h1("A")
print A
print "A.rows: %d" % A.rows()
print "A.cols: %d" % A.cols()
print "A.ld:   %d" % A.ld()
for k in range(6):
    print "A._data[%d] = %f" % (k,A._data[k])

# Create a 3x2 matrix
h1("B")
B = DMat.col_matrix([[1,2,3],[4,5,6]])
print B
print "B.rows: %d" % B.rows()
print "B.cols: %d" % B.cols()
print "B.ld:   %d" % B.ld()
for k in range(6):
    print "B._data[%d] = %f" % (k,A._data[k])


# Example matrix arithmetic
h1("Arithmetic")

h2("A*2")
print A*2

h2("A/2")
print A/2

h2("A+A")
print A+A

h2("A+1")
print A+1

h2("A-1")
print A-1

h2("A^T")
print A.t()

h2("A*[1,2,3]^T")
print A*[1,2,3]

h2("A*B")
print A*B
