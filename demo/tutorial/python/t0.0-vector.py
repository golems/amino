#!/usr/bin/env python

from amino import DVec

# Create a Vector
x = DVec([1,2,3,4])
print "x:     %s" % x
print "x.len: %d" % len(x)
print "x.inc: %d" % x.inc()

# Common Arithmetic Operators
print "-x:   %s" % (-x)
print "2*x:  %s" % (2*x)
print "x+x:  %s" % (x+x)
print "x+1:  %s" % (x+1)
print "x/2:  %s" % (x/2)

# Slice the vector
s = x[0:4:2]
print "s: %s" % s
print "s.len: %d" % len(s)
print "s.inc: %d" % s.inc()

# Increment the Slice
s+=1
print "s: %s" % s
print "x: %s" % x
