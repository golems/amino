Tutorial {#tutorial}
========

[TOC]


Vector and Matrix Programming {#tutorial_matrix}
=============================


Vectors {#tutorial_matrix_vector}
-------

We represent mathematical vectors using a length count, a data
pointer, and an "increment" between successive elements.  These three
items enable flexible access to parts of other vectors and matrices,
such as "slices" of vectors and rows, columns, or diagonals of
matrices.  The following C struct defines the vector for doubles:

    struct aa_dvec {
        size_t len;
        double *data;
        size_t inc;
    }


1. Import amino:

        from amino import DVec

2. Construct and print the vector \f$\mathbf{x} = [1,2,3,4]\f$:

        x = DVec([1,2,3,4])
        print x
        print len(x)
        print x.inc()

3. Try some common arithmetic operators:

        print "-x:   %s" % (-x)
        print "2*x:  %s" % (2*x)
        print "x+x:  %s" % (x+x)
        print "x+1:  %s" % (x+1)
        print "x/2:  %s" % (x/2)

4. Slice `x` with an increment of 2, giving every other element:

        s = x[0:4:2]
        print "s: %s" % s
        print "s.len: %d" % len(s)
        print "s.inc: %d" % s.inc()

5. Increment the slice (and underlying elements of `x`):

        s+=1
        print "s: %s" % s
        print "x: %s" % x


Matrices {#tutorial_matrix_matrix}
--------
