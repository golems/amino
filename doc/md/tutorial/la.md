Linear Algebra {#tutorial_la}
==============

[TOC]

This tutorial covers vector and matrix representations and concepts,
including vector slices, matrix blocks, and least-squares solutions.

Amino performs linear algebra using the standard,
high-performance-computing [BLAS](http://www.netlib.org/blas/) (Basic
Linear Algebra Subprograms) and
[LAPACK](http://www.netlib.org/lapack/) (Linear Algebra PACKage)
libraries.  Multiple, optimized implementations of BLAS/LAPACK are
available such as [ATLAS](http://math-atlas.sourceforge.net/),
[OpenBLAS](https://www.openblas.net/), and the [Intel Math Kernel
Library (MKL)](https://software.intel.com/en-us/mkl).


Vectors {#tutorial_la_vector}
-------

### Vector Representation

We represent mathematical vectors using a length count, a data
pointer, and an "increment" between successive elements.  For example,
consider the following vector:

\f[
    \mathbf{x} =
    \begin{bmatrix}
    1 \\ 2 \\ 3 \\ 4 \\ 5 \\ 6
    \end{bmatrix}

    \qquad
    \leadsto
    \qquad

    \begin{cases}
    \texttt{len:} & 6 \\
    \texttt{data:} &
    \begin{array}{|c|c|}
    \hline
     1 & 2 & 3 & 4 & 5 & 6\\
    \hline
    \end{array} \\
    \texttt{inc:} & 1 \\
    \end{cases}
\f]

We can "slice" every other element of \f$\mathbf{x}\f$ by using the
same data pointer and an increment of 2.  The length is now 3.

\f[
    \mathbf{x}_{0:6:2} =
    \begin{bmatrix}
    1 \\  3 \\  5
    \end{bmatrix}

    \qquad
    \leadsto
    \qquad

    \begin{cases}
    \texttt{len:} & 3 \\
    \texttt{data:} &
    \begin{array}{|c|c|}
    \hline
     1 & 2 & 3 & 4 & 5 & 6\\
    \hline
    \end{array} \\
    \texttt{inc:} & 2 \\
    \end{cases}

\f]


The combination of length, data pointer, and increment value items
enables flexible access to parts of other vectors and matrices, such
as "slices" of vectors and rows, columns, or diagonals of matrices.
The following C struct defines the vector for doubles:

~~~{.c}
struct aa_dvec {
    size_t len;
    double *data;
    size_t inc;
}
~~~


### Example Code

The following code illustrates the vector representations and slices.

<ol>
<li> Import amino:

~~~{.py}
from amino import DVec
~~~
</li>

<li> Construct and print the vector \f$\mathbf{x} = [1,2,3,4]\f$:

~~~{.py}
x = DVec([1,2,3,4])
print x
print len(x)
print x.inc()
~~~
</li>

<li> Try some common arithmetic operators:

~~~{.py}
print "-x:   %s" % (-x)
print "2*x:  %s" % (2*x)
print "x+x:  %s" % (x+x)
print "x+1:  %s" % (x+1)
print "x/2:  %s" % (x/2)
~~~
</li>

<li> Slice `x` with an increment of 2, giving every other element:

~~~{.py}
s = x[0:4:2]
print "s: %s" % s
print "s.len: %d" % len(s)
print "s.inc: %d" % s.inc()
~~~
</li>

<li> Increment the slice (and underlying elements of `x`):

~~~{.py}
s+=1
print "s: %s" % s
print "x: %s" % x
~~~
</li>


Matrices {#tutorial_la_matrix}
--------

### Matrix Representation

The memory layout for matrices stores elements column-by-column, i.e.,
[column-major
format](https://en.wikipedia.org/wiki/Row-_and_column-major_order).
For example, a 3x3 matrix would be stored in memory as follows:

\f[
    \mathbf{A} =
    \begin{bmatrix}
    x_0 & y_0 & z_0  \\
    x_1 & y_1 & z_1  \\
    x_2 & y_2 & z_2
    \end{bmatrix}

    \qquad
    \leadsto
    \qquad

    \begin{cases}

    \texttt{rows:} & 3 \\
    \texttt{cols:} & 3 \\
    \texttt{data:} &
    \begin{array}{|c|c|}
    \hline
    x_0 & x_1 & x_2 &
    y_0 & y_1 & y_2 &
    z_0 & z_1 & z_2
    \\
    \hline
    \end{array}
    \end{cases}

\f]

Thus, to represent the entire matrix, we need the row count, the
column count, and a pointer to the data.  To represent a
[block](https://en.wikipedia.org/wiki/Block_matrix) or submatrix, we
need one additional element: the "leading dimension," i.e., the number
of rows in the containing matrix.  Consider the block consisting of
the first two rows and columns of \f$\mathbf{A}\f$.  We use the same
data pointer, but the distance between successive columns of this
block is NOT the row count of the block, but the row count of overall
matrix \f$\mathbf{A}\f$:

\f[
    \mathbf{A}_{0:2,0:2} =
    \begin{bmatrix}
    x_0 & y_0 \\
    x_1 & y_1
    \end{bmatrix}

    \qquad
    \leadsto
    \qquad

    \begin{cases}

    \texttt{rows:} & 2 \\
    \texttt{cols:} & 2 \\
    \texttt{data:} &

    \underbrace{
    \begin{array}{|c|c|}
    \hline
     x_0 & \strut x_1 & x_2 \\
    \hline
    \end{array}
    }_{\texttt{ld=3}}
    \!\begin{array}{|c|c|}
    \hline
    y_0 &  \strut y_1 & y_2 \\
    \hline
    \end{array}
    \!\!\!\!\
    \begin{array}{|c|c|}
    \hline
    z_0 & \strut  z_1 & z_2 \\
    \hline
    \end{array}
    \\
    \texttt{ld:} & 3

    \end{cases}
\f]

Thus, our matrix represent consists of the row count, column count,
data pointer, and leading dimension. Note that for an entire matrix,
the leading dimension will equal the row count.

~~~{.c}
struct aa_dmat {
    size_t rows;
    size_t cols;
    double *data;
    size_t ld;
}
~~~

We can use the previously introduced vector format to view rows,
columns, and diagonals of our matrix.

A column vector has length equal to the number of rows, a data pointer
to the first element of the column, and a increment of 1:

\f[

  \texttt{col}(\mathbf{A},0) =
  \begin{bmatrix}
  x_0 \\ x_1 \\ x_2
  \end{bmatrix}

    \qquad
    \leadsto
    \qquad

    \begin{cases}
    \texttt{len:} & 3 \\
    \texttt{data:} &
    \begin{array}{|c|c|}
    \hline
    x_0 & x_1 & x_2
    \\
    \hline
    \end{array} \\
    \texttt{inc:} & 1

    \end{cases}

\f]


\f[

  \texttt{col}(\mathbf{A},1) =
  \begin{bmatrix}
  y_0 \\ y_1 \\ y_2
  \end{bmatrix}

    \qquad
    \leadsto
    \qquad

    \begin{cases}
    \texttt{len:} & 3 \\
    \texttt{data:} &
    \begin{array}{|c|c|}
    \hline
    y_0 & y_1 & y_2
    \\
    \hline
    \end{array} \\
    \texttt{inc:} & 1

    \end{cases}

\f]

A row vector has length equal to the number of columns, a data pointer
to the first element in the row, and an increment equal to the matrix
leading dimension.

\f[

  \texttt{row}(\mathbf{A},0) =
  \begin{bmatrix}
  x_0 & y_0 & z_0
  \end{bmatrix}

    \qquad
    \leadsto
    \qquad

    \begin{cases}
    \texttt{len:} & 3 \\
    \texttt{data:} &
    \begin{array}{|c|c|}
    \hline
    x_0 & x_1 & x_2 &
    y_0 & y_1 & y_2 &
    z_0
    \\
    \hline
    \end{array} \\
    \texttt{inc:} & 3

    \end{cases}

\f]

\f[

  \texttt{row}(\mathbf{A},1) =
  \begin{bmatrix}
  x_1 & y_1 & z_1
  \end{bmatrix}

    \qquad
    \leadsto
    \qquad

    \begin{cases}
    \texttt{len:} & 3 \\
    \texttt{data:} &
    \begin{array}{|c|c|}
    \hline
    x_1 & x_2 &
    y_0 & y_1 & y_2 &
    z_0 & z_1
    \\
    \hline
    \end{array} \\
    \texttt{inc:} & 3

    \end{cases}

\f]

The matrix diagonal vector has length equal to the minimum of rows and
columns, data pointer to the matrix data, and increment equal to one
plus the leading dimension:

\f[

  \texttt{diag}(\mathbf{A}) =
  \begin{bmatrix}
  x_0 \\ y_1 \\ z_2
  \end{bmatrix}

    \qquad
    \leadsto
    \qquad

    \begin{cases}
    \texttt{len:} & 3 \\
    \texttt{data:} &
    \begin{array}{|c|c|}
    \hline
    x_0 & x_1 & x_2 &
    y_0 & y_1 & y_2 &
    z_0 & z_1 & z_2
    \\
    \hline
    \end{array} \\
    \texttt{inc:} & 4

    \end{cases}

\f]

### Example Code

<ol>
<li> Import amino:

~~~{.py}
from amino import DVec, DMat
~~~
</li>

<li> Create a 2x3 matrix:

~~~{.py}
A = DMat.row_matrix([[1,2,3],[4,5,6]])
print A
print "A.rows: %d" % A.rows()
print "A.cols: %d" % A.cols()
print "A.ld:   %d" % A.ld()
for k in range(6):
    print "A._data[%d] = %f" % (k,A._data[k])
~~~
</li>

<li> Create a 3x2 matrix:

~~~{.py}
B = DMat.col_matrix([[1,2,3],[4,5,6]])
print B
print "B.rows: %d" % B.rows()
print "B.cols: %d" % B.cols()
print "B.ld:   %d" % B.ld()
for k in range(6):
    print "B._data[%d] = %f" % (k,A._data[k])

~~~
</li>

<li> Try some arithmetic functions:

~~~{.py}
print A*2
print A/2
print A+A
print A+1
print A-1
print A.t()
print A*[1,2,3]
print A*B
~~~
</li>

<li> Construct some vector views of the matrix:

~~~{.py}
print A
print A.row_vec(0)
print A.col_vec(1)

d = A.diag_vec()
print d
d += 10
print d

print A
~~~
</li>
</ol>



Linear Least Squares {#tutorial_la_lls}
--------------------

This sections discusses how to solve systems of linear equations using
the matrix inverse and pseudoinverse.

(An informed reader may note that explicitly computing the
(pseudo)inverse is not the most efficient method for solving linear
equations. However, obtaining an explicit pseudoinverse provides
useful capabilities when we are solving for a robot's motion, so we
will present the explicit computation here.)

### Systems of Linear Equations

We can represent and efficiently solve systems of linear equations in
terms of matrices.  Consider the example below with two linear
equations and two unknowns.  We represent the linear coefficients of
each unknown as a matrix.

\f[
    \begin{array}
    1 x_0 & + &  2 x_1 & = 5 \\
    3 x_0 & + & 4 x_1 & = 6
    \end{array}

    \quad\quad
    \leadsto
    \quad\quad

    \begin{bmatrix}
    1 & 2 \\
    3 & 4
    \end{bmatrix}
    \begin{bmatrix}
    x_0 \\ x_1
    \end{bmatrix}
    =
    \begin{bmatrix}
    5 \\ 6
    \end{bmatrix}
\f]

Then, we can solve the system of equations by multiplying each side by
the inverse of the coefficient matrix.

\f[
    \begin{bmatrix}
    x_0 \\ x_1
    \end{bmatrix}
    =
    \left(
    \begin{bmatrix}
    1 & 2 \\
    3 & 4
    \end{bmatrix}^{-1}
    \right)

    \begin{bmatrix}
    5 \\ 6
    \end{bmatrix}

    =
    \begin{bmatrix}
        -2 & 1 \\
        1.5 & -.5
    \end{bmatrix}
    \begin{bmatrix}
    5 \\ 6
    \end{bmatrix}

    =
    \begin{bmatrix}
    4 \\ 4.5
    \end{bmatrix}
\f]





### Over and under-determined Systems

If we have more unknowns than equations, there may be infinitely many
solutions to the system.  Similarly, more equations than unknowns may
result in a system with no solutions.  We can still construct a
coefficient matrix, but it will now be nonsquare:


In the underdetermined case, we have a "fat" matrix (more rows than
columns):
\f[
    \begin{array}
    1 x_0 & + &  2 x_1 &  3 x_2 & = & 7 \\
    4 x_0 & + & 5 x_1 &  6 x_2 & = & 8
    \end{array}

    \quad\quad
    \leadsto
    \quad\quad

    \begin{bmatrix}
    1 & 2 & 3\\
    4 & 5 & 6
    \end{bmatrix}
    \begin{bmatrix}
    x_0 \\ x_1 \\ x_2
    \end{bmatrix}
    =
    \begin{bmatrix}
    7 \\ 8
    \end{bmatrix}
\f]

In the overdetermined case, we have a "thin" matrix (more columns than
rows:
\f[
    \begin{array}
    1 x_0 & + & 2 x_1  \\
    3 x_0 & + & 4 x_1  \\
    5 x_0 & + & 6 x_1
    \end{array}

    \quad\quad
    \leadsto
    \quad\quad

    \begin{bmatrix}
    1 & 2 \\
    3 & 4 \\
    5 & 6
    \end{bmatrix}
    \begin{bmatrix}
    y_0 \\ y_1
    \end{bmatrix}
    =
    \begin{bmatrix}
    7 \\ 8 \\ 9
    \end{bmatrix}
\f]


Since we have a nonsquare matrix, we cannot compute it's inverse.
However, we can use the
[pseudoinverse](https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse).

\f[
    \begin{bmatrix}
    x_0 \\ x_1 \\ x_2
    \end{bmatrix}
    =
    \left(
    \begin{bmatrix}
    1 & 2 & 3 \\
    4 & 5 & 6
    \end{bmatrix}^{+}
    \right)

    \begin{bmatrix}
    7 \\ 8
    \end{bmatrix}

    =
    \begin{bmatrix}
        -9.44 & 0.444 \\
        -0.111 & 0.111 \\
        0.722 & -0.222
    \end{bmatrix}
    \begin{bmatrix}
    7 \\ 8
    \end{bmatrix}

    =
    \begin{bmatrix}
    -3.06 \\ 0.111 \\ 3.28
    \end{bmatrix}
\f]

\f[
    \begin{bmatrix}
    y_0 \\ y_1
    \end{bmatrix}
    \approx
    \left(
    \begin{bmatrix}
    1 & 2 \\
    3 & 4 \\
    5 & 6
    \end{bmatrix}^{+}
    \right)

    \begin{bmatrix}
    7 \\ 8 \\ 9
    \end{bmatrix}
    =
    \begin{bmatrix}
        -1.33 & -0.333 & 0.667 \\
        1.08 & 0.333 & -0.417
    \end{bmatrix}
    \begin{bmatrix}
    7 \\ 8 \\ 9
    \end{bmatrix}

    =
    \begin{bmatrix}
    -6 \\ -6.5
    \end{bmatrix}
\f]

The pseudoinverse finds the **least squares** solution.

In the underdetermined case, we are minimizing the sum of squares
(i.e., the Euclidean norm) of the vector \f$\mathbf{x}\f$:

* **Given:** Underdetermined system \f$\mathbf{A}\mathbf{x} =
  \mathbf{b}\f$, i.e., \f$\mathbf{A}\f$ has more columns (unknowns)
  than rows (equations).
* **Find:** Vector \f$\mathbf{x}\f$, such that:
  * *minimize:*  \f$\Vert\mathbf{x}\Vert\f$
  * *subject to:*  \f$\mathbf{A} \mathbf{x} = \mathbf{b}\f$
* **Solution:** \f$ \mathbf{x} = \mathbf{A}^+ \mathbf{b} \f$


In the overdetermined case, we are minimizing the sum of squares
of the *error*:

* **Given:** Overdetermined system \f$\mathbf{A}\mathbf{x} =
  \mathbf{b}\f$, , i.e., \f$\mathbf{A}\f$ has more rows (equations)
  than columns (unknowns).
* **Find:** Vector \f$\mathbf{z}\f$, such that:
  * *minimize:*  \f$\Vert\mathbf{A} \mathbf{z} - \mathbf{b}\Vert\f$
* **Solution:** \f$ \mathbf{x} = \mathbf{A}^+ \mathbf{b} \f$

### Example Code

<ol>
<li> Create a matrix and vector for a system of equations,
   \f$\mathbf{A}\mathbf{x} = \mathbf{b}\f$, with two equations and two
   unknowns:


~~~{.py}
A = DMat.row_matrix([[1,2],[3,4]])
b = [5,6]
~~~
</li>

<li> Invert the (square) matrix to solve the system:

~~~{.py}
x = A.inv() * b
print x
~~~
</li>

<li> Create a matrix and vector for an *over-determined* system:

~~~{.py}
A = DMat.row_matrix([[1,2],[3,4],[5,6]])
b = [7,8,9]
~~~
</li>

<li> Pseudo-invert the (non-square) matrix to find the least-squares
     solution:

~~~{.py}
x = A.pinv()*b
print x
~~~
</li>

<li> Create a matrix and vector for an *under-determined* system:

~~~{.py}
A = DMat.row_matrix([[1,2,3],[4,5,6]])
b = [7,8]
~~~
</li>

<li> Pseudo-invert the (non-square) matrix to find the least-squares
    solution:

~~~{.py}
x = A.pinv()*b
print x
~~~
</li>
</ol>
