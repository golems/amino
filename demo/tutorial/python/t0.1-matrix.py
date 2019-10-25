#!/usr/bin/env python

from amino import DVec, DMat


# Helpers to print(headings)
def h1(name):
    """Print a level 1 heading"""
    print("")
    print("{:^16}".format(name))
    print("{:=^16}".format(''))


def h2(name):
    """Print a level 2 heading"""
    print("")
    print("{:^16}".format(name))
    print("{:-^16}".format(''))


# Create a 2x3 matrix
h1("A")
A = DMat.row_matrix([[1, 2, 3], [4, 5, 6]])
print(A)
print("A.rows: %d" % A.rows)
print("A.cols: %d" % A.cols)
print("A.ld:   %d" % A.ld)
for k in range(6):
    print("A._data[%d] = %f" % (k, A._data[k]))

# Create a 3x2 matrix
h1("B")
B = DMat.col_matrix([[1, 2, 3], [4, 5, 6]])
print(B)
print("B.rows: %d" % B.rows)
print("B.cols: %d" % B.cols)
print("B.ld:   %d" % B.ld)
for k in range(6):
    print("B._data[%d] = %f" % (k, A._data[k]))

# Example matrix arithmetic
h1("Arithmetic")

h2("A*2")
print(A * 2)

h2("A/2")
print(A / 2)

h2("A+A")
print(A + A)

h2("A+1")
print(A + 1)

h2("A-1")
print(A - 1)

h2("A^T")
print(A.transpose())

h2("A*[1,2,3]^T")
print(A * [1, 2, 3])

h2("A*B")
print(A * B)

# Vector views of matrix
h1("Matrix Vector Views")

h2("A")
print(A)

h2("A.row_vec(0)")
print(A.row_vec(0))

h2("A.col_vec(1)")
print(A.col_vec(1))

h2("A.diag_vec()")
d = A.diag_vec()
print(d)

h2("A.diag_vec() += 10")
d += 10
print(d)

h2("A")
print(A)

# Block views of matrix
h1("Matrix Block Views")

h2("A")
print(A)

h2("As = A[0:2,0:2]")
As = A[0:2, 0:2]
print(As)
print("As.rows: %d" % As.rows)
print("As.cols: %d" % As.cols)
print("As.ld:   %d" % As.ld)

h2("As *= 2")
As *= 2
print(As)

h2("A")
print(A)

h2("B")
print(B)

h2("Bs = B[0:2,0:2]")
Bs = B[0:2, 0:2]
print(Bs)
print("Bs.rows: %d" % Bs.rows)
print("Bs.cols: %d" % Bs.cols)
print("Bs.ld:   %d" % Bs.ld)

h2("Bs *= 2")
Bs *= 2
print(Bs)

h2("B")
print(B)
