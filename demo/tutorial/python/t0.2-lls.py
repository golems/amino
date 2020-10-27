#!/usr/bin/env python3

from amino import DVec, DMat


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


def result(A, x, b):
    h2("A")
    print(A)
    h2("b")
    print(b)
    h2("x")
    print(x)


h1("Fully-Determined: A*x = b")
A = DMat.row_matrix([[1, 2], [3, 4]])
b = [5, 6]
x = A.inv() * b
result(A, x, b)

h1("Over-Determined: A*x = b")
A = DMat.row_matrix([[1, 2], [3, 4], [5, 6]])
b = [7, 8, 9]
x = A.pinv() * b
result(A, x, b)

h1("Under-Determined: A*x = b")
A = DMat.row_matrix([[1, 2, 3], [4, 5, 6]])
b = [7, 8]
x = A.pinv() * b
result(A, x, b)
