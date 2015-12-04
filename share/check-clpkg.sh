#!/bin/sh

COMMON_LISP=$1
top_srcdir=$2
cl_pkg=$3

LD_LIBRARY_PATH=.libs:$LD_LIBRARY_PATH exec > /dev/null 2> /dev/null $COMMON_LISP --script <<EOF
(defvar *top-srcdir* "$2")
(load "$2/share/load-ql.lisp")
(aa-load-system $3)
EOF
