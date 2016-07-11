#!/bin/sh

COMMON_LISP=$1
top_srcdir=$2
top_builddir=$2

LD_LIBRARY_PATH=.libs:$LD_LIBRARY_PATH exec $COMMON_LISP --script <<EOF
(defvar *top-srcdir* "$2")
(defvar *top-builddir* "$3")
(load "$2/share/make-aarx.lisp")
EOF
