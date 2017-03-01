#!/bin/sh

COMMON_LISP=$1
export top_srcdir=$2
export top_builddir=$3

LD_LIBRARY_PATH="`pwd`/.libs:$LD_LIBRARY_PATH" exec $COMMON_LISP --script <<EOF
(defvar *top-srcdir* "$top_srcdir")
(defvar *top-builddir* "$top_builddir")
(load "$top_srcdir/share/make-aarx.lisp")
EOF
