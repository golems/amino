#!/bin/sh -e

wget https://beta.quicklisp.org/quicklisp.lisp

sbcl --load quicklisp.lisp \
     --eval '(quicklisp-quickstart:install)' \
     --eval '(ql:quickload :cffi)' \
     --eval '(ql:quickload :cffi-grovel)' \
     --eval '(ql:quickload :cxml)' \
     --eval '(ql:quickload :cl-ppcre)' \
     --eval '(ql:quickload :clpython)' \
     --eval '(quit)'
