#!/bin/sh -e

wget -q https://beta.quicklisp.org/quicklisp.lisp

sbcl --load quicklisp.lisp \
     --eval '(quicklisp-quickstart:install)' \
     --eval '(let ((ql-util::*do-not-prompt* t)) (ql:add-to-init-file))' \
     --eval '(ql:quickload :cffi)' \
     --eval '(ql:quickload :cffi-grovel)' \
     --eval '(ql:quickload :cxml)' \
     --eval '(ql:quickload :cl-ppcre)' \
     --eval '(ql:quickload :clpython)' \
     --eval '(quit)'
