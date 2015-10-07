#!/bin/sh

COMMON_LISP=$1

LD_LIBRARY_PATH=.libs:$LD_LIBRARY_PATH $COMMON_LISP --script <<EOF


(unless (find-package :quicklisp)
  (let ((ql (find-if #'probe-file
                     (map 'list (lambda (setup) (merge-pathnames setup (user-homedir-pathname)))
                          '("quicklisp/setup.lisp" ".quicklisp/setup.lisp" "Quicklisp/setup.lisp")))))
    (cond
      (ql (load ql))
      ((not (find-package :asdf))
       (require :asdf)))))


(cond
  ((find-package :quicklisp)
   (ql:quickload :amino))
  (t (require :amino)))


(sb-ext:save-lisp-and-die "aarx.core" :executable t)
EOF
echo end
