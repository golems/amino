#!/bin/sh

sbcl --script <<EOF


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
   (ql:quickload :robray))
  (t (require :robray)))


(sb-ext:save-lisp-and-die "arxc.core" :executable t)
EOF
echo end
