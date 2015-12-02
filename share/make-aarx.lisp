(unless (find-package :quicklisp)
  (let ((ql (find-if #'probe-file
                     (map 'list (lambda (setup) (merge-pathnames setup (user-homedir-pathname)))
                          '("quicklisp/setup.lisp" ".quicklisp/setup.lisp" "Quicklisp/setup.lisp")))))
    (cond
      (ql (load ql))
      ((not (find-package :asdf))
       (require :asdf)))))


(let ((ql-package (find-package :quicklisp)))
  (cond
    (ql-package
     (funcall (intern "QUICKLOAD" ql-package)
              :amino))
    (t
      (require :amino))))

(sb-ext:save-lisp-and-die "aarx.core" :executable t)
