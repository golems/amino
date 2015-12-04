;; Try to load quicklisp and/or ASDF
(unless (find-package :quicklisp)
  (let ((ql (find-if #'probe-file
                     (map 'list (lambda (setup) (merge-pathnames setup (user-homedir-pathname)))
                          '("quicklisp/setup.lisp" ".quicklisp/setup.lisp" "Quicklisp/setup.lisp")))))
    (cond
      (ql (load ql))
      ((not (find-package :asdf))
       (require :asdf)))))

;; Define System loading function
(defun aa-load-system (system)
  (let ((ql-package (find-package :quicklisp))
        (asdf-package (find-package :asdf)))
    (cond
      (ql-package
       (funcall (intern "QUICKLOAD" ql-package)
                system))
      (asdf-package
       (funcall (intern "LOAD-SYSTEM" asdf-package)
                system))
      (t
       (require system))))
  (assert (find-package system)))
