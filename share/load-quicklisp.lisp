;; Try to load quicklisp and/or ASDF
(unless (find-package :quicklisp)
  (let ((ql (find-if #'probe-file
                     (append (map 'list (lambda (setup) (merge-pathnames setup (user-homedir-pathname)))
                                  '("quicklisp/setup.lisp" ".quicklisp/setup.lisp" "Quicklisp/setup.lisp"))
                             '("/usr/local/quicklisp/setup.lisp"
                               "/usr/local/src/quicklisp/setup.lisp"
                               "/usr/quicklisp/setup.lisp"
                               "/usr/src/quicklisp/setup.lisp"
                               "/opt/quicklisp/setup.lisp")))))
    (when ql
      (load ql))))


(require :asdf)

;; Include sycamore
(push (make-pathname :directory `(:relative ,(uiop/os:getenv "top_srcdir") "submodules" "sycamore" "src"))
      asdf:*central-registry*)


;; Define System loading function
(defun aa-load-system (system &optional (system-package system))
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
  (assert (find-package system-package)))
