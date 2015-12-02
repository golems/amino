;; Try to load quicklisp and/or ASDF
(unless (find-package :quicklisp)
  (let ((ql (find-if #'probe-file
                     (map 'list (lambda (setup) (merge-pathnames setup (user-homedir-pathname)))
                          '("quicklisp/setup.lisp" ".quicklisp/setup.lisp" "Quicklisp/setup.lisp")))))
    (cond
      (ql (load ql))
      ((not (find-package :asdf))
       (require :asdf)))))

;; Try to register lisp directory with ASDF
(when (find-package :asdf)
  (eval `(push (make-pathname :directory (append (pathname-directory (truename *top-srcdir*))
                                                 '("lisp")))
               ,(intern "*CENTRAL-REGISTRY*" :asdf))))

;; Try to load Amino
(let ((ql-package (find-package :quicklisp)))
  (cond
    (ql-package
     (funcall (intern "QUICKLOAD" ql-package)
              :amino))
    (t
      (require :amino))))

;; Make core file
(sb-ext:save-lisp-and-die "aarx.core" :executable t)
