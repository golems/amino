(load (make-pathname :directory (append (pathname-directory (truename (uiop/os:getenv "top_srcdir")))
                                        '("share"))
                     :name "load-quicklisp"
                     :type "lisp"))


;; Try to register lisp directory with ASDF
(when (find-package :asdf)
  (eval `(push (make-pathname :directory (append (pathname-directory  (truename (uiop/os:getenv "top_srcdir")))
                                                 '("lisp")))
               ,(intern "*CENTRAL-REGISTRY*" :asdf))))

;; Try to load Amino
(aa-load-system :amino)
(aa-load-system :amino-rx :amino)
(aa-load-system :amino-py '|aminopy|)

(ql:quickload :ntdoc)


;; Output
(ntdoc::markdown '(:amino-type :amino)
                 :system :amino
                 :target "lisp-amino.md"
                 :title "Amino Lisp API"
                 :key "AMINO_LISP_AMINO")

(ntdoc::markdown :robray
                 :system :amino-rx
                 :target "lisp-rx.md"
                 :title "Robray Lisp API"
                 :key "AMINO_LISP_RX")
