;; Load doc generator
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
