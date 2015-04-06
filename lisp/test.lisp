(in-package :amino)


(defun test ()
  (assert (equalp (aa:g* (aa::row-matrix '(1 2 3) '(4 5 6))
                         (vec 1 2 3))
                  (vec 14 32))))
