(in-package :amino)


(defun test ()
  (assert (equalp (aa:g* (aa::row-matrix '(1 2 3) '(4 5 6))
                         (aa:vec 1 2 3))
                  (aa:vec 14 32))))
