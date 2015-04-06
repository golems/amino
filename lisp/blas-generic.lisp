(in-package :amino)

;;;;;;;;;;;;;
;; LEVEL 1 ;;
;;;;;;;;;;;;;

(defmethod g* ((a matrix) (b array))
  (let ((y (make-vec (matrix-rows a))))
    (dgemv 1d0 a b 0d0 y)
    y))
