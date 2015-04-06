(in-package :amino)

;;;;;;;;;;;;;
;; LEVEL 1 ;;
;;;;;;;;;;;;;

(defmethod g* ((a number) (b simple-array))
  (ecase (array-element-type b)
    (double-float (dscal (coerce a 'double-float) (vec-copy b)))))

(defmethod g* ((a number) (b simple-array))
  (ecase (array-element-type b)
    (double-float (dscal (coerce a 'double-float) (vec-copy b)))))

(defmethod g* ((a simple-array) (b number))
  (g* b a))

;;;;;;;;;;;;;
;; LEVEL 2 ;;
;;;;;;;;;;;;;
(defmethod g* ((a matrix) (b array))
  (let ((y (make-vec (matrix-rows a))))
    (dgemv 1d0 a b 0d0 y)
    y))
