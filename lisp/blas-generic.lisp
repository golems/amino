(in-package :amino)

;;;;;;;;;;;;;
;; LEVEL 1 ;;
;;;;;;;;;;;;;

(defmethod generic* ((a real) (b simple-array))
  (ecase (array-element-type b)
    (double-float (dscal (coerce a 'double-float) (vec-copy b)))))

(defmethod generic* ((a real) (b simple-array))
  (ecase (array-element-type b)
    (double-float (dscal (coerce a 'double-float) (vec-copy b)))))

(defmethod generic* ((a simple-array) (b real))
  (g* b a))

;;;;;;;;;;;;;
;; LEVEL 2 ;;
;;;;;;;;;;;;;
(defmethod generic* ((a matrix) (b array))
  (let ((y (make-vec (matrix-rows a))))
    (dgemv 1d0 a b 0d0 y)))
