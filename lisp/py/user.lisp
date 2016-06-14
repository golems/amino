(in-package |aminopy|)


;;;;;;;;;;;;;;;;
;; Transforms ;;
;;;;;;;;;;;;;;;;

(defun |tf| (a)
  (tf a))

(defun |tf2| (a b)
  (tf* a b))

(defun |xangle| (a)
  (x-angle a))

(defun |yangle| (a)
  (y-angle a))

(defun |zangle| (a)
  (z-angle a))

(defun |vec| (&rest args)
  (apply #'vec args))
