(in-package :robray)

(defparameter *pov-output* *standard-output*)
(defparameter *pov-handedness* :left)

(defparameter *pov-indent* "")
(defparameter *pov-indent-width* 3) ;; as used in povray.org docs

(defmacro with-pov-block (output &body body)
  `(progn
     (write-char #\{ ,output)
     (let ((*pov-indent* (make-string (+ *pov-indent-width* (length *pov-indent*))
                                      :initial-element #\Space)))
       ,@body)
     (format ,output "~&~A}" *pov-indent*)))

(defstruct (pov-float-vector (:constructor %pov-float-vector (x y z)))
  "Type for a povray vector."
  (x 0d0 :type double-float)
  (y 0d0 :type double-float)
  (z 0d0 :type double-float))

(defstruct (pov-integer-vector (:constructor %pov-integer-vector (x y z)))
  "Type for a povray vector."
  (x 0 :type fixnum)
  (y 0 :type fixnum)
  (z 0 :type fixnum))

(defun pov-float-vector (elements)
  (apply #'%pov-float-vector elements))

(defun pov-integer-vector (elements)
  (apply #'%pov-integer-vector elements))

(defmethod print-object ((object pov-float-vector) stream)
  (format stream "~&~A<~F, ~F, ~F>"
          *pov-indent*
          (pov-float-vector-x object)
          (pov-float-vector-y object)
          (pov-float-vector-z object)))

(defmethod print-object ((object pov-integer-vector) stream)
  (format stream "~&~A<~D, ~D, ~D>"
          *pov-indent*
          (pov-integer-vector-x object)
          (pov-integer-vector-y object)
          (pov-integer-vector-z object)))

(defstruct (pov-value (:constructor pov-value (value)))
  value)

(defmethod print-object ((object pov-value) stream)
  (format stream "~&~A~A"
          *pov-indent*
          (pov-value-value object)))


(defstruct (pov-block (:constructor pov-block (name list)))
  name
  list)

(defmethod print-object ((object pov-block) stream)
  (let* ((old-indent *pov-indent*)
         (*pov-indent* (make-string (+ *pov-indent-width* (length *pov-indent*))
                                    :initial-element #\Space)))
    (format stream
            "~&~A~A {~&~{~&~A~}~&~A}"
            old-indent
            (pov-block-name object)
            (pov-block-list object)
            old-indent)))

(defstruct (pov-list (:constructor pov-list (name list)))
  name
  list)

(defmethod print-object ((object pov-list) stream)
  (let* ((old-indent *pov-indent*)
         (*pov-indent* (make-string (+ *pov-indent-width* (length *pov-indent*))
                                    :initial-element #\Space)))
    (format stream
            "~&~A~A {~&~{~&~A~^,~}~&~A}"
            old-indent
            (pov-list-name object)
            (pov-list-list object)
            old-indent)))

(defun pov-mesh2 (&key
                    vertex-vectors
                    face-indices
                    normal-vectors
                    normal-indices
                    )
  "Create a povray mesh2 object.

VERTEX-VECTORS: List of vertices in the mesh as pov-vertex
FACE-INDICES: List of vertex indices for each triangle, as pov-vertex
"
  (let ((args))
    (when vertex-vectors
      (push (pov-list "vertex_vectors" vertex-vectors) args))
    (when face-indices
      (push (pov-list "face_indices" face-indices) args))
    (when normal-vectors
      (push (pov-list "normal_vectors" normal-vectors) args))
    (when normal-indices
      (push (pov-list "normal" normal-indices) args))

    (pov-block "mesh2" args)))







;; (defun print-vector-handed (vector &optional (output *pov-output*))
;;   (ecase *pov-handedness*
;;     (:left
;;      (print-vector-xyz (vec-x vector) (vec-z vector) (vec-y vector)
;;                        output))
;;     (:right
;;      (print-vector-xyz (vec-x vector) (vec-y vector) (vec-z vector)
;;                        output))))

;(defun print-mesh2 (vertex-vectors normal-vectors
