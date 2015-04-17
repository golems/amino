(in-package :robray)

(defparameter *pov-output* *standard-output*)
(defparameter *pov-handedness* :left)

(defparameter *pov-indent* "")
(defparameter *pov-indent-width* 3) ;; as used in povray.org docs

(defmacro with-pov-indent (old-indent &body body)
  `(let* ((,old-indent *pov-indent*)
          (*pov-indent* (make-string (+ *pov-indent-width* (length *pov-indent*))
                                     :initial-element #\Space)))
     ,@body))

;; (defmacro with-pov-block (output &body body)
;;   `(progn
;;      (write-char #\{ ,output)
;;      (let ((*pov-indent* (make-string (+ *pov-indent-width* (length *pov-indent*))
;;                                       :initial-element #\Space)))
;;        ,@body)
;;      (format ,output "~&~A}" *pov-indent*)))

(defstruct (pov-float (:constructor pov-float (value)))
  (value 0 :type double-float))

(defmethod print-object ((object pov-float) stream)
  (format stream "~F"
          (pov-float-value object)))

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

(defun pov-float-vector-right (elements)
  (%pov-float-vector
   (elt elements 0)
   (elt elements 2)
   (elt elements 1)))


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
  (with-pov-indent old-indent
    (format stream
            "~&~A~A {~&~{~&~A~}~&~A}"
            old-indent
            (pov-block-name object)
            (pov-block-list object)
            old-indent)))

(defstruct (pov-list (:constructor %pov-list (name list length)))
  name
  length
  list)

(defun pov-list (name list &optional (length (length list)))
  (%pov-list name list length))

(defmethod print-object ((object pov-list) stream)
  (with-pov-indent old-indent
    (format stream
            "~&~A~A {~D,~{~A~^,~}~&~A}"
            old-indent
            (pov-list-name object)
            (pov-value (pov-list-length object))
            (pov-list-list object)
            old-indent)))

(defstruct (pov-item (:constructor pov-item (name value)))
  name
  value)

(defstruct (pov-rgb (:constructor %pov-rgb (r g b)))
  "Type for a povray RGB color."
  (r 0d0 :type double-float)
  (g 0d0 :type double-float)
  (b 0d0 :type double-float))

(defun pov-rgb (elements)
  (apply #'%pov-rgb (subseq elements 0 3)))

(defmethod print-object ((object pov-rgb) stream)
  (format stream "rgb<~F, ~F, ~F>"
          (pov-rgb-r object)
          (pov-rgb-g object)
          (pov-rgb-b object)))

(defmethod print-object ((object pov-item) stream)
  (format stream "~&~A~A ~A"
          *pov-indent* (pov-item-name object) (pov-item-value object)))

(defun pov-texture* (&rest things)
  (pov-block "texture" things))

(defun pov-texture (things)
  (pov-block "texture" things))

(defun pov-finish (things)
  (pov-block "finish" things))

(defun pov-finish* (&rest things)
  (pov-block "finish" things))

(defun pov-mesh2 (&key
                    vertex-vectors
                    face-indices
                    texture-list
                    normal-vectors
                    normal-indices
                    modifiers
                    )
  "Create a povray mesh2 object.

VERTEX-VECTORS: List of vertices in the mesh as pov-vertex
FACE-INDICES: List of vertex indices for each triangle, as pov-vertex
"
  (declare (ignore normal-vectors normal-indices))
  (let ((args modifiers))
    ;; TODO: figure this out
    ;; (when normal-indices
    ;;   (push (pov-list "normal_indices" normal-indices) args))
    ;; (when normal-vectors
    ;;   (push (pov-list "normal_vectors" normal-vectors) args))

    (when face-indices
      (push (pov-list "face_indices"
                      face-indices
                      (if texture-list
                          (/ (length face-indices) 2)
                          (length face-indices)))
            args))
    (when texture-list
      (push (pov-texture-list texture-list) args))
    (when vertex-vectors
      (push (pov-list "vertex_vectors" vertex-vectors) args))

    (pov-block "mesh2" args)))

(defun pov-texture-list (textures)
  (pov-list "texture_list" textures))

(defstruct (pov-directive (:constructor pov-directive (type name value)))
  type
  name
  value)

(defun pov-declare (name value)
  (pov-directive "declare" name value))

(defmethod print-object ((object pov-directive) stream)
  (with-pov-indent old-indent
    (declare (ignore old-indent))
    (format stream "#~A ~A = ~A"
            (pov-directive-type object)
            (pov-directive-name object)
            (pov-directive-value object))))

(defstruct (pov-sequence (:constructor pov-sequence (statements)))
  statements)

(defmethod print-object ((object pov-sequence) stream)
  (loop for x in (pov-sequence-statements object)
     do (print-object x stream)))

;; (defun print-vector-handed (vector &optional (output *pov-output*))
;;   (ecase *pov-handedness*
;;     (:left
;;      (print-vector-xyz (vec-x vector) (vec-z vector) (vec-y vector)
;;                        output))
;;     (:right
;;      (print-vector-xyz (vec-x vector) (vec-y vector) (vec-z vector)
;;                        output))))

;(defun print-mesh2 (vertex-vectors normal-vectors
