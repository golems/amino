(in-package :robray)

(defparameter *pov-output* *standard-output*)
(defparameter *pov-handedness* :left)

(defparameter *pov-indent* "")
(defparameter *pov-indent-width* 3) ;; as used in povray.org docs

;; Convert something to a POV-ray object
(defgeneric pov-object (object))

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

(defparameter *pov-swap-xy* (amino::col-matrix '(1 0 0) '(0 0 1) '(0 1 0)))

(defun rotation-matrix-swap-yz (matrix)
  (amino::tf-rotmat-mul *pov-swap-xy* matrix))

(defstruct pov-matrix
  elements)

(defgeneric pov-matrix (object))

(defmethod pov-matrix ((tf cons))
  (assert (= 12 (length tf)))
  (make-pov-matrix :elements
                   (loop for x in tf
                      collect (pov-value (pov-float x)))))

(defmethod pov-matrix ((tf array))
  (check-type tf (array double-float (12)))
  (pov-matrix (loop for x across tf collect x)))


(defmethod pov-matrix ((tf quaternion-translation))
  (let* ((matrix-0 (rotation-matrix (rotation tf)))
         (matrix-swap (amino::tf-rotmat-mul (amino::tf-rotmat-mul *pov-swap-xy* matrix-0)
                                            *pov-swap-xy*))
         (translation (translation tf)))
    ;(check-type matrix (simple-array double-float (9)))
    (pov-matrix (vec-cat (amino::matrix-data matrix-swap)
                         (vec3* (vec-x translation)
                                (vec-z translation)
                                (vec-y translation))))))

;; TODO: remap axes / swap YZ
(defmethod print-object ((object pov-matrix) stream)
  (with-pov-indent old-indent
    (format stream "~&~Amatrix <~{~A~^, ~}~&~A>"
            old-indent (pov-matrix-elements object) old-indent)))


(defstruct (pov-integer-vector (:constructor %pov-integer-vector (x y z)))
  "Type for a povray vector."
  (x 0 :type fixnum)
  (y 0 :type fixnum)
  (z 0 :type fixnum))

(defun pov-float-vector (elements)
  (%pov-float-vector (vec-x elements)
                     (vec-y elements)
                     (vec-z elements)))


(defmethod pov-object ((object vec3))
  (with-vec3 (x y z) object
    (%pov-float-vector x z y)))

(defun pov-float-vector-right (elements)
  (pov-object (vec3* (vec-x elements)
                     (vec-y elements)
                     (vec-z elements))))


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

(defun pov-rgb* (r g b)
  (%pov-rgb (coerce r 'double-float)
            (coerce g 'double-float)
            (coerce b 'double-float)))

(defun pov-rgb (elements)
  (apply #'pov-rgb* (subseq elements 0 3)))

(defmethod print-object ((object pov-rgb) stream)
  (format stream "rgb<~F, ~F, ~F>"
          (pov-rgb-r object)
          (pov-rgb-g object)
          (pov-rgb-b object)))

(defmethod print-object ((object pov-item) stream)
  (format stream "~&~A~A ~A"
          *pov-indent* (pov-item-name object) (pov-item-value object)))

(defun pov-texture (things)
  (pov-block "texture" things))
(defun pov-texture* (&rest things)
  (pov-texture things))

(defun pov-finish (things)
  (pov-block "finish" things))
(defun pov-finish* (&rest things)
  (pov-finish things))

(defun pov-pigment (things)
  (pov-block "pigment" things))
(defun pov-pigment* (&rest things)
  (pov-pigment things))

(defun pov-box (first-corner second-corner &optional modifiers)
  (pov-block "box" (list* first-corner second-corner modifiers)))

(defun pov-sphere (center radius &optional modifiers)
  (pov-block "sphere"
             (list* (pov-float-vector-right center)
                    (pov-value (pov-float radius))
                    modifiers)))


(defun pov-box-center (dimensions
                       &key modifiers)
  (let* ((first-corner-vec (g* 0.5 dimensions))
         (second-corner-vec (g* -.05 dimensions)))
    (pov-box (pov-float-vector-right first-corner-vec)
             (pov-float-vector-right second-corner-vec)
             modifiers)))


(defun pov-mesh2 (&key
                    vertex-vectors
                    face-indices
                    texture-list
                    normal-vectors
                    normal-indices
                    modifiers
                    mesh
                    matrix
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
    (when matrix
      (push (pov-matrix matrix) args))
    (when mesh
      (push (pov-value mesh) args))

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


(defstruct (pov-include (:constructor pov-include (file)))
  file)

(defmethod print-object ((object pov-include) stream)
  (format stream "~&#include ~S"
          (pov-include-file object)))

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
