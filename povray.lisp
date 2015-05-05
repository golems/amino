(in-package :robray)

(defparameter *pov-output* *standard-output*)
(defparameter *pov-handedness* :left)

(defparameter *pov-indent* "")
(defparameter *pov-indent-width* 3) ;; as used in povray.org docs

;; Convert something to a POV-ray object

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

(defstruct (pov-float (:constructor %pov-float (value)))
  (value 0d0 :type double-float))

(defun pov-float (value)
  (%pov-float (coerce value 'double-float)))

(defmethod print-object ((object pov-float) stream)
  (format stream "~F"
          (pov-float-value object)))

(defstruct (pov-float-vector (:constructor %pov-float-vector (x y z)))
  "Type for a povray vector."
  (x 0d0 :type double-float)
  (y 0d0 :type double-float)
  (z 0d0 :type double-float))

(defstruct pov-matrix
  elements)

(defun pov-matrix (tf)
  (let* ((matrix (amino::matrix-data (rotation-matrix (rotation tf))))
         (translation (translation tf)))
    ;; Swap the Y and Z axes because povray is left-handed
    (with-vec3 (x y z) translation
      (make-pov-matrix :elements
                       (list (aref matrix 0) (aref matrix 2) (aref matrix 1)
                             (aref matrix 6) (aref matrix 8) (aref matrix 7)
                             (aref matrix 3) (aref matrix 5) (aref matrix 4)
                             x z y)))))

(defmethod print-object ((object pov-matrix) stream)
  (let ((elements (pov-matrix-elements object)))
    (with-pov-indent old-indent
      (format stream "~&~Amatrix <~&~A~F, ~F, ~F,~&~A~F, ~F, ~F,~&~A~F, ~F, ~F,~&~A~F, ~F, ~F~&~A>"
              old-indent
              *pov-indent*
              (elt elements 0)
              (elt elements 1)
              (elt elements 2)
              *pov-indent*
              (elt elements 3)
              (elt elements 4)
              (elt elements 5)
              *pov-indent*
              (elt elements 6)
              (elt elements 7)
              (elt elements 8)
              *pov-indent*
              (elt elements 9)
              (elt elements 10)
              (elt elements 11)
              old-indent))))

(defstruct (pov-integer-vector (:constructor %pov-integer-vector (x y z)))
  "Type for a povray vector."
  (x 0 :type fixnum)
  (y 0 :type fixnum)
  (z 0 :type fixnum))

(defun pov-float-vector (elements)
  (%pov-float-vector (coerce (vec-x elements) 'double-float)
                     (coerce (vec-y elements) 'double-float)
                     (coerce (vec-z elements) 'double-float)))

(defun pov-float-vector-right (elements)
  (%pov-float-vector (coerce (vec-x elements) 'double-float)
                     (coerce (vec-z elements) 'double-float)
                     (coerce (vec-y elements) 'double-float)))

(defun pov-integer-vector (elements)
  (%pov-integer-vector (vec-x elements)
                       (vec-y elements)
                       (vec-z elements)))

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

(defstruct (pov-rgbf (:constructor %pov-rgbf (r g b f)))
  "Type for a povray RGB color."
  (r 0d0 :type double-float)
  (g 0d0 :type double-float)
  (b 0d0 :type double-float)
  (f 0d0 :type double-float))

(defun pov-rgbf* (r g b f)
  (%pov-rgbf (coerce r 'double-float)
             (coerce g 'double-float)
             (coerce b 'double-float)
             (coerce f 'double-float)))

(defun pov-rgbf (elements)
  (apply #'pov-rgbf* (subseq elements 0 4)))

(defmethod print-object ((object pov-rgbf) stream)
  (format stream "rgbf<~F, ~F, ~F, ~F>"
          (pov-rgbf-r object)
          (pov-rgbf-g object)
          (pov-rgbf-b object)
          (pov-rgbf-f object)))

(defun pov-color (color)
  (etypecase color
    (list
     (pov-item "color"
               (ecase (length color)
                 (3 (pov-rgb color))
                 (4 (pov-rgbf color)))))))

(defun pov-alpha (alpha)
  (pov-item "transmit"
            (pov-float (- 1d0 (clamp alpha 0d0 1d0)))))

(defmethod print-object ((object pov-item) stream)
  (format stream "~&~A~A ~A"
          *pov-indent* (pov-item-name object) (pov-item-value object)))

(defmacro def-pov-block (name)
  (let ((name (string-downcase (string name))))
    (let ((fun (intern (string-upcase (concatenate 'string "pov-" name))))
          (fun* (intern (string-upcase (concatenate 'string "pov-" name "*")))))
      `(progn
         (defun ,fun (things)
           (pov-block ,name (ensure-list things)))
         (defun ,fun* (&rest things)
           (,fun things))))))

(def-pov-block texture)
(def-pov-block finish)
(def-pov-block pigment)
(def-pov-block transform)


(defun pov-box (first-corner second-corner &optional modifiers)
  (pov-block "box" (list* first-corner second-corner modifiers)))

(defun pov-box-center (dimensions
                       &key modifiers)
  (let* ((first-corner-vec (g* 0.5d0 dimensions))
         (second-corner-vec (g* -.5d0 dimensions)))
    (pov-box (pov-float-vector-right first-corner-vec)
             (pov-float-vector-right second-corner-vec)
             modifiers)))

(defun pov-sphere (center radius &optional modifiers)
  (pov-block "sphere"
             (list* (pov-float-vector-right center)
                    (pov-value (pov-float radius))
                    modifiers)))

(defun pov-cylinder (first-center second-center radius &optional modifiers)
  (pov-block "cylinder"
             (list* first-center
                    second-center
                    (pov-value (pov-float radius))
                    modifiers)))

(defun pov-cylinder-axis (axis radius &optional modifiers)
  (pov-cylinder (pov-float-vector-right '(0 0 0))
                (pov-float-vector-right axis)
                radius
                modifiers))


(defun pov-cone (big-center big-radius small-center small-radius &optional modifiers)
  (pov-block "cone" (list* big-center (pov-value (pov-float big-radius))
                           small-center (pov-value (pov-float small-radius))
                           modifiers)))

(defun pov-cone-axis (axis big-radius small-radius &optional modifiers)
  (pov-cone (pov-float-vector-right '(0 0 0)) big-radius
            (pov-float-vector-right axis) small-radius
            modifiers))





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
  (let ((args modifiers))
    ;; TODO: figure this out
    ;; (when normal-indices
    ;;   (push (pov-list "normal_indices" normal-indices) args))
    (when matrix
      (push (pov-matrix matrix) args))
    (when mesh
      (push (pov-value mesh) args))

    (when normal-indices
      (push (pov-list "normal_indices" normal-indices) args))

    (when face-indices
      (push (pov-list "face_indices"
                      face-indices
                      (if texture-list
                          (/ (length face-indices) 2)
                          (length face-indices)))
            args))
    (when texture-list
      (push (pov-texture-list texture-list) args))

    (when normal-vectors
      (push (pov-list "normal_vectors" normal-vectors) args))
    (when vertex-vectors
      (push (pov-list "vertex_vectors" vertex-vectors) args))

    (pov-block "mesh2" args)))

(defun pov-texture-list (textures)
  (pov-list "texture_list" textures))


(defstruct (pov-version (:constructor pov-version (value)))
  value)

(defmethod print-object ((object pov-version) stream)
  (with-pov-indent old-indent
    (declare (ignore old-indent))
    (format stream "~&#version ~A;"
            (pov-version-value object))))

(defstruct (pov-directive (:constructor pov-directive (type name value)))
  type
  name
  value)

(defun pov-declare (name value)
  (pov-directive "declare" name value))

(defmethod print-object ((object pov-directive) stream)
  (with-pov-indent old-indent
    (declare (ignore old-indent))
    (format stream "~&#~A ~A = ~A"
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

(defun pov-quality (float-quality)
  "Return numeric povray quality for proportional to RATIO.
RATIO: Floating point quality value in the range [0,1]"
  (clamp (round (* float-quality 11))
         0 11))


(defstruct (pov-if= (:constructor pov-if= (variable value statements)))
  variable
  value
  statements)

(defstruct (pov-switch (:constructor pov-switch (value clauses &optional else)))
  value
  clauses
  else)

(defmethod print-object ((object pov-switch) stream)
  (with-pov-indent old-indent
    (declare (ignore old-indent))
    (format stream "~&#switch (~A) ~{~A~}~@[~A~]~&#end"
            (pov-switch-value object)
            (pov-switch-clauses object)
            (pov-switch-else object))))

(defstruct (pov-case (:constructor pov-case (value statements)))
  value
  statements)

(defmethod print-object ((object pov-case) stream)
  (with-pov-indent old-indent
    (format stream "~&~A#case (~A) ~{~A~}~&~A#break"
            old-indent
            (pov-case-value object) (pov-case-statements object)
            old-indent)))

(defstruct (pov-line-comment (:constructor %pov-line-comment (value)))
  (value "" :type string))

(defun pov-line-comment (value)
  (assert (not (find #\Newline value)))
  (%pov-line-comment value))

(defmethod print-object ((object pov-line-comment) stream)
  (format stream "~&~A// ~A ~%"
          *pov-indent*
          (pov-line-comment-value object)))

;(defmethod print-object

(defun pov-args (file
                 &key
                   output
                   (options *render-options*)
                   verbose
                   threads
                   other)
  `(,(namestring file)
     ,@(when output (list (format nil "+O~A" output)))
     "-D" ; don't invoke display
     "-GS"
     ,@(when (get-render-option options :antialias) (list "+A")) ; anti-alias
     ,@(when threads (list (format nil "+WT~D" threads)))
     ,(if verbose "+V" "-V")
     ,(format nil "+Q~D" (pov-quality (get-render-option options :quality)))
     ,(format nil "+W~D" (get-render-option options :width))
     ,(format nil "+H~D" (get-render-option options :height))
     ,@other))

(defun pov-output-file (pov-file &optional (suffix ".png"))
  (ppcre:regex-replace "\.pov$" (namestring pov-file) suffix))

(defun pov-render (things
                   &key
                     file
                     output
                     (options *render-options*)
                     (directory *robray-tmp-directory*))
  (let ((things (if (listp things)
                    (pov-sequence things)
                    things)))

    ;; write output
    (output things file :directory directory)
    ;; run povray
    (let ((args (pov-args file
                          :output output
                          :options options)))
      (format t "~&Running: povray ~{~A~^ ~}" args)
      (let ((result
             (sb-ext:run-program "povray" args
                                 :search t
                                 :directory directory
                                 :wait t)))
        (if (zerop (sb-ext:process-exit-code result))
            (format t "~&done")
            (format t "~&Povray rendering failed~%"))))))
