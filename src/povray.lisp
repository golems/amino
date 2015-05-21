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


(defstruct (pov-uv-vector (:constructor %pov-uv-vector (u v)))
  "Type for a povray vector."
  (u 0d0 :type double-float)
  (v 0d0 :type double-float))

(defmethod print-object ((object pov-uv-vector) stream)
  (format stream "~&~A<~F, ~F>"
          *pov-indent*
          (pov-uv-vector-u object)
          (pov-uv-vector-v object)))


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

(defun %pov-float-vector-right (x y z)
  (%pov-float-vector x z y))

(defun pov-float-vector-right (elements)
  (%pov-float-vector-right (coerce (vec-x elements) 'double-float)
                           (coerce (vec-y elements) 'double-float)
                           (coerce (vec-z elements) 'double-float)))

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
  (multiple-value-call #'pov-rgb*
    (etypecase elements
      (list (values (first elements)
                    (second elements)
                    (third elements)))
      (array (values (aref elements 0)
                     (aref elements 1)
                     (aref elements 2))))))

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
  (pov-item "color"
            (ecase (length color)
              (3 (pov-rgb color))
              (4 (pov-rgbf color)))))

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


(defun pov-group-array (function array)
  (let ((n (length array)))
    (loop for i = 0 then (+ 3 i)
       while (< i n)
       collect (funcall function
                        (aref array i)
                        (aref array (+ 1 i))
                        (aref array (+ 2 i))))))




(defun pov-mesh2 (&key
                    mesh-data
                    modifiers
                    mesh
                    matrix
                    (handedness :right)
                    )
  "Create a povray mesh2 object.

VERTEX-VECTORS: List of vertices in the mesh as pov-vertex
FACE-INDICES: List of vertex indices for each triangle, as pov-vertex
"
  (let ((args modifiers)
        (vector-function (ecase handedness
                           (:right #'%pov-float-vector-right)
                           (:left #'%pov-float-vector))))
    (labels ((arg (arg) (push arg args)))

      (when matrix
        (arg (pov-matrix matrix)))
      (when mesh
        (arg (pov-value mesh)))

      (when mesh-data
        (let* ((textures (mesh-data-texture-properties mesh-data))
               (texture-indices (mesh-data-texture-indices mesh-data))
               (vertex-indices (mesh-data-vertex-indices mesh-data))
               (face-count (/ (length vertex-indices) 3)))

          (when (= (length textures) 1)
            (arg (pov-alist-texture (car textures))))

          (let ((normal-indices (mesh-data-normal-indices mesh-data)))
            (when (>  (length normal-indices) 0)
              (arg (pov-list "normal_indices"
                             (pov-group-array #'%pov-integer-vector normal-indices)))))

          (let ((uv-indices (mesh-data-uv-indices mesh-data)))
            (format t "~&uv-len: ~A" (length uv-indices))
            (when (>  (length uv-indices) 0)
              (assert (= face-count
                         (/ (length uv-indices) 3)))
              (arg (pov-list "uv_indices"
                             (pov-group-array #'%pov-integer-vector uv-indices)))))

          (when vertex-indices
            (let* ((n (length vertex-indices))
                   (has-texture (and textures
                                     texture-indices
                                     (> (length textures) 1))))
              (when has-texture
                (assert (= (length texture-indices)
                           (/ (length vertex-indices) 3))))
              (arg (pov-list "face_indices"
                             (loop for i = 0 then (+ 3 i)
                                for j from 0
                                while (< i n)
                                for face = (%pov-integer-vector (aref vertex-indices i)
                                                                (aref vertex-indices (+ 1 i))
                                                                (aref vertex-indices (+ 2 i)))
                                nconc
                                  (if has-texture
                                      (list face (pov-value (aref texture-indices j)))
                                      (list face)))
                             face-count))))

          (when (> (length textures) 1)
            (arg (pov-texture-list (map 'list #'pov-alist-texture textures))))

          (when-let ((normals (mesh-data-normal-vectors mesh-data)))
            (arg (pov-list "normal_vectors"
                           (pov-group-array vector-function normals))))

          (when-let ((uv-vectors (mesh-data-uv-vectors mesh-data)))
            (arg (pov-list "uv_vectors"
                           (loop with n = (length uv-vectors)
                              for i = 0 then (+ 2 i)
                              while (< i n)
                              collect (%pov-uv-vector (aref uv-vectors i)
                                                      (aref uv-vectors (+ 1 i)))))))

          (when-let ((vertices (mesh-data-vertex-vectors mesh-data)))
            (arg (pov-list "vertex_vectors"
                           (pov-group-array vector-function vertices))))))

      (pov-block "mesh2" args))))

(defun pov-texture-list (textures)
  (pov-list "texture_list" textures))


(defun pov-image-map (file &optional modifiers)
  (let* ((type-ext (string-downcase (file-type file)))
         (type (string-case type-ext
                 (("gif" "jpeg" "ppm" "pgm" "png" "tiff" "sys")
                  type-ext)
                 ("jpg" "jpeg")
                 ("tif" "tiff")
                 (otherwise (error "Unrecognized file type for image map: ~A" file)))))
    (pov-block "image_map"
               (list* (pov-item type (format nil "\"~A\"" file))
                      modifiers))))

(defun pov-alist-texture (alist)
  (let ((finishes nil)
        (pigments nil)
        (has-image-map (assoc :image-map alist)))
    (labels ((avg-rgb (rgb)
               (pov-float (etypecase rgb
                            ((or single-float double-float) rgb)
                            (sequence (/ (+ (elt rgb 0)
                                            (elt rgb 1)
                                            (elt rgb 2))
                                         3)))))
             (add-finish (name finish)
               (push (pov-item name finish) finishes))
             (add-pigment (name pigment)
               (unless has-image-map
                 (push (pov-item name pigment) pigments)))
             (alpha ()
               (when-let ((assoc-alpha (assoc :alpha alist)))
                 (unless has-image-map
                   (push (pov-alpha (cdr assoc-alpha)) pigments)))))
      (loop
         for property in '(:ambient :diffuse :color :specular :index-of-refraction :image-map)
         for pair = (assoc property alist)
         for value = (cdr pair)
         when pair
         do (case property
              (:ambient
               (add-finish "ambient" (pov-rgb value)))
              (:diffuse
               (add-finish "diffuse" (avg-rgb value))
               ;; color information can also in the diffuse property
               (when (and (or (listp value)
                              (vectorp value))
                          (not (assoc :color alist)))
                 (alpha)
                 (add-pigment "color" (pov-rgb value))))
              (:color
               (alpha)
               (add-pigment "color" (pov-rgb value)))
              (:specular
               (add-finish "specular" (avg-rgb value)))
              (:index-of-refraction ; TODO
               )
              (:image-map
               (push (pov-image-map value) pigments)
               ;; TODO: assumed uv mapping
               (push (pov-value "uv_mapping") pigments)))))
    (pov-texture (nconc (when pigments
                          (list (pov-pigment pigments)))
                        (when finishes
                          (list (pov-finish finishes)))))))


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
    (let ((args (cons "povray" (pov-args file
                                         :output output
                                         :options options))))
      (format t "~&Running: ~{~A~^ ~}" args)
      (multiple-value-bind (output error-output status)
          (uiop/run-program:run-program args
                                        :directory directory)
        (declare (ignore output error-output))
        (if (zerop status)
            (format t "~&done")
            (format t "~&Povray rendering failed~%"))))))
