;;;; -*- mode: lisp -*-
;;;;
;;;; Copyright (c) 2015, Rice University
;;;; All rights reserved.
;;;;
;;;; Author(s): Neil T. Dantam <ntd@gatech.edu>
;;;;
;;;;   Redistribution and use in source and binary forms, with or
;;;;   without modification, are permitted provided that the following
;;;;   conditions are met:
;;;;
;;;;   * Redistributions of source code must retain the above
;;;;     copyright notice, this list of conditions and the following
;;;;     disclaimer.
;;;;   * Redistributions in binary form must reproduce the above
;;;;     copyright notice, this list of conditions and the following
;;;;     disclaimer in the documentation and/or other materials
;;;;     provided with the distribution.
;;;;   * Neither the name of copyright holder the names of its
;;;;     contributors may be used to endorse or promote products
;;;;     derived from this software without specific prior written
;;;;     permission.
;;;;
;;;;   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
;;;;   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
;;;;   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
;;;;   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
;;;;   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
;;;;   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
;;;;   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
;;;;   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
;;;;   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
;;;;   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
;;;;   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
;;;;   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;;;   POSSIBILITY OF SUCH DAMAGE.

(in-package :robray)

(defparameter *pov-output* *standard-output*)
(defparameter *pov-handedness* :left)

(defparameter *pov-indent* "")
(defparameter *pov-newline-indent* #\Newline)
(defparameter *pov-indent-width* 3) ;; as used in povray.org docs

(defun pov-print (object stream)
  (write-sequence (rope-string (object-rope object))
                  stream))

(defun pov-rope-reduce (sequence separator)
  (rope-map #'object-rope sequence :separator separator))

(defmacro with-pov-indent (&body body)
  `(let* ((*pov-indent* (make-string (+ *pov-indent-width* (length *pov-indent*))
                                     :initial-element #\Space))
          (*pov-newline-indent* (rope #\Newline *pov-indent*)))
     ,@body))

(defun pov-cache-file (original-file &optional (directory *robray-tmp-directory*))
  (format-pathname "~A/povray/~A"
                   directory
                   (if (pathnamep original-file)
                       (namestring original-file)
                       (rope-string original-file))))

;;; Simple types ;;;

(defun pov-float (f)
  (format nil "~F" f))
(defun pov-int (i)
  (format nil "~D" i))

(defun pov-bracket (rope)
  (rope '< rope '>))

(defun pov-vector (function &rest args)
  (declare (dynamic-extent args))
  (pov-bracket
        (rope-map function args :separator '|, |)))

(defun %pov-integer-vector (x y z)
  (pov-bracket (rope (pov-int x) '|, |
                     (pov-int y) '|, |
                     (pov-int z) )))


(defun pov-integer-vector (elements)
  (%pov-integer-vector (vec-x elements)
                       (vec-y elements)
                       (vec-z elements)))

(defun %pov-float-vector (x y z)
  (pov-bracket (rope (pov-float x) '|, |
                     (pov-float y) '|, |
                     (pov-float z) )))

(defun pov-float-vector (elements)
  (with-vec3 (x y z) elements
    (%pov-float-vector x y z)))

(defun %pov-float-vector-right (x y z)
  (%pov-float-vector x z y))

(defun pov-float-vector-right (elements)
  (with-vec3 (x y z) elements
    (%pov-float-vector-right x y z)))

(defun pov-uv-vector (u v)
  (pov-vector #'object-rope u v))

(defun pov-item (name value)
  (rope name '| | value))

(defun %pov-rgb (r g b)
  (rope '|rgb|
        (%pov-float-vector r g b)))

(defun pov-rgb* (r g b)
  (%pov-rgb r g b))

(defun pov-rgb (elements)
  (multiple-value-call #'pov-rgb*
    (etypecase elements
      (list (values (first elements)
                    (second elements)
                    (third elements)))
      (array (values (aref elements 0)
                     (aref elements 1)
                     (aref elements 2))))))

(defun %pov-rgbf (r g b f)
  (rope '|rgbf|
        (pov-vector #'pov-float r g b f)))

(defun pov-rgbf* (r g b f)
  (%pov-rgbf r g b f))

(defun pov-rgbf (elements)
  (apply #'pov-rgbf* (subseq elements 0 4)))


;;; Complex Types ;;;


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
  (pov-print object stream))

(defmethod object-rope ((object pov-matrix))
  (let ((elements (pov-matrix-elements object)))
    (with-pov-indent
      (let ((sep-1 (rope '|,| *pov-newline-indent*))
            (sep-0 '|, |))
        (rope '|matrix <|
              *pov-newline-indent*
              (elt elements 0) sep-0
              (elt elements 1) sep-0
              (elt elements 2) sep-1

              (elt elements 3) sep-0
              (elt elements 4) sep-0
              (elt elements 5) sep-1

              (elt elements 6) sep-0
              (elt elements 7) sep-0
              (elt elements 8) sep-1

              (elt elements 9) sep-0
              (elt elements 10) sep-0
              (elt elements 11) '| >|)))))

(defstruct (pov-block (:constructor pov-block (name list)))
  name
  list)

(defmethod print-object ((object pov-block) stream)
  (pov-print object stream))

(defmethod object-rope ((object pov-block))
  (let ((old-indent *pov-newline-indent*))
    (with-pov-indent
      (rope (rope (pov-block-name object) '| {|
                  *pov-newline-indent*)
            (pov-rope-reduce (pov-block-list object) *pov-newline-indent*)
            (rope old-indent '|}|)))))

(defstruct (pov-list (:constructor %pov-list (name list length)))
  name
  length
  list)

(defun pov-list (name list &optional (length (length list)))
  (%pov-list name list length))

(defmethod print-object ((object pov-list) stream)
  (pov-print object stream))

(defmethod object-rope ((object pov-list))
 (let ((old-indent *pov-newline-indent*))
    (with-pov-indent
      (let ((sep (rope '|,| *pov-newline-indent*)))
        (rope (rope (pov-list-name object) '| {|)
              (rope *pov-newline-indent* (pov-list-length object)
                    sep)
              (pov-rope-reduce (pov-list-list object) sep)
              (rope old-indent '|}|))))))

(defun pov-color (color)
  (pov-item "color"
            (ecase (length color)
              (3 (pov-rgb color))
              (4 (pov-rgbf color)))))

(defun pov-alpha (alpha)
  (pov-item "transmit"
            (- 1d0 (clamp alpha 0d0 1d0))))

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
                    radius
                    modifiers)))

(defun pov-cylinder (first-center second-center radius &optional modifiers)
  (pov-block "cylinder"
             (list* first-center
                    second-center
                    radius
                    modifiers)))

(defun pov-cylinder-axis (axis radius &optional modifiers)
  (pov-cylinder (pov-float-vector-right '(0 0 0))
                (pov-float-vector-right axis)
                radius
                modifiers))


(defun pov-cone (big-center big-radius small-center small-radius &optional modifiers)
  (pov-block "cone" (list* big-center big-radius
                           small-center small-radius
                           modifiers)))

(defun pov-cone-axis (axis big-radius small-radius &optional modifiers)
  (pov-cone (pov-float-vector-right '(0 0 0)) big-radius
            (pov-float-vector-right axis) small-radius
            modifiers))


(defun pov-quote (text)
  ;; TODO: escape string
  (rope #\" text #\"))

(defun pov-text (value &key
                         (font :monospace)
                         (thickness 1)
                         (offset 0)
                         modifiers)
  (let ((font (case font
                ((:monospace :fixed) '|crystal.ttf|)
                (:serif '|timrom.ttf|)
                ((:sans :sans-serif) '|cyrvetic.ttf|)
                (otherwise font))))
    (pov-block '|text|
               (list* (rope '|ttf| #\Space
                            (pov-quote font) #\Space
                            (pov-quote value) #\Space
                            (format nil "~F, ~F"
                                    thickness offset))
                      modifiers))))


(defun pov-group-array (function array)
  (let* ((n (length array))
         (result (make-array (/ n 3))))
    (loop for i = 0 then (+ 3 i)
       for j from 0
       while (< i n)
       do (setf (aref result j)
                (funcall function
                         (aref array i)
                         (aref array (+ 1 i))
                         (aref array (+ 2 i)))))
    result))

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
        (arg mesh))

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
                                      (list face (aref texture-indices j))
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
                              collect (pov-uv-vector (aref uv-vectors i)
                                                     (aref uv-vectors (+ 1 i)))))))

          (when-let ((vertices (mesh-data-vertex-vectors mesh-data)))
            (arg (pov-list "vertex_vectors"
                           (pov-group-array vector-function vertices))))))

      (pov-block "mesh2" args))))

(defun pov-texture-list (textures)
  (pov-list "texture_list" textures))

(defun pov-image-map-type (type-ext)
  (let ((type-ext (string-downcase type-ext)))
    (string-case type-ext
      (("gif" "jpeg" "ppm" "pgm" "png" "tiff" "sys")
       type-ext)
      ("jpg" "jpeg")
      ("tif" "tiff")
      (otherwise (error "Unrecognized file type for image map: ~A" type-ext)))))

(defun pov-image-map (file &optional modifiers)
  (let ((type (pov-image-map-type (file-type file))))
    (pov-block "image_map"
               (list* (pov-item type (rope #\" file #\"))
                      modifiers))))

(defun pov-alist-texture (alist)
  (let ((finishes nil)
        (pigments nil)
        (has-image-map (assoc :image-map alist)))
    (labels ((avg-rgb (rgb)
               (etypecase rgb
                 ((or single-float double-float) rgb)
                 (sequence (/ (+ (elt rgb 0)
                                 (elt rgb 1)
                                 (elt rgb 2))
                              3))))
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
               (push "uv_mapping" pigments)))))
    (pov-texture (nconc (when pigments
                          (list (pov-pigment pigments)))
                        (when finishes
                          (list (pov-finish finishes)))))))


(defstruct (pov-version (:constructor pov-version (value)))
  value)

(defmethod print-object ((object pov-version) stream)
  (pov-print object stream))

(defmethod object-rope ((object pov-version))
  (rope '|#version |
        (pov-version-value object)
        '|;|))

(defstruct (pov-directive (:constructor pov-directive (type name value)))
  type
  name
  value)

(defun pov-declare (name value)
  (pov-directive "declare" name value))

(defmethod print-object ((object pov-directive) stream)
  (pov-print object stream))

(defmethod object-rope ((object pov-directive))
  (with-pov-indent
    (rope '|#| (pov-directive-type object) '| |
          (pov-directive-name object)
          '| = |
          (pov-directive-value object))))

(defstruct (pov-include (:constructor pov-include (file)))
  file)

(defmethod print-object ((object pov-include) stream)
  (pov-print object stream))

(defmethod object-rope ((object pov-include))
  (rope '|#include "| (pov-include-file object)
        '|"|))

(defstruct (pov-sequence (:constructor pov-sequence (statements)))
  statements)

(defmethod print-object ((object pov-sequence) stream)
  (pov-print object stream))

(defmethod object-rope ((object pov-sequence))
  (pov-rope-reduce (pov-sequence-statements object)
                   #\Newline))

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
  (with-pov-indent
    (format stream "~&#switch (~A) ~{~A~}~@[~A~]~&#end"
            (pov-switch-value object)
            (pov-switch-clauses object)
            (pov-switch-else object))))

(defstruct (pov-case (:constructor pov-case (value statements)))
  value
  statements)

(defmethod print-object ((object pov-case) stream)
  (let ((old-indent *pov-indent*))
    (with-pov-indent
      (format stream "~&~A#case (~A) ~{~A~}~&~A#break"
              old-indent
              (pov-case-value object) (pov-case-statements object)
              old-indent))))

(defstruct (pov-line-comment (:constructor %pov-line-comment (value)))
  (value "" :type string))

(defun pov-line-comment (value)
  (assert (not (find #\Newline value)))
  (%pov-line-comment value))

(defmethod print-object ((object pov-line-comment) stream)
  (format stream "~&~A// ~A ~%"
          *pov-indent*
          (pov-line-comment-value object)))

(defmethod object-rope ((object pov-line-comment))
  (rope '|// | (pov-line-comment-value object)))

;(defmethod print-object

(defun pov-camera (tf &key
                        (width 16)
                        (height 9))
  (let* ((quat (tf-quaternion tf))
         (trans (tf-translation tf))
         (lookat (g+ trans
                     (transform quat (vec3* 0 0 -1))))
         (sky (transform quat (vec3* 0 1 0))))
  (pov-block "camera"
             (list
              ;(pov-item "location" (pov-float-vector-right (vec3* 0 0 0)))
              (pov-item "location" (pov-float-vector-right trans))
              (pov-item "look_at"  (pov-float-vector-right lookat))
              (pov-item "sky"  (pov-float-vector-right sky))
              (pov-item "right" (rope "x*" width "/" height))))))


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
      (princ (rope-string (rope-split " " args)))
      (multiple-value-bind (output status)
          (capture-program-output args :directory directory)
        (unless (zerop status)
          (error "Povray rendering failed.~%~%~A" output))))))
