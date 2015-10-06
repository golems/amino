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

(defvar *scene-graph*)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; SCENE FRAMES STRUCTURES ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; GEOMETRY ;;;


(defstruct scene-mesh
  "A scene object defined by a mesh."
  name
  source-file
  povray-file)

(defstruct scene-box
  "A box object.
The center of the box is at the origin, and it is aligned with the frame axes."
  dimension)
(defun scene-box (dimension)
  (make-scene-box :dimension dimension))

(defstruct scene-sphere
  "A sphere object.
The center of the sphere is at the origin."
  radius)
(defun scene-sphere (radius)
  (make-scene-sphere :radius radius))


(defstruct (scene-cylinder (:constructor scene-cylinder))
  "A cylinder object.
The cylinder starts at the origin and extends by HEIGHT in the Z direction."
  height
  radius)

(defstruct (scene-cone (:constructor scene-cone))
  "A cone object.
The cone starts at the origin and extends by HEIGHT in the Z direction."
  height
  start-radius
  end-radius)

(defstruct scene-grid
  "A grid object.
The center of the grid plane is at the origin, and it is normal to the Z axis"
  dimension
  delta
  thickness)
(defun scene-grid (dimension delta thickness)
  (make-scene-grid :dimension dimension
                   :delta delta
                   :thickness thickness))

(defparameter *scene-font* :monospace)

(defstruct scene-text
  (value nil :type rope)
  (font *scene-font* :type rope)
  (thickness 1)
  (offset 0d0))

(defun scene-text (value &key (thickness 1))
  (make-scene-text :value value
                   :thickness thickness))

(defparameter *draw-options*
  '((:no-shadow . nil)
    (:color . (0 0 0))
    (:specular . (0 0 0))
    (:alpha . 1d0)
    (:visual . t)
    (:collision . t)
    (:type . nil)))

(defun draw-option (options key)
  (alist-get-default options key *draw-options*))

(defun draw-options-default (&key
                               (options *draw-options*)
                               (no-shadow (draw-option options :no-shadow))
                               (specular (draw-option options :specular))
                               (color (draw-option options :color))
                               (alpha (draw-option options :alpha))
                               (visual (draw-option options :visual))
                               (type (draw-option options :type))
                               (collision (draw-option options :collision)))
  (list* (cons :no-shadow no-shadow)
         (cons :color color)
         (cons :specular specular)
         (cons :alpha alpha)
         (cons :visual visual)
         (cons :collision collision)
         (cons :type type)
         options))

(defun draw-options (&rest options-plist)
  (plist-alist options-plist))

(defun merge-draw-options (new-options &optional (base-options *draw-options*))
  (append new-options base-options))




(defstruct scene-geometry
  shape
  options
  type
  (collision t)
  (visual t))

(defun scene-geometry (shape options)
  (make-scene-geometry :shape shape
                       :options options
                       :type (let ((type (draw-option options :type)))
                               (etypecase type
                                 (list (apply #'tree-set #'rope-compare-fast type))
                                 (tree-set type)
                                 (rope (tree-set #'rope-compare-fast type))))
                       :collision (draw-option options :collision)
                       :visual (draw-option options :visual)))

(defun scene-geometry-isa (geometry type)
  (let ((tree (scene-geometry-type geometry)))
    (when tree (tree-set-member-p tree type))))

;;;;;;;;;;;;;;
;;; FRAMES ;;;
;;;;;;;;;;;;;;

(deftype frame-name ()
  `rope)

(defun frame-name-compare (a b)
  (rope-compare-lexographic a b))

(defstruct frame-inertial
  (mass 0d0 :type double-float)
  (inertia nil))

(defstruct scene-frame
  "Base struct for frames in the scene."
  (name nil :type frame-name)
  (parent nil :type frame-name)
  (inertial nil :type (or null frame-inertial))
  (tf +tf-ident+ :type tf)
  (geometry nil :type list))

(defun scene-frame-geometry-isa (frame type)
  (find-if (lambda (g)
             (scene-geometry-isa g type))
           (scene-frame-geometry frame)))

(defun scene-frame-geometry-collision (frame)
  (loop for g in (scene-frame-geometry frame)
     when (scene-geometry-collision g)
     collect g))

(defstruct (scene-frame-fixed (:include scene-frame))
  "A frame with a fixed transformation")

(defun scene-frame-fixed (parent name &key
                                        geometry
                                        inertial
                                        (tf (identity-tf)))
  "Create a new fixed frame."
  (make-scene-frame-fixed :name name
                          :parent parent
                          :tf tf
                          :inertial inertial
                          :geometry (ensure-list geometry)))


(deftype configuration-map ()
  `tree-map)

(defun configuration-map-equal (a b)
  (let ((a (tree-map-alist a))
        (b (tree-map-alist b)))
    (when (= (length a)
             (length b))
      (loop
         for x in a
         for y in b
         do (unless (and (zerop (frame-name-compare (car x) (car y)))
                         (= (cdr x) (cdr y)))
              (return-from configuration-map-equal nil)))
      t)))


(defun prefix-configuration-map (prefix map)
  (fold-tree-map (lambda (map key value)
                   (tree-map-insert map (rope prefix key) value))
                 (make-tree-map #'frame-name-compare)
                 map))

(defstruct joint-limit
  (min 0 :type double-float)
  (max 0 :type double-float))

(defstruct joint-limits
  (position nil :type (or null joint-limit))
  (velocity nil :type (or null joint-limit))
  (acceleration nil :type (or null joint-limit))
  (effort nil :type (or null joint-limit)))

(defstruct (scene-frame-joint (:include scene-frame))
  "Base struct for varying scene frames."
  (configuration-name nil :type frame-name)
  (configuration-offset 0d0 :type double-float)
  (axis (vec3* 0d0 0d0 1d0) :type vec3)
  (limits nil :type (or null joint-limits)))

(defstruct (scene-frame-revolute (:include scene-frame-joint))
  "A frame representing a revolute (rotating) joint.")

(defun joint-limit (&key value min max)
  (cond
    ;; no limit
    ((and (null min)
          (null max)
          (null value))
     nil)
    ;; explit min/max
    ((and min max (null value))
     (assert (null value))
     (make-joint-limit :min min :max max))
    ;; something else
    ((and (null min)
          (null max)
          value)
     (etypecase value
       (number
        (make-joint-limit :min (- value) :max (+ value)))
       (joint-limit value)))
    ;; bad args
    (t
     (error "invalid joint-limit arguments"))))

(defun joint-limits (&key
                       effort-limit min-effort max-effort
                       acceleration-limit min-acceleration max-acceleration
                       velocity-limit min-velocity max-velocity
                       position-limit min-position max-position)
  (make-joint-limits
   :position (joint-limit :value position-limit :min min-position :max max-position)
   :velocity (joint-limit :value velocity-limit :min min-velocity :max max-velocity)
   :acceleration (joint-limit :value acceleration-limit :min min-acceleration :max max-acceleration)
   :effort (joint-limit :value effort-limit :min min-effort :max max-effort)))

(defun scene-frame-revolute (parent name &key
                                           axis
                                           (configuration-name name)
                                           (configuration-offset 0d0)
                                           (tf (identity-tf))
                                           limits)
  "Create a new revolute frame."
  (assert axis)
  (make-scene-frame-revolute :name name
                             :configuration-name configuration-name
                             :configuration-offset (coerce configuration-offset 'double-float)
                             :parent parent
                             :limits limits
                             :axis (vec3 axis)
                             :tf (tf tf)))

(defstruct (scene-frame-prismatic (:include scene-frame-joint))
  "A frame representing a prismatic (sliding) joint.")

(defun scene-frame-prismatic (parent name &key
                                            axis
                                            (configuration-name name)
                                            (configuration-offset 0d0)
                                            limits
                                            (tf (identity-tf)))
  "Create a new prismatic frame."
  (assert axis)
  (make-scene-frame-prismatic :name name
                              :configuration-name configuration-name
                              :configuration-offset (coerce configuration-offset 'double-float)
                              :parent parent
                              :axis (vec3 axis)
                              :limits limits
                              :tf (tf tf)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; SCENE GRAPH STRUCTURE ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; TODO: dot output

(defun scene-frame-compare (frame-a frame-b)
  (labels ((name (frame)
             (etypecase frame
               (rope frame)
               (scene-frame (scene-frame-name frame)))))
    (declare (dynamic-extent #'name))
    (frame-name-compare (name frame-a)
                        (name frame-b))))

(defstruct scene-graph
  (frames (make-tree-set #'scene-frame-compare)))

;;; Basic Operations ;;;

(defun scene-graph-lookup (scene-graph frame-name)
  "Find a frame"
  (tree-set-find (scene-graph-frames scene-graph)
                 frame-name))

(defun %scene-graph-add-frame (scene-graph frame)
  "Add a frame to the scene."
  (make-scene-graph :frames (tree-set-replace (scene-graph-frames scene-graph)
                                              frame)))

(defun scene-graph-remove-frame (scene-graph frame-name)
  "Remove a frame from the scene."
  (make-scene-graph :frames (tree-set-remove (scene-graph-frames scene-graph)
                                             frame-name)))

(defun scene-graph-add-geometry (scene-graph frame-name geometry)
  "Bind collision geometry to a frame in the scene."
  (let ((frame (copy-structure (scene-graph-lookup scene-graph frame-name))))
    (push geometry (scene-frame-geometry frame))
    (%scene-graph-add-frame scene-graph frame)))

(defun scene-graph-set-inertial (scene-graph frame-name
                                 &key
                                   (mass 0d0)
                                   inertia)
  "Bind collision geometry to a frame in the scene."
  (let ((frame (copy-structure (scene-graph-lookup scene-graph frame-name))))
    (setf (scene-frame-inertial frame)
          (make-frame-inertial :mass mass
                               :inertia inertia))
    (%scene-graph-add-frame scene-graph frame)))

(defun %scene-graph-merge (scene-graph-1 scene-graph-2)
  "Combine two scene graphs."
  (let* ((set-1 (scene-graph-frames scene-graph-1))
         (set-2 (scene-graph-frames scene-graph-2))
         (intersection (tree-set-intersection set-1 set-2)))
    (assert (zerop (tree-set-count intersection)) ()
            "Duplicate frames in merged trees: ~A"
            (map-tree-set 'list #'scene-frame-name intersection))
    (make-scene-graph :frames (tree-set-union set-1 set-2))))


(defvar *scene-directory* (make-pathname))

(defun %scene-graph (things)
  (labels ((load-thing (thing)
             (load-scene-file thing))
              ;; (let ((dir (pathname-directory thing)))
              ;;   (if (and dir (eq :absolute (car dir)))
              ;;       thing
              ;;       (merge-pathnames *scene-directory* thing))))
           (rec (scene-graph thing)
             (etypecase thing
               (scene-frame
                (%scene-graph-add-frame scene-graph thing))
               (scene-graph
                (%scene-graph-merge scene-graph thing))
               (list
                (%scene-graph-merge scene-graph
                                    (%scene-graph thing)))
               ((or pathname string)
                (%scene-graph-merge scene-graph
                                    (load-thing thing)))
               (rope
                (rec scene-graph (rope-string thing))))))
    (fold #'rec (make-scene-graph) things)))

(defun scene-graph (&rest things)
  (%scene-graph things))

(defmacro scene-graph-f (place &rest things)
  `(setq ,place
         (scene-graph ,place ,@things)))

(defmacro do-scene-graph-frames ((frame-variable scene-graph &optional result) &body body)
  `(do-tree-set (,frame-variable (scene-graph-frames ,scene-graph) ,result)
     ,@body))

(defun map-scene-graph-frames (result-type function scene-graph)
  (map-tree-set result-type function (scene-graph-frames scene-graph)))


(defmacro do-scene-graph-geometry (((frame geometry) scene-graph &optional result)
                                   &body body)
  (with-gensyms (fun)
    `(flet ((,fun (,frame ,geometry)
              ,@body))
       (declare (dynamic-extent #',fun))
       (do-scene-graph-frames (,frame ,scene-graph ,result)
         (loop
            for ,geometry in (scene-frame-geometry ,frame)
            do (,fun ,frame ,geometry))))))


(defun map-scene-graph-geometry (result-type function scene-graph)
  (cond
    ((null result-type)
     (do-scene-graph-geometry ((frame geometry) scene-graph)
       (funcall function frame geometry)))

    ((eq 'list result-type)
     (apply #'nconc
            (map-scene-graph-frames result-type
                                    (lambda (f)
                                      (loop for g in (scene-frame-geometry f)
                                         collect (funcall function f g)))
                                    scene-graph)))
    (t (error "Unsupported result-type ~A" result-type))))


(defun scene-graph-meshes (scene-graph)
  (let ((meshes (make-hash-table :test #'equal)))
    ;; hash meshes
    (do-scene-graph-geometry ((f g) scene-graph
                              (hash-table-values meshes))
      (declare (ignore f))
      (let ((shape (scene-geometry-shape g)))
        (when (scene-mesh-p shape)
          (let ((name (rope-string (scene-mesh-name shape))))
            (if-let ((other-mesh (gethash name meshes)))
              ;; check matching data
              (assert (equal (scene-mesh-source-file shape)
                             (scene-mesh-source-file other-mesh)))
              ;; add to hash table
              (setf (gethash name meshes) shape))))))))

(defmacro do-scene-graph-geometry-types (((frame geometry) (scene-graph type) &optional result)
                                         &body body)
  (with-gensyms (fun typevar)
    `(flet ((,fun (,frame ,geometry)
              ,@body))
       (declare (dynamic-extent #',fun))
       (let ((,typevar ,type))
         (do-scene-graph-geometry ((,frame ,geometry) ,scene-graph ,result)
           (when (scene-geometry-isa ,geometry ,typevar)
             (,fun ,frame ,geometry)))))))


(defun fold-scene-graph-frames (function initial-value scene-graph)
  (fold-tree-set function initial-value (scene-graph-frames scene-graph)))

(defun prefix-scene-graph (prefix scene-graph &key
                                                (prefix-parents t))
  (let ((frames))
    (do-scene-graph-frames (frame (scene-graph scene-graph) (scene-graph frames))
      (let ((frame (copy-structure frame))
            (parent (scene-frame-parent frame)))
        (setf (scene-frame-name frame)
              (rope prefix (scene-frame-name frame)))
        (when (and parent prefix-parents)
          (setf (scene-frame-parent frame)
                (rope prefix parent)))
        (when (scene-frame-joint-p frame)
          (setf (scene-frame-joint-configuration-name frame)
                (rope prefix (scene-frame-joint-configuration-name frame))))
        (push frame frames)))))

(defun scene-graph-joints (scene-graph)
  (fold-scene-graph-frames (lambda (list frame)
                             (etypecase frame
                               (scene-frame-fixed list)
                               ((or scene-frame-prismatic scene-frame-revolute)
                                (cons (scene-frame-name frame) list))))
                           nil
                           scene-graph))

(defun scene-mesh-inc (mesh-file)
  "Return the include file for the mesh file"
  (pov-cache-file (rope mesh-file ".inc")))

(defun scene-graph-resolve-mesh (scene-graph &key
                                               reload
                                               (mesh-up-axis "Z")
                                               (mesh-forward-axis "Y")
                                               (directory *robray-tmp-directory*))
  ;(print 'resolve-mesh)
  (let ((mesh-files  ;; filename => (list mesh-nodes)
         (make-hash-table :test #'equal)))
    (labels ((resolve-mesh (mesh)
               (when-let ((source (and (not (scene-mesh-povray-file mesh))
                                       (scene-mesh-source-file  mesh))))
                 (push mesh (gethash source mesh-files))))
             (test-shape (shape)
               (when (and shape (scene-mesh-p shape))
                 (resolve-mesh shape))))
      ;; collect mesh files
      (do-scene-graph-geometry ((frame geometry) scene-graph)
        (declare (ignore frame))
        (test-shape (scene-geometry-shape geometry))))
    (maphash (lambda (mesh-file mesh-nodes)
               ;(format *standard-output* "~&Converting ~A..." mesh-file)
               (multiple-value-bind (geom-name inc-file)
                   (mesh-povray mesh-file :directory directory
                                :reload reload
                                :mesh-up-axis mesh-up-axis
                                :mesh-forward-axis mesh-forward-axis)

               ;; (let* ((inc-file (output-file (scene-mesh-inc mesh-file) directory))
               ;;        (convert (or reload
               ;;                     (not (probe-file inc-file))
               ;;                     (>= (file-write-date mesh-file)
               ;;                         (file-write-date inc-file))))
               ;;        (geom-name (progn (ensure-directories-exist inc-file)
               ;;                          (collada-povray mesh-file
               ;;                                          (when convert inc-file)))))
               ;;   (if convert
               ;;       (format *standard-output* "~&    to  ~A..." inc-file)
               ;;       (format *standard-output* "~&    cached"))

                 (dolist (mesh-node mesh-nodes)
                   (setf (scene-mesh-name mesh-node) geom-name
                         (scene-mesh-povray-file mesh-node) inc-file))))
             mesh-files)
    scene-graph))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Computing Transforms ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun alist-configuration-map (alist &optional default-map)
  (fold (lambda (map item)
          (tree-map-insert map (car item) (cdr item)))
        (or default-map (make-tree-map #'frame-name-compare))
        alist))

(defun pairlist-configuration-map (names values &optional default-map)
  (alist-configuration-map (pairlis names values) default-map))


(defun scene-frame-configuration (frame configuration-map default-configuration)
  (let ((variable (tree-map-find configuration-map
                                 (scene-frame-joint-configuration-name frame)
                                 default-configuration))
        (offset (scene-frame-joint-configuration-offset frame)))
    (+  variable offset)))

(defun scene-frame-tf-local (frame configuration-map default-configuration)
  "Find the local TF from its parent to FRAME"
  (etypecase frame
    (scene-frame-fixed (scene-frame-fixed-tf frame))
    (scene-frame-joint
     (assert configuration-map ()
             "Cannot find joint frames without configuration variables")
     (let ((config (scene-frame-configuration frame configuration-map default-configuration))
           (axis (scene-frame-joint-axis frame)))
       (tf-mul (scene-frame-tf frame)
               (etypecase frame
                 (scene-frame-revolute
                  (tf* (with-vec3 (x y z) axis
                         (axis-angle* x y z config))
                       (identity-vec3)))
                 (scene-frame-prismatic
                  (tf* (identity-quaternion)
                       (g* config axis)))))))))


;; (defun scene-graph-tf-relative (scene-graph frame-name
;;                                 &key
;;                                   configuration-map
;;                                   (default-configuration 0d0))
;;   (scene-frame-tf-relative (scene-graph-lookup scene-graph frame-name)
;;                            configuration-map
;;                            default-configuration))

;; (defun scene-graph-tf-relative-map (scene-graph configuration-map
;;                                     &key (default-configuration 0d0))
;;   (fold-scene-graph-frames (lambda (hash frame)
;;                              (setf (gethash (scene-frame-name frame) hash)
;;                                    (scene-graph-tf-relative scene-graph (scene-frame-name frame)
;;                                                             :configuration-map configuration-map
;;                                                             :default-configuration default-configuration))
;;                              hash)
;;                            (make-hash-table :test #'equal)
;;                            scene-graph))

(defun scene-graph-tf-absolute (scene-graph frame-name
                                &key
                                  configuration-map
                                  (tf-absolute-map (make-string-hash-table))
                                  (default-configuration 0d0))
  (declare (type hash-table tf-absolute-map))
  (labels ((rec (frame-name)
             (cond
               ((null frame-name) ; global frame
                (tf nil))
               ((gethash frame-name tf-absolute-map) ; already in hash
                (gethash frame-name tf-absolute-map))
               (t
                (let ((frame (scene-graph-lookup scene-graph frame-name)))
                  (assert frame () "Frame ~A not found in scene graph"
                          frame-name)
                  (let* ((parent-name (scene-frame-parent frame))
                         (tf-frame (scene-frame-tf-local frame
                                                         configuration-map
                                                         default-configuration)))
                    (if parent-name
                        (normalize (g* (rec (scene-frame-parent frame))
                                       tf-frame))
                        tf-frame)))))))
    (rec frame-name)))


(defun scene-graph-tf-relative (scene-graph parent child
                                &key
                                  configuration-map
                                  (default-configuration 0d0)
                                  (tf-absolute-map (make-string-hash-table)))
  "Find the relative TF from PARENT to CHILD"
  (let ((g-tf-parent(scene-graph-tf-absolute scene-graph parent
                                             :configuration-map configuration-map
                                             :tf-absolute-map tf-absolute-map
                                             :default-configuration default-configuration))
        (g-tf-child (scene-graph-tf-absolute scene-graph child
                                             :configuration-map configuration-map
                                             :tf-absolute-map tf-absolute-map
                                             :default-configuration default-configuration)))
    (tf-mul (tf-inverse g-tf-parent)
            g-tf-child)))



(defun scene-graph-tf-absolute-map (scene-graph
                                    &key
                                      configuration-map
                                      (default-configuration 0d0))
  (labels ((rec (tf-abs frame)
             (let ((name (scene-frame-name frame)))
               (setf (gethash name tf-abs)
                     (scene-graph-tf-absolute scene-graph name
                                              :configuration-map configuration-map
                                              :tf-absolute-map tf-abs
                                              :default-configuration default-configuration)))
             tf-abs))
    (fold-scene-graph-frames #'rec (make-hash-table :test #'equal) scene-graph)))


(defun scene-graph-reparent (scene-graph new-parent frame-name
                             &key
                               tf
                               (tf-absolute-map (make-string-hash-table))
                               configuration-map
                               (default-configuration 0d0))
  "Change the parent of frame FRAME-NAME to NEW-PARENT"
  (declare (type frame-name new-parent frame-name))
  (let ((tf (if tf
                tf
                (scene-graph-tf-relative scene-graph new-parent frame-name
                                         :configuration-map configuration-map
                                         :default-configuration default-configuration
                                         :tf-absolute-map tf-absolute-map)))
        (old-frame (scene-graph-lookup scene-graph frame-name)))
    (check-type old-frame scene-frame-fixed)
    (let ((new-frame (copy-scene-frame-fixed old-frame)))
      (setf (scene-frame-parent new-frame) new-parent
            (scene-frame-fixed-tf new-frame) tf)
      (%scene-graph-add-frame (scene-graph-remove-frame scene-graph frame-name)
                             new-frame))))


(defun scene-graph-add-tf (scene-graph tf-tag &key
                                                geometry
                                                (actual-parent (tf-tag-parent tf-tag))
                                                (tf-absolute-map (make-string-hash-table))
                                                configuration-map
                                                (default-configuration 0d0))
  "Add a fixed frame to the scene."
  (let* ((parent (tf-tag-parent tf-tag))
         (tf-rel (tf-tag-tf tf-tag))
         (name (tf-tag-child tf-tag))
         (tf (if (eq parent actual-parent)
                 tf-rel
                 (tf-mul (scene-graph-tf-relative scene-graph actual-parent parent
                                                  :configuration-map configuration-map
                                                  :default-configuration default-configuration
                                                  :tf-absolute-map tf-absolute-map)
                         tf-rel))))
    (%scene-graph-add-frame scene-graph (scene-frame-fixed actual-parent name
                                                          :tf tf
                                                          :geometry geometry))))



(defun scene-graph-configurations (scene-graph)
  "Returns the configuration variables in the scene graph."
  (fold-scene-graph-frames (lambda (list frame)
                             (if (scene-frame-joint-p frame)
                                 (cons (scene-frame-name frame) list)
                                 list))
                           nil
                           scene-graph))



;;;;;;;;;;;;;;
;;; POVRAY ;;;
;;;;;;;;;;;;;;

(defun scene-visual-pov (geometry tf)
  (declare (type scene-geometry geometry))
  (let* ((options (scene-geometry-options geometry))
         (modifiers (list tf))
         (shape (scene-geometry-shape geometry)))
    (when (and options
               ;; Don't clobber mesh texture
               ;; TODO: there's probably a better way to do this
               (not (scene-mesh-p shape)))
      (push (pov-alist-texture options)
            modifiers))
    (when (draw-option options :no-shadow)
      (push '|no_shadow| modifiers))
    (let ((scale (alist-get-default options :scale nil)))
      (when scale (push (pov-item "scale" scale)
                        modifiers)))
    (etypecase shape
      (scene-text (pov-text (scene-text-value shape)
                            :font (scene-text-font shape)
                            :thickness (scene-text-thickness shape)
                            :modifiers modifiers))
      (scene-mesh (pov-mesh2 :mesh (scene-mesh-name shape)
                             :modifiers modifiers))
      (scene-grid (pov-mesh2 :mesh-data (grid-mesh shape)
                             :modifiers modifiers))
      (scene-box
       (pov-box-center (scene-box-dimension shape)
                       :modifiers modifiers))
      (scene-cylinder
       (pov-cylinder-axis (vec 0 0 (scene-cylinder-height shape))
                          (scene-cylinder-radius shape)
                          modifiers))
      (scene-cone
       (pov-cone-axis (vec 0 0 (scene-cone-height shape))
                      (scene-cone-start-radius shape)
                      (scene-cone-end-radius shape)
                      modifiers))
      (scene-sphere
       (pov-sphere (identity-vec3)
                   (scene-sphere-radius shape)
                   modifiers)))))


(defun grid-mesh (grid)
  (let ((dim (scene-grid-dimension grid))
        (delta (scene-grid-delta grid))
        (thickness (scene-grid-thickness grid))
        (verts (make-array 0 :fill-pointer t :adjustable t))
        (indices (make-array 0 :fill-pointer t :adjustable t)))
    (labels ((helper (x0 x-max y delta vec-fun)
               (loop
                  with i-begin = (length verts)
                  with y-1 = (+ y (/ thickness 1))
                  with y-0 = (- y-1)
                  for x-c = x0 then (+ x-c delta)
                  while (<= (abs x-c) x-max)
                  for i0 from i-begin by 4
                  for i1 from (+ i-begin 1) by 4
                  for i2 from (+ i-begin 2) by 4
                  for i3 from (+ i-begin 3) by 4
                  for x-0 = (- x-c thickness)
                  for x-1 = (+ x-c thickness)
                  do (progn
                       (vector-push-extend (funcall vec-fun x-0 y-0 0) verts)
                       (vector-push-extend (funcall vec-fun x-0 y-1 0) verts)
                       (vector-push-extend (funcall vec-fun x-1 y-0 0) verts)
                       (vector-push-extend (funcall vec-fun x-1 y-1 0) verts)
                       (vector-push-extend (vector i0 i1 i2) indices)
                       (vector-push-extend (vector i3 i1 i2) indices)))))
      (helper 0 (vec-x dim) (vec-y dim) (vec-x delta) #'vec3*)
      (helper (- (vec-x delta)) (vec-x dim) (vec-y dim) (- (vec-x delta)) #'vec3*)
      (helper 0 (vec-y dim) (vec-x dim) (vec-y delta) (lambda (y x z) (vec3* x y z)))
      (helper (- (vec-y delta)) (vec-y dim) (vec-x dim) (- (vec-y delta)) (lambda (y x z) (vec3* x y z)))
      (make-mesh-data :vertex-vectors (vec-flatten verts)
                      :vertex-indices (fnvec-flatten indices)))))

(defun scene-graph-pov-frame (scene-graph
                              &key
                                render
                                (options (render-options-default))
                                configuration-map
                                output
                                (directory *robray-tmp-directory*)
                                include
                                (default-configuration 0d0))
"Generate the POV-ray scene for the given scene-graph."
  (let ((pov-things)
        (tf-abs (scene-graph-tf-absolute-map scene-graph
                                             :configuration-map configuration-map
                                             :default-configuration default-configuration))
        (mesh-set (make-tree-set #'string-compare))
        (use-collision (get-render-option options :use-collision)))
    ;(format t "~& col: ~A" use-collision)
    (labels ((thing (thing)
               (push thing pov-things))
             (include (file)
               (thing (pov-include file))))
      ;; push frame geometry
      (do-scene-graph-geometry ((frame geometry) scene-graph)
        ;(format t "~&converting frame: ~A" (scene-frame-name frame))
        (when (if use-collision
                  (scene-geometry-collision geometry)
                  (scene-geometry-visual geometry))
          (let ((shape (scene-geometry-shape geometry))
                (name (scene-frame-name frame)))
            (thing (scene-visual-pov geometry
                                     (pov-transform* (pov-matrix (gethash name tf-abs)))))
            (thing (pov-line-comment (format nil "FRAME: ~A" name)))
            (when (scene-mesh-p shape)
              (setf (tree-set-find mesh-set)
                    (scene-mesh-povray-file shape))))))
      (map-tree-set nil #'include mesh-set)
      (map nil #'include (reverse (ensure-list include)))
      ;; version
      (thing (pov-version "3.7")))

    ;; result
    (let ((result (pov-sequence pov-things)))
      (cond (render
             (pov-render result
                         :file output
                         :directory directory
                                :options options)
             nil)
            (output
             (output result
                     output
                     :directory directory)
             nil)
            (t result)))))
