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
  data
  source-file
  povray-file)

(defparameter *scene-font* :monospace
  "Default font for scene text.")

(defstruct scene-text
  "A scene object for text."
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
    (:metallic nil)
    (:visual . t)
    (:collision . t)
    (:scale . 1)
    (:override-texture . nil)
    (:type . nil))
  "Association list of default drawing options for geometry.")

(defun draw-option (options key)
  "Return the draw option in OPTIONS for KEY."
  (alist-get-default options key *draw-options*))

(defun octet-color (r g b)
  (list (/ r 255.0)
        (/ g 255.0)
        (/ b 255.0)))

(defmacro push-option (keyword value supplied place)
  `(when ,supplied
     (push (cons ,keyword ,value) ,place)))

(defun draw-options-default (&key
                               (options *draw-options*)
                               (scale nil scale-supplied)
                               (no-shadow nil no-shadow-supplied)
                               (specular nil specular-supplied)
                               (color nil color-supplied)
                               (alpha nil alpha-supplied)
                               ;; povray options
                               (metallic nil metallic-supplied)
                               (brilliance nil brilliance-supplied)
                               (diffuse nil diffuse-supplied)
                               (roughness nil roughness-supplied)
                               (reflection nil reflection-supplied)
                               (ambient nil ambient-supplied)
                               (crand nil crand-supplied)
                               (override-texture nil override-texture-supplied)
                               ;; collision vs. visual
                               (visual nil visual-supplied)
                               (type nil type-supplied)
                               (collision nil collision-supplied))
  "Construct a drawing options set using default values from OPTIONS."
  (push-option :scale scale scale-supplied options)
  (push-option :no-shadow no-shadow no-shadow-supplied options)
  (push-option :specular specular specular-supplied options)
  (push-option :color color color-supplied options)
  (push-option :alpha alpha alpha-supplied options)

  (push-option :metallic metallic metallic-supplied options)
  (push-option :brilliance brilliance brilliance-supplied options)
  (push-option :diffuse diffuse diffuse-supplied options)
  (push-option :roughness roughness roughness-supplied options)
  (push-option :reflection reflection reflection-supplied options)
  (push-option :ambient ambient ambient-supplied options)
  (push-option :crand crand crand-supplied options)
  (push-option :override-texture override-texture override-texture-supplied options)

  (push-option :visual visual visual-supplied options)
  (push-option :collision collision collision-supplied options)
  (push-option :type type type-supplied options)
  options)

(defun draw-options (&rest options-plist)
  "Construct drawing options from a propetry list."
  (plist-alist options-plist))

(defun merge-draw-options (new-options &optional (base-options *draw-options*))
  "Merge two drawing options.

Values in NEW-OPTIONS supersede values in BASE-OPTIONS."
  (append new-options base-options))

(defmacro push-draw-option (name value place)
  `(push (cons ,name ,value) ,place))

(defstruct scene-geometry
  "Container for geometry attached to scene frames."
  shape
  options
  type
  (collision t)
  (visual t)
  c-geom)

(defun %scene-geometry (shape options &optional (c-geom))
  (make-scene-geometry :shape shape
                       :options options
                       :type (let ((type (draw-option options :type)))
                               (etypecase type
                                 (list (apply #'tree-set #'rope-compare-fast type))
                                 (tree-set type)
                                 (rope (tree-set #'rope-compare-fast type))))
                       :collision (draw-option options :collision)
                       :visual (draw-option options :visual)
                       :c-geom c-geom))

(defun %rx-scene-geometry (rx-geom options)
  (%scene-geometry (rx-geom-shape rx-geom) options rx-geom))

(defun modify-scene-geometry (geom options)
  ;(print geom)
  (let ((c-geom (aa-rx-geom-modify-opt (scene-geometry-c-geom geom)
                                       (alist-rx-geom-opt options)))
        (shape (scene-geometry-shape geom)))
    (cond
      ((amino-ffi::foreign-container-p shape)
       (%rx-scene-geometry c-geom options))
      ((scene-mesh-p shape)
       (%scene-geometry shape options c-geom))
      (t (assert nil)))))



(defun scene-geometry-box (options dimension)
  "Create geometry for a box.

OPTIONS: drawing options for the box
DIMESIONS: length 3 vector or sequence giving x, y, and z dimensions."
  (%rx-scene-geometry (aa-rx-geom-box (alist-rx-geom-opt options)
                                      (vec3 dimension))
                      options))

(defun scene-geometry-sphere (options radius)
  "Create geometry for a sphere."
  (%rx-scene-geometry (aa-rx-geom-sphere (alist-rx-geom-opt options)
                                         radius)
                      options))

(defun scene-geometry-cylinder (options &key radius height)
  "Create geometry for a cylinder."
  (%rx-scene-geometry (aa-rx-geom-cylinder (alist-rx-geom-opt options)
                                           height radius)
                   options))

(defun scene-geometry-cone (options &key height start-radius end-radius)
  "Create geometry for a cone."
  (%rx-scene-geometry (aa-rx-geom-cone (alist-rx-geom-opt options)
                                       height
                                       start-radius
                                       end-radius)
                   options))

(defun scene-geometry-grid (options &key dimension delta width)
  "Create geometry for a grid."
  (%rx-scene-geometry (aa-rx-geom-grid (alist-rx-geom-opt options)
                                       (ensure-vec dimension)
                                       (ensure-vec delta)
                                       width)
                   options))

(defun scene-geometry-torus (options &key major-radius minor-radius (angle (* 2 pi)))
  "Create geometry for a torus.

OPTIONS: drawing options for the box
DIMESIONS: length 3 vector or sequence giving x, y, and z dimensions."
  (%rx-scene-geometry (aa-rx-geom-torus (alist-rx-geom-opt options)
                                        angle
                                        major-radius
                                        minor-radius)

                      options))

(defun scene-geometry-text (options text &key (thickness 1))
  "Create geometry for text."
  (%scene-geometry (scene-text text :thickness thickness)
                   options))

(defun scene-geometry-mesh (options mesh)
  "Create geometry for a mesh."
  (%scene-geometry (etypecase mesh
                     (scene-mesh mesh)
                     ((or pathname string)
                      (make-scene-mesh :source-file mesh)))
                   options))

(defun scene-geometry-isa (geometry type)
  (let ((tree (scene-geometry-type geometry)))
    (when tree (tree-set-member-p tree type))))


(defun scene-geometry-cylinder-p (geometry)
  (when (scene-geometry-p geometry)
    (scene-cylinder-p (scene-geometry-shape geometry))))

(defun scene-geometry-cylinder-height (geometry)
  (scene-cylinder-height (scene-geometry-shape geometry)))

;;;;;;;;;;;;;;;;;;;;
;;; Scene Object ;;;
;;;;;;;;;;;;;;;;;;;;

(deftype scene-object-name ()
  `rope)

(defstruct scene-object
  (name nil :type config-name))

(defun scene-object-name-compare (a b)
  (rope-compare-lexographic a b))

(defun scene-object-compare (obj-a obj-b)
  (labels ((name (obj)
             (etypecase obj
               (rope obj)
               (scene-object (scene-object-name obj)))))
    (declare (dynamic-extent #'name))
    (scene-object-name-compare (name obj-a)
                               (name obj-b))))

;;;;;;;;;;;;;;;;;;;;;;
;;; Configurations ;;;
;;;;;;;;;;;;;;;;;;;;;

(deftype config-name ()
  `scene-object-name)

(defun config-name-compare (a b)
  (scene-object-name-compare a b))

(defstruct joint-limit
  (min 0 :type double-float)
  (max 0 :type double-float))

(defstruct joint-limits
  (position nil :type (or null joint-limit))
  (velocity nil :type (or null joint-limit))
  (acceleration nil :type (or null joint-limit))
  (effort nil :type (or null joint-limit)))

(defstruct (scene-config (:include scene-object))
  (limits nil :type (or null joint-limits)))

;;;;;;;;;;;;;;
;;; FRAMES ;;;
;;;;;;;;;;;;;;

(deftype frame-name ()
  `scene-object-name)

(defun frame-name-compare (a b)
  (scene-object-name-compare a b))

(defun frame-name-global-p (frame-name)
  (or (null frame-name)
      (zerop (length (rope-string frame-name)))))

(defstruct frame-inertial
  (mass 0d0 :type double-float)
  (inertia nil))

(defstruct (scene-frame (:include scene-object))
  "Base struct for frames in the scene."
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

(define-condition oedipus-frame-error (error)
    ((name :initarg :name)))

(defun check-frame-parent (parent name)
  (when (zerop (rope-compare-fast parent name))
    (error 'oedipus-frame-error :name (rope-string name))))

(defmethod print-object ((object oedipus-frame-error) stream)
  (format stream "Frame `~A' cannot be its own parent."
          (slot-value object 'name)))

(define-condition frame-exist-error (error)
    ((name :initarg :name)))

(defmethod print-object ((object frame-exist-error) stream)
  (format stream "Frame `~A' does not exist."
          (slot-value object 'name)))

(defun check-frame-exists (frame scene-graph)
  (unless (or (null frame)
              (zerop (length frame))
              (scene-graph-lookup scene-graph frame))
    (error 'frame-exist-error :name frame)))

(defun scene-frame-fixed (parent name &key
                                        geometry
                                        inertial
                                        (tf (identity-tf)))
  "Create a new fixed frame."
  (check-frame-parent parent name)
  (make-scene-frame-fixed :name name
                          :parent parent
                          :tf tf
                          :inertial inertial
                          :geometry (ensure-list geometry)))


(deftype configuration-map ()
  `tree-map)

(defun make-configuration-map ()
  (make-tree-map #'frame-name-compare))

(defun configuration-map-merge (original-map update-map)
  (tree-map-insert-map original-map update-map))

(defun configuration-map-pairs (names values)
  (let ((result (make-configuration-map)))
    (map nil (lambda (name value)
               (tree-map-insertf result name value))
         names values)
    result))

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

(defun configuration-map-find (config-map name &optional (default 0d0))
  (tree-map-find config-map name default))

(defun prefix-configuration-map (prefix map)
  (fold-tree-map (lambda (map key value)
                   (tree-map-insert map (rope prefix key) value))
                 (make-configuration-map)
                 map))


(defstruct (scene-frame-joint (:include scene-frame))
  "Base struct for varying scene frames."
  (configuration-name nil :type frame-name)
  (configuration-offset 0d0 :type double-float)
  (axis (vec3* 0d0 0d0 1d0) :type vec3)
  (%limits nil :type (or null joint-limits)))

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
        (make-joint-limit :min (coerce (- value) 'double-float) :max (coerce (+ value) 'double-float)))
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
                                           geometry
                                           limits)
  "Create a new revolute frame."
  (assert axis)
  (check-frame-parent parent name)
  (make-scene-frame-revolute :name name
                             :geometry (ensure-list geometry)
                             :configuration-name configuration-name
                             :configuration-offset (coerce configuration-offset 'double-float)
                             :parent parent
                             :%limits limits
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
  (check-frame-parent parent name)
  (make-scene-frame-prismatic :name name
                              :configuration-name configuration-name
                              :configuration-offset (coerce configuration-offset 'double-float)
                              :parent parent
                              :axis (vec3 axis)
                              :%limits limits
                              :tf (tf tf)))


;;;;;;;;;;;;;;;;;;;;;
;;; Collision Sets ;;
;;;;;;;;;;;;;;;;;;;;;
(deftype collision-set ()
  `tree-set)

(defun make-collision-set ()
  (make-tree-set #'compare-allowed-collision))

(defun collision-set-insert (collision-set frame-1 frame-2)
  (multiple-value-bind (frame-1 frame-2)
      (cond-compare (frame-1 frame-2 #'frame-name-compare)
                    (values frame-1 frame-2)
                    (values frame-1 frame-2)
                    (values frame-2 frame-1))
    (tree-set-insert collision-set
                     (cons frame-1 frame-2))))


(defun map-collision-set (result-type function collision-set)
  (map-tree-set result-type (lambda (c)
                              (funcall function (car c) (cdr c)))
                collision-set))

(defmacro do-collision-set (((frame-1 frame-2) set &optional result)
                            &body
                              body)
  (with-gensyms (c)
    `(do-tree-set (,c ,set ,result)
       (let ((,frame-1 (car ,c))
             (,frame-2 (cdr ,c)))
         ,@body))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; SCENE GRAPH STRUCTURE ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; TODO: Make separate config set

(defun compare-allowed-collision (a b)
  (or-compare (config-name-compare (car a) (car b))
              (config-name-compare (cdr a) (cdr b))))

(defun make-scene-graph-files (&rest files)
  (fold #'tree-set-insert
        (make-tree-set #'sycamore-util:string-compare)
        files))

(defstruct scene-graph
  "The data type for scene graphs, i.e., kinematic trees.

This is a purely functional type, so opertions on scene-graphs
create (partial) copies without modifying or destroying the original."
  (frames (make-tree-set #'scene-object-compare) :type tree-set)
  (allowed-collisions (make-collision-set) :type collision-set)
  (files (make-scene-graph-files) :type tree-set)
  (configs (make-tree-set #'scene-object-compare) :type tree-set))

(defun scene-graph-frame-count (sg)
  (tree-set-count (scene-graph-frames sg)))

(defmethod print-object ((object scene-graph) stream)
  (print-unreadable-object (object stream :type t :identity nil)
    (write (scene-graph-frame-names object) :stream stream)))

(defun merge-scene-graph (scene-graph new-scene-graph &key
                                                        (if-exists :error))
  "Combine two scene graphs."
  (declare (type (or (eql :error) (eql :supersede)) if-exists))
  (labels ((merge-set (set-1 set-2 type-string)
             (ecase if-exists
               (:error
                (let ((intersection (tree-set-intersection set-1 set-2)))
                  (assert (zerop (tree-set-count intersection)) ()
                          "Duplicate ~A in merged trees: ~A"
                          type-string
                          (map-tree-set 'list #'scene-object-name intersection)))
                (tree-set-union set-1 set-2))
               (:supersede
                (fold-tree-set #'tree-set-replace set-1 set-2)))))
    (make-scene-graph :frames (merge-set (scene-graph-frames scene-graph)
                                         (scene-graph-frames new-scene-graph)
                                         "frames")
                      :files (tree-set-union (scene-graph-files scene-graph)
                                             (scene-graph-files new-scene-graph))
                      :configs (merge-set (scene-graph-configs scene-graph)
                                          (scene-graph-configs new-scene-graph)
                                          "configs")
                      :allowed-collisions (tree-set-union (scene-graph-allowed-collisions scene-graph)
                                                          (scene-graph-allowed-collisions new-scene-graph)))))

(defun reload-scene-graph (scene-graph &key reload)
  (fold-tree-set (lambda (sg file)
                   (merge-scene-graph sg
                                      (load-scene-file file :reload reload)))
                 (make-scene-graph)
                 (scene-graph-files scene-graph)))



;;; Basic Operations ;;;

(defun scene-graph-find (scene-graph frame-name)
  "Find a frame"
  (tree-set-find (scene-graph-frames scene-graph)
                 frame-name))

(defun scene-graph-lookup (scene-graph frame-name)
  "Find a frame"
  (scene-graph-find scene-graph frame-name))

(defun scene-graph-parent-name (scene-graph frame-name)
  "Find a parent frame name"
  (scene-frame-parent (scene-graph-find scene-graph
                                        frame-name) ))

(defun scene-graph-allow-collision (scene-graph frame-1 frame-2)
  (let ((scene-graph (copy-scene-graph scene-graph)))
    (setf (scene-graph-allowed-collisions scene-graph)
          (collision-set-insert (scene-graph-allowed-collisions scene-graph)
                                frame-1
                                frame-2))
    scene-graph))

(defun scene-graph-allow-collisions (scene-graph collision-set)
  "Insert the provided allowed collisions into the allowable set for SCENE-GRAPH."
  (flet ((add-cons (scene-graph cons)
           (destructuring-bind (a . b) cons
             (scene-graph-allow-collision scene-graph a b))))
    (funcall (etypecase collision-set
               (list #'fold)
               (tree-set #'fold-tree-set))
             #'add-cons scene-graph collision-set)))

(defun %scene-graph-add-frame (scene-graph frame)
  "Add a frame to the scene."
  ;; TODO: handle duplicate configs
  (make-scene-graph :frames (tree-set-replace (scene-graph-frames scene-graph)
                                              frame)
                    :files (scene-graph-files scene-graph)
                    :allowed-collisions (scene-graph-allowed-collisions scene-graph)
                    :configs (if (scene-frame-joint-p frame)
                                 ;; add config
                                 (tree-set-insert (scene-graph-configs scene-graph)
                                                  (make-scene-config
                                                   :name (scene-frame-joint-configuration-name frame)
                                                   :limits (scene-frame-joint-%limits frame)))
                                 ;; now new config
                                 (scene-graph-configs scene-graph))))

(defun %clean-allowed-collisions (frame-name collision-set)
  (fold-tree-set (lambda (new-set x)
                   (if (or (sycamore:rope= (car x) frame-name)
                           (sycamore:rope= (cdr x) frame-name))
                       new-set
                       (tree-set-insert new-set x)))
                 (make-collision-set)
                 collision-set))

(defun scene-graph-clean-frame (scene-graph frame-name)
  "Fully remove a frame from the scene."
  ;; TODO: remove configs
  (make-scene-graph :frames (tree-set-remove (scene-graph-frames scene-graph)
                                             frame-name)
                    :files (scene-graph-files scene-graph)
                    :allowed-collisions (%clean-allowed-collisions frame-name
                                                                   (scene-graph-allowed-collisions
                                                                    scene-graph))
                    :configs (scene-graph-configs scene-graph)))

(defun scene-graph-remove-frame (scene-graph frame-name)
  "Remove a frame from the scene. It still stays in the allowed-collision set."
  (make-scene-graph :frames (tree-set-remove (scene-graph-frames scene-graph)
                                             frame-name)
                    :files (scene-graph-files scene-graph)
                    :allowed-collisions (scene-graph-allowed-collisions scene-graph)
                    :configs (scene-graph-configs scene-graph)))


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

;; TODO: don't assume frame name == config-name
(defun scene-graph-config-limits (scene-graph config-name)
  (when-let ((scene-config (tree-set-find (scene-graph-configs scene-graph)
                                          config-name)))
    (scene-config-limits scene-config)))

(defun scene-graph-joint-center (scene-graph name)
  (let* ((limits (scene-graph-config-limits scene-graph name))
         (position-limit (when limits (joint-limits-position limits))))
    (if position-limit
        (* .5d0 (+ (joint-limit-max position-limit)
                   (joint-limit-min position-limit)))
        0d0)))


(defun scene-graph-center-map (scene-graph)
  (let ((vars (scene-graph-configurations scene-graph)))
    (fold (lambda (map var)
            (tree-map-insert map var (scene-graph-joint-center scene-graph var)))
          (make-configuration-map)
          vars)))

(defun scene-graph-position-limit-p (scene-graph configuration-map)
  "Check if configuration-map is within the position limits of scene-graph"
  (fold-tree-map (lambda (violations name position)
                   (if-let ((limits (scene-graph-config-limits scene-graph name)))
                     (let ((position-limit (joint-limits-position limits)))
                       (if (or (null position-limit)
                               (and (<= position (joint-limit-max position-limit))
                                    (>= position (joint-limit-min position-limit))))
                           ;; in limits
                           violations
                           ;; out of limits
                           (cons name violations)))
                     ;; no limits
                     violations))
                 nil configuration-map))

(defun %scene-graph (things)
  (labels ((rec (scene-graph thing)
             (etypecase thing
               (scene-frame
                (%scene-graph-add-frame scene-graph thing))
               (scene-graph
                (merge-scene-graph scene-graph thing))
               (list
                (merge-scene-graph scene-graph
                                   (%scene-graph thing)))
               ((or pathname string)
                (rec scene-graph (load-scene-file thing)))
               (rope
                (rec scene-graph (rope-string thing))))))
    (fold #'rec (make-scene-graph) things)))

(defun scene-graph (&rest things)
  "Construct a scene-graph from THINGS.

Elements of things may be other scene-graphs, individual frames, files
to load, or sequences of any of these."
  (%scene-graph things))

(defmacro scene-graph-f (place &rest things)
  "Add THINGS to the scene-graph in PLACE and assign result to PLACE."
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


(defun scene-graph-frame-names (scene-graph)
  (map-scene-graph-frames 'list #'scene-frame-name scene-graph))

(defun check-scene-graph (scene-graph)
  "Validate the scene graph.

Throws an error if scene graph is invalid."
  ;; Check frames
  (do-scene-graph-frames (frame scene-graph)
    (let ((name (rope-string (scene-frame-name frame)))
          (parent (rope-string (scene-frame-parent frame))))
      (check-frame-parent parent name)
      ;; TODO: am I my own grandparent
      (check-frame-exists parent scene-graph)))
  ;; Check allowed collisions
  (do-collision-set ((frame-1 frame-2) (scene-graph-allowed-collisions scene-graph))
    (check-frame-exists frame-1 scene-graph)
    (check-frame-exists frame-2 scene-graph)))

(defun check-scene-graph-parents (scene-graph)
  (do-scene-graph-frames (frame scene-graph)
    (let ((name (rope-string (scene-frame-name frame)))
          (parent (rope-string (scene-frame-parent frame))))
      (unless (or (null parent)
                  (zerop (length parent))
                  (scene-graph-lookup scene-graph parent))
        (error "Parent frame `~A' not found for child `~A'"
               parent name))))
  scene-graph)

(defun scene-graph-meshes (scene-graph)
  (let ((meshes (make-hash-table :test #'equalp)))
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
                                                draw-options
                                                root
                                                tf
                                                (prefix-parents t))
  (let ((frames))
    (push (scene-frame-fixed root prefix
                             :tf (tf tf))
          frames)
    (do-scene-graph-frames (frame (scene-graph scene-graph) (scene-graph frames))
      (let ((frame (copy-structure frame))
            (parent (scene-frame-parent frame)))
        ;; prefix name
        (setf (scene-frame-name frame)
              (rope prefix (scene-frame-name frame)))
        ;; prefix parent
        (if parent
            (when prefix-parents
              (setf (scene-frame-parent frame)
                    (rope prefix parent)))
            (setf (scene-frame-parent frame)
                  prefix))
        ;; prefix config
        (when (scene-frame-joint-p frame)
          (setf (scene-frame-joint-configuration-name frame)
                (rope prefix (scene-frame-joint-configuration-name frame))))
        ;; draw-options
        (when draw-options
          ;(print (scene-frame-geometry frame))
          ;; (print (map 'list
          ;;             (lambda (g)
          ;;               (modify-scene-geometry
          ;;                g
          ;;                (merge-draw-options draw-options
          ;;                                    (scene-geometry-options g))))
          ;;             (scene-frame-geometry frame)))
          (setf (scene-frame-geometry frame)
                (map 'list
                     (lambda (g)
                       (modify-scene-geometry
                        g
                        (merge-draw-options draw-options
                                            (scene-geometry-options g))))
                     (scene-frame-geometry frame)))
          )
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



(defun scene-graph-remove-geometry (scene-graph frames)
  (let* ((frames (etypecase frames
                  (list (apply #'tree-set #'frame-name-compare frames))
                  (tree-set frames)))
         (new-sg (copy-scene-graph scene-graph))
         (new-frames (fold-scene-graph-frames
                      (lambda (set frame)
                        (tree-set-insert set
                                         (if (tree-set-member-p frames (scene-frame-name frame))
                                             (let ((frame (copy-scene-frame frame)))
                                               (setf (scene-frame-geometry frame) nil)
                                               frame)
                                             frame)))
                      (tree-set #'scene-object-compare) new-sg)))


    (setf (scene-graph-frames new-sg) new-frames)
    new-sg))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Computing Transforms ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun alist-configuration-map (alist &optional default-map)
  (fold (lambda (map item)
          (tree-map-insert map (car item) (cdr item)))
        (or default-map (make-configuration-map))
        alist))

(defun pairlist-configuration-map (names values &optional default-map)
  (fold (lambda (map name value)
          (tree-map-insert map name value))
        (or default-map (make-configuration-map))
        names values))

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
  "Compute the absolute transform for FRAME-NAME."
  (declare (type hash-table tf-absolute-map))
  (labels ((rec (frame-name)
             (cond
               ((frame-name-global-p frame-name) ; global frame
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



(defun scene-graph-allowed-collisions-rope (scene-graph)
  (rope (map-tree-set 'list (lambda (pair)
                              (format nil "allow_collision \"~A\" \"~A\";~%"
                                      (car pair) (cdr pair)))
                      (scene-graph-allowed-collisions scene-graph))))

(defun scene-graph-dot (scene-graph &key output)
  (output-dot output
              (lambda (s)
                (labels ((sanitize (name)
                           (substitute #\_ #\- name))
                         (node (frame)
                           (let ((n (scene-frame-name frame))
                                 (shape (if (scene-frame-joint-p frame)
                                            "oval"
                                            "box")))

                             (format s "~&  ~A[label=\"~A\",shape=~A];"
                                     (sanitize n)
                                     n
                                     shape)))
                         (edge (frame)
                           (let ((n (scene-frame-name frame))
                                 (p (scene-frame-parent frame))
                                 (q (if (scene-frame-joint-p frame)
                                        (scene-frame-joint-configuration-name frame)
                                        "")))
                           (format s "~&  ~A -> ~A[label=\"~A\"];"
                                   (if (frame-name-global-p p)
                                       "ROOT"
                                       (sanitize p))
                                   (sanitize n)
                                   (sanitize q)))))
                  (format s "~&digraph {  ~&")
                  (do-scene-graph-frames (frame scene-graph)
                    (node frame))
                  (do-scene-graph-frames (frame scene-graph)
                    (edge frame))
                  (format s "~&}~&")))))
