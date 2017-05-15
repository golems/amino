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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Mutable Scene Graphs ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(deftype frame-id ()
  `(unsigned-byte 31))

(defun mutable-scene-graph-frame-id (m-sg frame)
  (aa-rx-sg-frame-id m-sg (rope-string frame)))

(defun mutable-scene-graph-frame-name (m-sg id)
  (aa-rx-sg-frame-name m-sg id))

;; (defmethod print-object ((object mutable-scene-graph) stream)
;;   (print-unreadable-object (object stream :type t)
;;     (format stream "(~x)"
;;             (cffi:pointer-address (amino-ffi::foreign-container-pointer object)))))

(defun %mutable-scene-graph (scene-graph)
  (let ((scene-graph (check-scene-graph-parents (scene-graph scene-graph)))
        (sg (aa-rx-sg-create)))
    (setf (mutable-scene-graph-scene-graph sg) scene-graph)
    ;; Insert Frames
    (do-scene-graph-frames (frame scene-graph)
      (let* ((tf (scene-frame-tf frame))
             (q (tf-quaternion tf))
             (v (tf-translation tf))
             (name (rope-string (scene-frame-name frame)))
             (parent (rope-string (scene-frame-parent frame))))

        (etypecase frame
          ;; fix frame
          (scene-frame-fixed
           (aa-rx-sg-add-frame-fixed sg parent name q v))
          ;; joint frame
          (scene-frame-joint
           (let* ((axis (scene-frame-joint-axis frame))
                  (offset (scene-frame-joint-configuration-offset frame))
                  (config-name (rope-string (scene-frame-joint-configuration-name frame)))
                  (limits (scene-graph-config-limits scene-graph config-name)))
             ;; add the frame
             (etypecase frame
               (scene-frame-revolute
                (aa-rx-sg-add-frame-revolute sg parent name q v
                                             config-name axis offset))
               (scene-frame-prismatic
                (aa-rx-sg-add-frame-revolute sg parent name q v
                                             config-name axis offset)))
             ;; set limits
             (when limits
               (when-let ((limit (joint-limits-position limits)))
                 (aa-rx-sg-set-limit-pos sg config-name
                                         (joint-limit-min limit)
                                         (joint-limit-max limit)))
               (when-let ((limit (joint-limits-velocity limits)))
                 (aa-rx-sg-set-limit-vel sg config-name
                                         (joint-limit-min limit)
                                         (joint-limit-max limit)))
               (when-let ((limit (joint-limits-acceleration limits)))
                 (aa-rx-sg-set-limit-acc sg config-name
                                         (joint-limit-min limit)
                                         (joint-limit-max limit)))
               (when-let ((limit (joint-limits-effort limits)))
                 (aa-rx-sg-set-limit-eff sg config-name
                                         (joint-limit-min limit)
                                         (joint-limit-max limit)))))))))
    ;; Insert Geometry
    (do-scene-graph-geometry ((frame geometry) scene-graph)
      (let* ((frame-name (rope-string (scene-frame-name frame)))
             (shape (scene-geometry-shape geometry))
             (c-geom
              (typecase shape
                (scene-text
                 (warn "Text shape not implementated in mutable scene."))
                ;(scene-mesh
                 ;(warn "Mesh shape not implementated in mutable scene."))
                (t
                 (when-let ((c-geom (scene-geometry-c-geom geometry)))
                   ;; up refcount
                   (aa-rx-geom-copy c-geom)
                   c-geom)))))
        ;; attach
        (when c-geom
          (aa-rx-geom-attach sg frame-name c-geom))))
    ;; Index
    (let ((result (aa-rx-sg-init sg)))
      (unless (zerop result)
        (error "Could not initialize mutable-scene-graph: 0x~x" result)))
    ;; Create lisp indices between configuration names and IDs
    (let* ((n-config (aa-rx-sg-config-count sg))
           (hash (make-hash-table :test #'equal))
           (array (make-array n-config)))
      (loop for config-id below n-config
         for config-name = (aa-rx-sg-config-name sg config-id)
         do
           (assert (= config-id
                      (aa-rx-sg-config-id sg config-name)))
           (setf (gethash config-name hash) config-id
                 (aref array config-id) config-name))
      (setf (mutable-scene-graph-config-name-array sg) array
            (mutable-scene-graph-config-index-map sg) hash))
    ;; Result
    sg))


(defun mutable-scene-graph (scene-graph)
  (etypecase scene-graph
    (mutable-scene-graph scene-graph)
    (scene-graph (%mutable-scene-graph scene-graph))
    (sub-scene-graph (sub-scene-graph-mutable-scene-graph scene-graph))))

(defun config-vector-helper (map configs vector)
  (labels ((helper (config-name config-value)
             (when-let ((i (gethash (rope-string config-name) map)))
               (setf (aref vector i)
                     (coerce config-value 'double-float)))))
    (etypecase configs
      (tree-map
       (do-tree-map ((config-name config-value) configs)
         (helper config-name config-value)))
      (list
       (loop for (config-name . config-value) in configs
          do (helper config-name config-value)))))
  vector)

(defun mutable-scene-graph-config-vector (mutable-scene-graph configs &optional vector)
  (let* ((m-sg mutable-scene-graph)
         (n (aa-rx-sg-config-count m-sg)))
    (config-vector-helper (mutable-scene-graph-config-index-map m-sg)
                          configs
                          (or vector (make-vec n)))))

(defun mutable-scene-graph-config-map (mutable-scene-graph vector &key (start 0))
  (let ((n (mutable-scene-graph-config-count mutable-scene-graph))
        (map (make-configuration-map)))
    (loop for i below n
       for name = (mutable-scene-graph-config-name mutable-scene-graph i)
       for i-array = (+ start i)
       do
         (tree-map-insertf map name
                           (aref vector i-array)))
    map))

(defun mutable-scene-graph-config-name (m-sg i)
  (aref (mutable-scene-graph-config-name-array m-sg)
        i))

(defun mutable-scene-graph-config-index (m-sg name)
  (gethash name (mutable-scene-graph-config-index-map m-sg)))

(defun mutable-scene-graph-frame-count (m-sg)
  (aa-rx-sg-frame-count m-sg))

(defun mutable-scene-graph-tf-array (m-sg &key
                                            configuration-map
                                            tf-absolute
                                            tf-relative)
  (let* ((config-array (mutable-scene-graph-config-vector m-sg configuration-map))
         (frame-count (mutable-scene-graph-frame-count m-sg))
         (tf-absolute (or tf-absolute (make-vec (* 7 frame-count))))
         (tf-relative (or tf-relative (make-vec (* 7 frame-count)))))
    (assert (>= (length tf-relative)
                (* 7 frame-count)))
    (assert (>= (length tf-absolute)
                (* 7 frame-count)))
    (with-foreign-simple-vector (config-ptr config-length) config-array :input
      (with-foreign-simple-vector (rel-ptr rel-length) tf-relative :output
        (declare (ignore rel-length))
        (with-foreign-simple-vector (abs-ptr abs-length) tf-absolute :output
          (declare (ignore abs-length))
          (aa-rx-sg-tf m-sg
                       config-length config-ptr
                       frame-count
                       rel-ptr 7
                       abs-ptr 7))))
    (values tf-absolute
            tf-relative)))

;;;;;;;;;;;;;;;;;
;;; Subgraphs ;;;
;;;;;;;;;;;;;;;;;

(cffi:defcfun aa-rx-sg-chain-create rx-sg-sub-t
  (sg rx-sg-t)
  (root rx-frame-id)
  (tip rx-frame-id))

(cffi:defcfun aa-rx-sg-sub-config rx-config-id
  (sg rx-sg-sub-t)
  (i size-t))

(cffi:defcfun aa-rx-sg-sub-frame rx-frame-id
  (sg rx-sg-sub-t)
  (i size-t))

(cffi:defcfun aa-rx-sg-sub-config-get :void
  (ssg rx-sg-sub-t)
  (n-all size-t)
  (config-all :pointer)
  (n-sub size-t)
  (config-sub :pointer))

(cffi:defcfun aa-rx-sg-sub-config-set :void
  (ssg rx-sg-sub-t)
  (n-sub size-t)
  (config-sub :pointer)
  (n-all size-t)
  (config-all :pointer))


(cffi:defcfun aa-rx-sg-sub-expand-path :void
  (ssg rx-sg-sub-t)
  (n-pts amino-ffi::size-t)
  (q-start :pointer)
  (path-sub :pointer)
  (path-all :pointer))

(defun sub-scene-graph-init (m-sg ssg)
  (setf (sub-scene-graph-mutable-scene-graph ssg)
        m-sg)
  (let* ((n (sub-scene-graph-config-count ssg))
         (hash (make-hash-table :test #'equal))
         (array (make-array n)))
    (loop for i below n
       for config-id = (aa-rx-sg-sub-config ssg i)
       for config-name = (aa-rx-sg-config-name m-sg config-id)
       do
         (setf (gethash config-name hash) i
               (aref array i) config-name))
    (setf (sub-scene-graph-config-name-array ssg) array
          (sub-scene-graph-config-index-map ssg) hash))
  ssg)

(defun scene-graph-chain (scenegraph root tip)
  "Construct the sub-scene-graph for a kinematic chain from ROOT to TIP."
  (let* ((m-sg (mutable-scene-graph scenegraph))
         (root-id (mutable-scene-graph-frame-id m-sg root))
         (tip-id (mutable-scene-graph-frame-id m-sg tip)))
    (when (and (< root-id 0)
               (not (= root-id +frame-id-root+)))
      (error "Unrecognized frame ~A" root))
    (when (< tip-id 0)
      (error "Unrecognized frame ~A" tip))
    (let ((ssg (aa-rx-sg-chain-create m-sg root-id tip-id)))
      (sub-scene-graph-init m-sg ssg))))

(cffi:defcfun (sub-scene-graph-config-count "aa_rx_sg_sub_config_count") size-t
  (sub-sg rx-sg-sub-t))

(defun sub-scene-graph-all-config-count (ssg)
  (let ((m-sg (sub-scene-graph-mutable-scene-graph ssg)))
    (aa-rx-sg-config-count m-sg)))

(defun sub-scene-graph-config-vector (ssg configs &optional vector)
  (config-vector-helper (sub-scene-graph-config-index-map ssg)
                        configs
                        (or vector
                            (make-vec (sub-scene-graph-config-count ssg)))))

(defun sub-scene-graph-all-config-vector (ssg configs &optional vector)
  (mutable-scene-graph-config-vector (sub-scene-graph-mutable-scene-graph ssg)
                                     configs
                                     vector))

(defun sub-scene-graph-config-map (ssg sub-vector &optional (map (make-configuration-map)))
  (assert (= (length sub-vector)
             (sub-scene-graph-config-count ssg)))
  (dotimes (i (length sub-vector))
    (tree-map-insertf map
                      (sub-scene-graph-config-name ssg i)
                      (aref sub-vector i)))
  map)

(defun sub-scene-graph-all-config-map (ssg all-vector &optional (map (make-configuration-map)))
  (assert (= (length all-vector)
             (sub-scene-graph-all-config-count ssg)))
  (dotimes (i (length all-vector))
    (tree-map-insertf map
                      (sub-scene-graph-all-config-name ssg i)
                      (aref all-vector i)))
  (print map)
  map)

(defun sub-scene-graph-config-name (ssg i)
  (aref (sub-scene-graph-config-name-array ssg)
        i))

(defun sub-scene-graph-all-config-name (ssg i)
  (mutable-scene-graph-config-name (sub-scene-graph-mutable-scene-graph ssg)
                                   i))

(defun sub-scene-graph-scene-graph (ssg)
  "Return the (full) scene-graph for a sub-scene-graph."
  (mutable-scene-graph-scene-graph (sub-scene-graph-mutable-scene-graph ssg)))

(defun sub-scene-graph-center-map (ssg &optional (configuration-map (make-configuration-map)))
  ;; TODO: configs vs frames
  (dotimes (i (sub-scene-graph-config-count ssg))
    (let ((name (sub-scene-graph-config-name ssg i)))
      (tree-map-insertf configuration-map name (scene-graph-joint-center (sub-scene-graph-scene-graph ssg)
                                                                         name))))
  configuration-map)
