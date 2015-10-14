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

;;;;;;;;;;;;;;;;;;;;;;;
;;; Foreign Binding ;;;
;;;;;;;;;;;;;;;;;;;;;;;

(cffi:defcfun aa-rx-sg-destroy :void
  (value :pointer))

(cffi:defcfun aa-rx-sg-create rx-sg-t)

(cffi:defcfun aa-rx-sg-init :void
  (sg rx-sg-t))

(cffi:defcfun aa-rx-sg-frame-type frame-type
  (sg rx-sg-t)
  (frame rx-frame-id))

(cffi:defcfun aa-rx-sg-frame-name :string
  (sg rx-sg-t)
  (frame rx-frame-id))

(cffi:defcfun aa-rx-sg-config-name :string
  (sg rx-sg-t)
  (config rx-config-id))

(cffi:defcfun aa-rx-sg-frame-parent rx-frame-id
  (sg rx-sg-t)
  (frame rx-frame-id))

(cffi:defcfun aa-rx-sg-frame-count size-t
  (sg rx-sg-t))

(cffi:defcfun aa-rx-sg-config-count size-t
  (sg rx-sg-t))

(cffi:defcfun aa-rx-sg-frame-config rx-config-id
  (sg rx-sg-t)
  (frame rx-frame-id))

(cffi:defcfun aa-rx-sg-frame-id rx-frame-id
  (sg rx-sg-t)
  (name :string))

(cffi:defcfun aa-rx-sg-config-id rx-config-id
  (sg rx-sg-t)
  (name :string))

(cffi:defcfun aa-rx-sg-add-frame-fixed :void
  (sg rx-sg-t)
  (parent :string)
  (name :string)
  (q amino::quaternion-t)
  (v amino::vector-3-t))

(cffi:defcfun aa-rx-sg-add-frame-prismatic :void
  (sg rx-sg-t)
  (parent :string)
  (name :string)
  (q amino::quaternion-t)
  (v amino::vector-3-t)
  (config-name :string)
  (axis amino::vector-3-t)
  (offset :double))

(cffi:defcfun aa-rx-sg-add-frame-revolute :void
  (sg rx-sg-t)
  (parent :string)
  (name :string)
  (q amino::quaternion-t)
  (v amino::vector-3-t)
  (config-name :string)
  (axis amino::vector-3-t)
  (offset :double))

(cffi:defcfun aa-rx-sg-rm-frame :void
  (sg rx-sg-t)
  (name :string))

(cffi:defcfun aa-rx-sg-set-limit-pos :void
  (sg rx-sg-t)
  (config-name :string)
  (min :double)
  (max :double))

(cffi:defcfun aa-rx-sg-set-limit-vel :void
  (sg rx-sg-t)
  (config-name :string)
  (min :double)
  (max :double))

(cffi:defcfun aa-rx-sg-set-limit-acc :void
  (sg rx-sg-t)
  (config-name :string)
  (min :double)
  (max :double))

(cffi:defcfun aa-rx-sg-set-limit-eff :void
  (sg rx-sg-t)
  (config-name :string)
  (min :double)
  (max :double))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Mutable Scene Graphs ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; (defmethod print-object ((object mutable-scene-graph) stream)
;;   (print-unreadable-object (object stream :type t)
;;     (format stream "(~x)"
;;             (cffi:pointer-address (amino-ffi::foreign-container-pointer object)))))

(defun mutable-scene-graph (scene-graph)
  (let ((sg (aa-rx-sg-create))
        (scene-graph (scene-graph scene-graph)))
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
           (let ((axis (scene-frame-joint-axis frame))
                 (offset (scene-frame-joint-configuration-offset frame))
                 (config-name (scene-frame-joint-configuration-name frame))
                 (limits (scene-frame-joint-limits frame)))
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
    (aa-rx-sg-init sg)
    ;; Create lisp indices between configuration names and IDs
    (let* ((n-config (aa-rx-sg-config-count sg))
           (hash (make-hash-table :test #'equal))
           (array (make-array n-config)))
      (loop for config-id below n-config
         for config-name = (aa-rx-sg-config-name sg config-id)
         do
           (setf (gethash config-name hash) config-id
                 (aref array config-id) config-name))
      (setf (mutable-scene-graph-config-name-array sg) array
            (mutable-scene-graph-config-index-map sg) hash))
    ;; Result
    sg))

(defun config-vector-helper (map configs vector)
  (labels ((helper (config-name config-value)
             (setf (aref vector (gethash (rope-string config-name) map))
                   (coerce config-value 'double-float))))
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

;;;;;;;;;;;;;;;;;
;;; Subgraphs ;;;
;;;;;;;;;;;;;;;;;

(cffi:defcfun aa-rx-sg-sub-destroy :void
  (ssg :pointer))

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
  (let* ((m-sg (mutable-scene-graph scenegraph))
         (root-id (aa-rx-sg-frame-id m-sg (rope-string root)))
         (tip-id (aa-rx-sg-frame-id m-sg (rope-string tip))))
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
