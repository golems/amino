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

;;;;;;;;;;;;;;
;;; CL Set ;;;
;;;;;;;;;;;;;;

(cffi:defcfun aa-rx-cl-set-destroy :void
  (cl-set :pointer))

(cffi:defcfun aa-rx-cl-set-create rx-cl-set-t
  (m-sg rx-sg-t))

(cffi:defcfun aa-rx-cl-set-set :void
  (cl-set rx-cl-set-t)
  (i rx-frame-id)
  (j rx-frame-id)
  (is-colliding :boolean))

(cffi:defcfun aa-rx-cl-set-get :boolean
  (cl-set rx-cl-set-t)
  (i rx-frame-id)
  (j rx-frame-id))

;;;;;;;;;;;;;;;;;;
;;; Collisions ;;;
;;;;;;;;;;;;;;;;;;

(cffi:defcfun aa-rx-cl-destroy :void
  (cl :pointer))

(cffi:defcfun aa-rx-cl-create rx-cl-t
  (m-sg rx-sg-t))

(cffi:defcfun aa-rx-cl-allow-name rx-cl-t
  (cl rx-cl-t)
  (frame-0 :string)
  (frame-1 :string)
  (allowed :boolean))

(cffi:defcfun aa-rx-cl-allow :void
  (cl rx-cl-t)
  (id0 rx-frame-id)
  (id1 rx-frame-id)
  (allowed :boolean))

(cffi:defcfun aa-rx-cl-check :boolean
  (cl rx-cl-t)
  (n-tf size-t)
  (tf :pointer)
  (ld-tf size-t)
  (cl-set rx-cl-set-t))

;;;;;;;;;;;;;;;;
;;; Wrappers ;;;
;;;;;;;;;;;;;;;;

(defun cl-set-extract (m-sg cl-set)
  (let ((scene-graph (mutable-scene-graph-scene-graph m-sg))
        (result (make-collision-set)))
    (do-scene-graph-frames (frame-1 scene-graph)
      (do-scene-graph-frames (frame-2 scene-graph)
        (let ((frame-1 (scene-frame-name frame-1))
              (frame-2 (scene-frame-name frame-2)))
          (cond-compare (frame-1 frame-2 #'frame-name-compare)
                        (let ((id-1  (mutable-scene-graph-frame-id m-sg frame-1))
                              (id-2  (mutable-scene-graph-frame-id m-sg frame-2)))
                          (when (aa-rx-cl-set-get cl-set id-1 id-2)
                            (setq result (collision-set-insert result frame-1 frame-2))))
                        nil
                        nil))))
      result))

(defun scene-collision-check (scene-graph &key
                                            collect-collisions
                                            configuration-map)
  (let ((m-sg (mutable-scene-graph scene-graph)))
    (aa-rx-sg-cl-init m-sg)
    (let* ((tf-abs (mutable-scene-graph-tf-array m-sg :configuration-map configuration-map))
           (frame-count (mutable-scene-graph-frame-count m-sg))
           (cl-set (if collect-collisions
                       (aa-rx-cl-set-create m-sg)
                       (null-rx-cl-set)))
           (cl (aa-rx-cl-create m-sg)))
      ;; Allow Collisions
      (do-tree-set (cons (scene-graph-allowed-collisions scene-graph))
        (destructuring-bind (a . b) cons
          (let ((id-0 (mutable-scene-graph-frame-id m-sg a))
                (id-1 (mutable-scene-graph-frame-id m-sg b)))
            (aa-rx-cl-allow cl id-0 id-1 t))))
      ;; Get Result
      (let ((result))
        (with-foreign-simple-vector (tf-ptr tf-length) tf-abs :input
          (assert (= (* 7 frame-count) tf-length))
          (setq result
                (aa-rx-cl-check cl frame-count tf-ptr 7
                                cl-set)))
        (when result
          (if collect-collisions
              (cl-set-extract m-sg cl-set)
              t))))))

(defun scene-graph-allow-configuration (scene-graph configuration-map)
  (let ((collisions (scene-collision-check scene-graph
                                           :collect-collisions t
                                           :configuration-map configuration-map)))
    (scene-graph-allow-collisions scene-graph collisions)))
