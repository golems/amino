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


;;; MP struct ;;;

(cffi:defcfun aa-rx-mp-destroy :void
  (mp :pointer))

(cffi:defcfun aa-rx-mp-create rx-mp-t
  (ssg rx-sg-sub-t))

(defun motion-planner (sub-scene-graph)
  (aa-rx-mp-create sub-scene-graph))

(cffi:defcfun aa-rx-mp-set-start :void
  (mp rx-mp-t)
  (n-all size-t)
  (q-all :pointer))

(cffi:defcfun aa-rx-mp-set-goal :void
  (mp rx-mp-t)
  (n-subset size-t)
  (q-subset :pointer))

(cffi:defcfun aa-rx-mp-plan :int
  (mp rx-mp-t)
  (timeout amino-ffi::coercible-double)
  (n-path :pointer)
  (p-path :pointer))

(cffi:defcfun aa-rx-sg-cl-init :void
  (sg rx-sg-t))

(defun motion-plan (sub-scene-graph start-map goal-map
                    &key
                      (timeout 1d0))
  (let* ((ssg sub-scene-graph)
         (m-sg (sub-scene-graph-mutable-scene-graph ssg)))
    (aa-rx-sg-cl-init m-sg)
    (let* ((planner (motion-planner sub-scene-graph))
           (n-all (sub-scene-graph-all-config-count ssg))
           (q-start (make-vec n-all))
           (q-goal (make-vec (sub-scene-graph-config-count ssg))))
      ;; Set start state
      (sub-scene-graph-all-config-vector ssg start-map q-start)
      (with-foreign-simple-vector (pointer length) q-start :input
        (aa-rx-mp-set-start planner length pointer))
      ;; Set goal state
      ;; TODO: copy unset elements from start
      (sub-scene-graph-config-vector ssg goal-map q-goal)
      (with-foreign-simple-vector (pointer length) q-goal :input
        (aa-rx-mp-set-goal planner length pointer))
      ;; Call Planner
      (multiple-value-bind (result n-path path-ptr)
          (cffi:with-foreign-object (plan-length 'amino-ffi:size-t)
            (cffi:with-foreign-object (plan-ptr :pointer)
              (let ((result (aa-rx-mp-plan planner timeout plan-length plan-ptr)))
                (values result
                        (cffi:mem-ref plan-length 'amino-ffi:size-t)
                        (cffi:mem-ref plan-ptr :pointer)))))
        (if (< result 0)
            ;; error, no plan
            (values nil nil)
            ;; got a plan
            (let* ((m (* n-path n-all))
                   (result (make-vec m)))
              ;; copy plan to lisp-space
              (with-foreign-simple-vector (pointer length) result :output
                (amino-ffi:libc-memcpy pointer path-ptr
                                       (* length (cffi:foreign-type-size :double))))
              ;; free c-space plan
              (amino-ffi:libc-free path-ptr)
              (values result t)))))))
