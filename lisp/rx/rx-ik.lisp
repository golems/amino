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

(defparameter *ik-options* nil)


;;;;;;;;;;;;;;;;;;
;;; IK OPTIONS ;;;
;;;;;;;;;;;;;;;;;;

(cffi:defcfun aa-rx-ksol-opts-create rx-ksol-opts-t)

(cffi:defcfun aa-rx-ksol-opts-set-dt :void
  (opts rx-ksol-opts-t)
  (dt :double))

(cffi:defcfun aa-rx-ksol-opts-set-tol-angle :void
  (opts rx-ksol-opts-t)
  (tol :double))

(cffi:defcfun aa-rx-ksol-opts-set-tol-trans :void
  (opts rx-ksol-opts-t)
  (tol :double))

(cffi:defcfun aa-rx-ksol-opts-set-tol-angle-svd :void
  (opts rx-ksol-opts-t)
  (tol :double))

(cffi:defcfun aa-rx-ksol-opts-set-tol-trans-svd :void
  (opts rx-ksol-opts-t)
  (tol :double))

(cffi:defcfun aa-rx-ksol-opts-set-tol-dq :void
  (opts rx-ksol-opts-t)
  (tol :double))

(cffi:defcfun aa-rx-ksol-opts-set-tol-k-dls :void
  (opts rx-ksol-opts-t)
  (k :double))

(cffi:defcfun aa-rx-ksol-opts-set-tol-s2min :void
  (opts rx-ksol-opts-t)
  (s2min :double))

(cffi:defcfun aa-rx-ksol-opts-set-gain-angle :void
  (opts rx-ksol-opts-t)
  (k :double))

(cffi:defcfun aa-rx-ksol-opts-set-gain-trans :void
  (opts rx-ksol-opts-t)
  (k :double))

(cffi:defcfun aa-rx-ksol-opts-set-frame :void
  (opts rx-ksol-opts-t)
  (k :int))

(cffi:defcfun aa-rx-ksol-opts-get-frame :int
  (opts rx-ksol-opts-t))

(cffi:defcfun aa-rx-ksol-opts-set-max-iterations :void
  (opts rx-ksol-opts-t)
  (n size-t))

(cffi:defcfun aa-rx-ksol-opts-take-config :void
  (opts rx-ksol-opts-t)
  (n size-t)
  (q :pointer)
  (refop ref-op))

(cffi:defcfun aa-rx-ksol-opts-take-gain-config :void
  (opts rx-ksol-opts-t)
  (n size-t)
  (q :pointer)
  (refop ref-op))

(cffi:defcfun aa-rx-ksol-opts-take-seed :void
  (opts rx-ksol-opts-t)
  (n size-t)
  (q :pointer)
  (refop ref-op))

(cffi:defcfun aa-rx-ksol-opts-center-configs :void
  (opts rx-ksol-opts-t)
  (ssg rx-sg-sub-t)
  (gain :double))

;;;;;;;;;;;;;;;;;;;
;;; Jacobian IK ;;;
;;;;;;;;;;;;;;;;;;;

(cffi:defcfun aa-rx-ik-jac-cx-create rx-ik-jac-cx-t
  (sub-scene-graph rx-sg-sub-t)
  (opts rx-ksol-opts-t))

(cffi:defcfun aa-rx-ik-jac-solve :int
  (context rx-ik-jac-cx-t)
  (n-tf size-t)
  (tf :pointer)
  (ld-tf size-t)
  (n-q size-t)
  (q :pointer))

;; (cffi:defcfun aa-rx-sg-sub-ksol-dls :int
;;   (ssg rx-sg-sub-t)
;;   (opts (rx-ksol-opts-t))
;;   (n-tf size-t)
;;   (tf :pointer)
;;   (ld-tf size-t)
;;   (n-q-all size-t)
;;   (q-all :pointer)
;;   (n-q size-t)
;;   (q-subset :pointer))


;;;;;;;;;;;;;;;
;;; Wrapper ;;;
;;;;;;;;;;;;;;;

(defun ksol-opt (&key dt frame-id
                   gain-angle gain-trans)
  (let ((opts (aa-rx-ksol-opts-create)))
    (when dt
      (aa-rx-ksol-opts-set-dt opts dt))
    (when frame-id
      (aa-rx-ksol-opts-set-frame opts frame-id))
    (when gain-angle
      (aa-rx-ksol-opts-set-gain-angle opts gain-angle))
    (when gain-trans
      (aa-rx-ksol-opts-set-gain-trans opts gain-trans))
    opts))

(defun scene-graph-ik (scene-graph
                       &key
                         start
                         frame
                         tf
                         parent
                         (options *ik-options*))
  (let* ((tf (if parent
                 (tf-mul (scene-graph-tf-absolute scene-graph parent :configuration-map start)
                         tf)
                 tf))
         (opts (or options (aa-rx-ksol-opts-create)))
         (ssg (scene-graph-chain scene-graph nil frame))
         (ik-cx (aa-rx-ik-jac-cx-create ssg opts))
         (q-sub (make-vec (sub-scene-graph-config-count ssg)))
         (tf-array (tf-array tf))
         (start (or start (sub-scene-graph-center-map ssg)))
         (q-all (make-vec (sub-scene-graph-all-config-count ssg))))
    ;; TODO: Option arguments
    (sub-scene-graph-all-config-vector ssg start q-all)
    ;; Set options
    (with-foreign-simple-vector (q-all-ptr q-all-length) q-all :input
      (aa-rx-ksol-opts-take-seed opts q-all-length q-all-ptr :borrow))
    (let ((r))
      (with-foreign-simple-vector (q-sub-ptr q-sub-length) q-sub :output
        (with-foreign-simple-vector (tf-ptr tf-length) tf-array :input
          (assert (= 7 tf-length))
          (setq r
                (aa-rx-ik-jac-solve ik-cx
                                    1 tf-ptr 7
                                    q-sub-length q-sub-ptr))))
      (when (zerop r)
        (sub-scene-graph-config-map ssg q-sub)))))
