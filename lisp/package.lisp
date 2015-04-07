;;;; -*- mode: lisp -*-
;;;;
;;;; Copyright (c) 2013, Georgia Tech Research Corporation
;;;; All rights reserved.
;;;;
;;;; Author(s): Neil T. Dantam <ntd@gatech.edu>
;;;; Georgia Tech Humanoid Robotics Lab
;;;; Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
;;;;
;;;;
;;;; This file is provided under the following "BSD-style" License:
;;;;
;;;;
;;;;   Redistribution and use in source and binary forms, with or
;;;;   without modification, are permitted provided that the following
;;;;   conditions are met:
;;;;
;;;;   * Redistributions of source code must retain the above copyright
;;;;     notice, this list of conditions and the following disclaimer.
;;;;
;;;;   * Redistributions in binary form must reproduce the above
;;;;     copyright notice, this list of conditions and the following
;;;;     disclaimer in the documentation and/or other materials provided
;;;;     with the distribution.
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

(defpackage :amino-type
  (:use :cl :cffi :alexandria)
  (:export
   ;; real-array
   :real-array
   :make-real-array
   :real-array-data
   ;; matrix
   :matrix
   :%make-matrix
   :%matrix-data
   :%matrix-offset
   :%matrix-stride
   :%matrix-cols
   :%matrix-rows
   ;; conditions
   :matrix-storage-error
   :matrix-dimension-error
   :check-matrix-bounds
   :check-matrix-dimensions
   ))


(defpackage :amino-ffi
  (:use :cl :cffi :alexandria :amino-type)
  (:export
   ;; macros
   :def-ref-type
   :with-reference
   :with-foreign-matrix
   :with-foreign-vector
   :with-foreign-simple-vector
   :def-la-cfun
   :def-blas-cfun
   ;; foreign types
   :size-t
   :int-ref-t :double-ref-t :float-ref-t :size-ref-t :char-ref-t
   ;; BLAS
   :blas-size-t :blas-size-ref-t
   :def-blas-cfun
   :transpose-t
   ))

(defpackage :amino
  (:use :cl :cffi :alexandria :amino-type :amino-ffi)
  (:export
   ;; General types
   :vec
   :make-vec
   ;; TF Types
   :vec3 :vec3*
   :axis-angle :axis-angle*
   :quaternion :quaternion* :quaternion-x :quaternion-y :quaternion-z :quaternion-w
   :rotation-matrix
   :euler-zyx :euler-zyx*
   :dual-quaternion :quaternion-translation :transformation-matrix
   :dual-quaternion-2 :quaternion-translation-2 :transformation-matrix-2
   :x-angle :y-angle :z-angle
   :tf
   :translation
   :rotation
   ;; CFFI Translated types
   ;; Generics
   :transform
   :g*
   :matrix->list
   :inverse
   ;; Misc
   :parse-float
   )
  (:nicknames :aa))
