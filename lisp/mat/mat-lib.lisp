;;;; -*- mode: lisp -*-
;;;;
;;;; Copyright (c) 2018, Colorado School of Mines
;;;; All rights reserved.
;;;;
;;;; Author(s): Neil T. Dantam <ndantam@mines.edu>
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


;; descriptor binding
(in-package :amino)


(defcallback ld-error-handler :void
    ((message :string))
  (matrix-storage-error "~A" message))


(defcfun aa-la-set-err :void
  (fun :pointer))

;;;;;;;;;;;;;;;;;;;;
;;; BLAS Level 1 ;;;
;;;;;;;;;;;;;;;;;;;;

(defcfun aa-dvec-swap :void
  (x dvec-inout)
  (y dvec-inout))

(defcfun aa-dvec-scal :void
  (a :double)
  (x dvec-inout))

(defcfun aa-dvec-copy :void
  (x dvec-input)
  (y dvec-output))

(defcfun aa-dvec-dot :double
  (x dvec-input)
  (y dvec-input))

(defcfun aa-dvec-nrm2 :double
  (x dvec-input))

(defcfun aa-dvec-axpy :void
  (a :double)
  (x dvec-input)
  (y dvec-inout))

;;;;;;;;;;;;;;;;;;;;
;;; BLAS Level 2 ;;;
;;;;;;;;;;;;;;;;;;;;

(defcfun aa-dmat-gemv :void
  (trans transpose-t)
  (alpha :double)
  (A dmat-input)
  (x dvec-input)
  (beta :double)
  (y dvec-inout))

;;;;;;;;;;;;;;;;;;;;
;;; BLAS Level 3 ;;;
;;;;;;;;;;;;;;;;;;;;

(defcfun aa-dmat-gemm :void
  (trans-A transpose-t)
  (trans-B transpose-t)
  (alpha :double)
  (A dmat-input)
  (B dmat-input)
  (beta :double)
  (C dmat-inout))


;;;;;;;;;;;;;;;;;;;;;;;;
;;; MATRIX FUNCTIONS ;;;
;;;;;;;;;;;;;;;;;;;;;;;;

(defcfun aa-dmat-inv1 :int
  (A dmat-inout))
