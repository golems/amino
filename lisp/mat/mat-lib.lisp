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

;; AA_API void
;; aa_dmat_row_vec( const struct aa_dmat *src, size_t row, struct aa_dvec *dst );

(defcfun ("aa_dmat_row_vec" %aa-dmat-row-vec) :void
  (mat :pointer)
  (row size-t )
  (vec :pointer))

(defcfun ("aa_dmat_col_vec" %aa-dmat-col-vec) :void
  (mat :pointer)
  (col size-t )
  (vec :pointer))


;;;;;;;;;;;;;;;;;;;;
;;; BLAS Level 1 ;;;
;;;;;;;;;;;;;;;;;;;;

(defcfun aa-dvec-swap :void
  (x dvec-inout)
  (y dvec-inout))

(defcfun aa-dvec-scal :void
  (a amino-ffi::coercible-double)
  (x dvec-inout))

(defcfun aa-dvec-copy :void
  (x dvec-input)
  (y dvec-output))

(defcfun ("aa_dvec_copy" %aa-dvec-copy-to-foreign) :void
  (x dvec-input)
  (y :pointer))

(defcfun ("aa_dvec_copy" %aa-dvec-copy-from-foreign) :void
  (x :pointer)
  (y dvec-input))


(defun %aa-dvec-copy-foreign (pointer)
  (let ((v (make-vec (foreign-slot-value pointer '(:struct aa-dvec) 'len))))
    (%aa-dvec-copy-from-foreign pointer v)
    v))

(defcfun aa-dvec-dot :double
  (x dvec-input)
  (y dvec-input))

(defcfun aa-dvec-ssd :double
  (x dvec-input)
  (y dvec-input))

(defcfun aa-dvec-nrm2 :double
  (x dvec-input))

(defcfun aa-dvec-axpy :void
  (a amino-ffi::coercible-double)
  (x dvec-input)
  (y dvec-inout))

;;;;;;;;;;;;;;;;;;;;
;;; BLAS Level 2 ;;;
;;;;;;;;;;;;;;;;;;;;

(defcfun aa-dmat-gemv :void
  (trans transpose-t)
  (alpha amino-ffi::coercible-double)
  (A dmat-input)
  (x dvec-input)
  (beta amino-ffi::coercible-double)
  (y dvec-inout))

;;;;;;;;;;;;;;;;;;;;
;;; BLAS Level 3 ;;;
;;;;;;;;;;;;;;;;;;;;

(defcfun aa-dmat-gemm :void
  (trans-A transpose-t)
  (trans-B transpose-t)
  (alpha amino-ffi::coercible-double)
  (A dmat-input)
  (B dmat-input)
  (beta amino-ffi::coercible-double)
  (C dmat-inout))


;;;;;;;;;;;;;;;;;;;;;;;;
;;; MATRIX FUNCTIONS ;;;
;;;;;;;;;;;;;;;;;;;;;;;;

(defcfun aa-dmat-inv1 :int
  (A dmat-inout))


(defcfun aa-dmat-copy :void
  (x dmat-input)
  (y dmat-output))
