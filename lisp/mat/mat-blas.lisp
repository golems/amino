;;;; -*- mode: lisp -*-
;;;;
;;;; Copyright (c) 2013, Georgia Tech Research Corporation
;;;; Copyright (c) 2019, Colorado School of Mines
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


(in-package :amino)

;;;;;;;;;;;;;;;;;;;;
;;; LEVEL 1 BLAS ;;;
;;;;;;;;;;;;;;;;;;;;


(defun dscal (alpha x)
  (aa-dvec-scal alpha x)
  x)

(defun daxpy (alpha x y)
  (aa-dvec-axpy alpha x y)
  y)

(defun ddot (x y)
  (aa-dvec-dot x y))

(defun dnrm2 (x)
  (aa-dvec-nrm2 x))

;; ;;; asum
;; (def-blas-cfun ("dasum_" blas-dasum) :double
;;   (n blas-size-t)
;;   (x :vector))

;; (defun dasum (x)
;;   (with-foreign-vector (x inc-x n) x :input
;;     (blas-dasum n x inc-x)))


(defun dcopy (x &optional y)
  (let ((y (or y
               (make-vec (vec-length x)))))
    (aa-dvec-copy x y)
    y))

;;;;;;;;;;;;;;;;;;;;
;;; LEVEL 2 BLAS ;;;
;;;;;;;;;;;;;;;;;;;;


(defun dgemv (alpha a x beta y &key (transpose :no-transpose))
  (aa-dmat-gemv transpose alpha a x beta y)
  y)


;;;;;;;;;;;;;;;;;;;;
;;; LEVEL 3 BLAS ;;;
;;;;;;;;;;;;;;;;;;;;
