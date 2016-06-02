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


(in-package :amino)

;;;;;;;;;;;;;;;;;;;;
;;; LEVEL 1 BLAS ;;;
;;;;;;;;;;;;;;;;;;;;

;;; scal
(def-blas-cfun ("dscal_" blas-dscal) :void
  (n blas-size-t)
  (alpha :double)
  (x :vector))

(defun dscal (alpha x)
  (with-foreign-vector (x inc-x n) x :inout
    (blas-dscal n alpha x inc-x)))

;;; axpy
(def-blas-cfun ("daxpy_" blas-daxpy) :void
  (n blas-size-t)
  (alpha :double)
  (x :vector)
  (y :vector))

(defun daxpy (alpha x y)
  (with-foreign-vector (x inc-x n-x) x :input
    (with-foreign-vector (y inc-y n-y) y :inout
      (check-matrix-dimensions n-x n-y)
      (blas-daxpy n-x alpha x inc-x y inc-y))))

;;; dot
(def-blas-cfun ("ddot_" blas-ddot) :double
  (n blas-size-t)
  (x :vector)
  (y :vector))

(defun ddot (x y)
  (with-foreign-vector (x inc-x n-x) x :input
    (with-foreign-vector (y inc-y n-y) y :input
      (check-matrix-dimensions n-x n-y)
      (blas-ddot n-x x inc-x y inc-y))))

;;; nrm2
(def-blas-cfun ("dnrm2_" blas-dnrm2) :double
  (n blas-size-t)
  (x :vector))

(defun dnrm2 (x)
  (with-foreign-vector (x inc-x n) x :input
    (blas-dnrm2 n x inc-x)))

;;; asum
(def-blas-cfun ("dasum_" blas-dasum) :double
  (n blas-size-t)
  (x :vector))

(defun dasum (x)
  (with-foreign-vector (x inc-x n) x :input
    (blas-dasum n x inc-x)))

;;; copy
(def-blas-cfun ("dcopy_" blas-dcopy) :void
  (n blas-size-t)
  (x :vector)
  (y :vector))

(defun dcopy (x &optional y)
  (let ((y (or y
               (make-vec (vec-length x)))))
    (with-foreign-vector (x incx len-x) x :input
      (with-foreign-vector (y incy len-y) y :output
        (check-matrix-dimensions len-x len-y)
        (blas-dcopy len-x x incx y incy)))
    y))

;;;;;;;;;;;;;;;;;;;;
;;; LEVEL 2 BLAS ;;;
;;;;;;;;;;;;;;;;;;;;

;;; gemv
(def-blas-cfun ("dgemv_" blas-dgemv) :void
  (trans transpose-t)
  (m blas-size-t)
  (n blas-size-t)
  (alpha :double)
  (a :matrix)
  (x :vector)
  (beta :double)
  (y :vector))

(defun dgemv (alpha a x beta y &key transpose)
  (with-foreign-matrix (a ld-a m n) a :input
    (with-foreign-vector (x inc-x n-x) x :input
      (with-foreign-vector (y inc-y n-y) y :inout
        (check-matrix-dimensions m n-y)
        (check-matrix-dimensions n n-x)
        (blas-dgemv transpose m n
                    alpha a ld-a
                    x inc-x
                    beta y inc-y)))))

;;;;;;;;;;;;;;;;;;;;
;;; LEVEL 3 BLAS ;;;
;;;;;;;;;;;;;;;;;;;;
