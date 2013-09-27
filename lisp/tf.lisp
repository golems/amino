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


;;; Matrices

(defcfun aa-tf-12 :void
  (tf transformation-matrix-t)
  (p0 vector-3-t)
  (p1 vector-3-t))
(defun tf-12 (tf p0 &optional (p1 (make-vec 3)))
  (aa-tf-12 tf p0 p1)
  p1)


(defcfun aa-tf-12chain :void
  (tf1 transformation-matrix-t)
  (tf2 transformation-matrix-t)
  (tf transformation-matrix-t))
(defun tf-12chain (tf1 tf2 &optional (tf (make-matrix 3 4)))
  (aa-tf-12chain tf1 tf2 tf)
  tf)

(defcfun aa-tf-xangle2rotmat :void
  (theta :double)
  (r rotation-matrix-t))
(defun tf-xangle2rotmat (theta &optional (r (make-matrix 3 3)))
  (aa-tf-xangle2rotmat theta r)
  r)

(defcfun aa-tf-yangle2rotmat :void
  (theta :double)
  (r rotation-matrix-t))
(defun tf-yangle2rotmat (theta &optional (r (make-matrix 3 3)))
  (aa-tf-yangle2rotmat theta r)
  r)

(defcfun aa-tf-zangle2rotmat :void
  (theta :double)
  (r rotation-matrix-t))
(defun tf-zangle2rotmat (theta &optional (r (make-matrix 3 3)))
  (aa-tf-zangle2rotmat theta r)
  r)

(defcfun aa-tf-12inv :void
  (tf transformation-matrix-t)
  (tf-i transformation-matrix-t))
(defun tf-inv (tf &optional (tf-i (make-matrix 3 4)))
  (aa-tf-12inv tf tf-i))

(defcfun aa-tf-12rel :void
  (tf1 transformation-matrix-t)
  (tf2 transformation-matrix-t)
  (tf transformation-matrix-t))
(defun tf-rel (tf1 tf2 &optional (tf (make-matrix 3 4)))
  (aa-tf-12rel tf1 tf2 tf)
  tf)

;;; Quaternions

(defmacro def-q2 ((c-fun lisp-fun) doc-string)
  `(progn (defcfun ,c-fun :void
            (x quaternion-t)
            (y quaternion-t))
          (defun ,lisp-fun (x &optional (y (make-matrix 4 1)))
            ,doc-string
            (,c-fun x y)
            y)))

(defmacro def-q3 ((c-fun lisp-fun) doc-string)
  `(progn (defcfun ,c-fun :void
            (x1 quaternion-t)
            (x2 quaternion-t)
            (y quaternion-t))
          (defun ,lisp-fun (x1 x2 &optional (y (make-matrix 4 1)))
            ,doc-string
            (,c-fun x1 x2 y)
            y)))

(def-q2 (aa-tf-qconj tf-qconj) "Quaternion conjugate")
(def-q2 (aa-tf-qinv tf-qinv) "Quaternion inverse")
(def-q2 (aa-tf-qnormalize2 tf-qnormalize) "Normalize unit quaternion")
(def-q2 (aa-tf-qminimize2 tf-qminimize) "Minimum angle unit quaternion")
(def-q2 (aa-tf-qexp tf-qexp) "Quaternion exponential")
(def-q2 (aa-tf-qln tf-qln) "Quaternion logarithm")

(def-q3 (aa-tf-qadd tf-qadd) "Add two quaternions")
(def-q3 (aa-tf-qsub tf-qsub) "Subtract quaternions")
(def-q3 (aa-tf-qmul tf-qmul) "Multiply quaternions")
(def-q3 (aa-tf-qcmul tf-qcmul) "Multiply conjugate of first arg with second arg")
(def-q3 (aa-tf-qmulc tf-qmulc) "Multiply first arg with conjugate of second arg")

(defcfun aa-tf-qslerp :void
  (r :double)
  (q0 quaternion-t)
  (q1 quaternion-t)
  (q quaternion-t))
(defun tf-qslerp (r q0 q1 &optional (q (make-vec 4)))
  "Quaternion spherical linear interpolation"
  (aa-tf-qslerp r q0 q1 q)
  q)


(defcfun aa-tf-qsvel :void
  (q0 quaternion-t)
  (w vector-3-t)
  (q1 quaternion-t))
(defun tf-qsvel (q0 w &optional (q1 (make-vec 4)))
  "Integrate unit quaternion rotational velocity"
  (aa-tf-qsvel q0 w q1)
  q1)

(defcfun aa-tf-rotmat2quat :void
  (r rotation-matrix-t)
  (q quaternion-t))
(defun tf-rotmat2quat (r &optional (q (make-matrix 4 1)))
  "Convert rotation matrix to quaternion"
  (aa-tf-rotmat2quat r q)
  q)

(defcfun aa-tf-quat2rotmat :void
  (q quaternion-t)
  (r rotation-matrix-t))
(defun tf-quat2rotmat (q &optional (r (make-matrix 3 3)))
  "Convert quaternion to rotation matrix"
  (aa-tf-quat2rotmat q r)
  r)

(defcfun aa-tf-xangle2quat :void
  (theta :double)
  (r quaternion-t))
(defun tf-xangle2quat (theta &optional (r (make-matrix 4 1)))
  "Convert rotation about x to unit quaternion"
  (aa-tf-xangle2quat theta r)
  r)

(defcfun aa-tf-yangle2quat :void
  (theta :double)
  (r quaternion-t))
(defun tf-yangle2quat (theta &optional (r (make-matrix 4 1)))
  "Convert rotation about y to unit quaternion"
  (aa-tf-yangle2quat theta r)
  r)

(defcfun aa-tf-zangle2quat :void
  (theta :double)
  (r quaternion-t))
(defun tf-zangle2quat (theta &optional (r (make-matrix 4 1)))
  "Convert rotation about z to unit quaternion"
  (aa-tf-zangle2quat theta r)
  r)


;;; Dual quaternion


(defmacro def-dq2 ((c-fun lisp-fun) doc-string)
  `(progn (defcfun ,c-fun :void
            (x dual-quaternion-t)
            (y dual-quaternion-t))
          (defun ,lisp-fun (x &optional (y (make-matrix 8 1)))
            ,doc-string
            (,c-fun x y)
            y)))

(defmacro def-dq3 ((c-fun lisp-fun) doc-string)
  `(progn (defcfun ,c-fun :void
            (x1 dual-quaternion-t)
            (x2 dual-quaternion-t)
            (y dual-quaternion-t))
          (defun ,lisp-fun (x1 x2 &optional (y (make-matrix 8 1)))
            ,doc-string
            (,c-fun x1 x2 y)
            y)))

(defcfun aa-tf-duqu-trans :void
  (d dual-quaternion-t)
  (x vector-3-t))
(defun tf-duqu-trans (d &optional (x (make-vec 3)))
  "Extract dual quaternion translation"
  (aa-tf-duqu-trans d x)
  x)

(defcfun aa-tf-qv2duqu :void
  (q quaternion-t)
  (v vector-3-t)
  (d dual-quaternion-t))
(defun tf-qv2duqu (q v &optional (d (make-matrix 8 1)))
  "Convert unit quaternion and translation vector to dual quaternion"
  (aa-tf-qv2duqu q v d)
  d)

(defcfun aa-tf-duqu2qv :void
  (d dual-quaternion-t)
  (q quaternion-t)
  (v vector-3-t))
(defun tf-duqu2qv (d &optional
                   (q (make-matrix 4 1))
                   (v (make-matrix 3 1)))
  "Convert dual quaternion to unit quaternion and translation vector"
  (aa-tf-duqu2qv d q v)
  (values q v))


(defcfun aa-tf-xxyz2duqu :void
  (x-angle :double)
  (x :double)
  (y :double)
  (z :double)
  (s dual-quaternion-t))
(defun tf-xxyz2duqu (x-angle x y z &optional (s (make-matrix 8 1)))
  "x-angle and vector to dual quaternion"
  (aa-tf-xxyz2duqu x-angle x y z s)
  s)


(defcfun aa-tf-yxyz2duqu :void
  (y-angle :double)
  (x :double)
  (y :double)
  (z :double)
  (s dual-quaternion-t))
(defun tf-yxyz2duqu (y-angle x y z &optional (s (make-matrix 8 1)))
  "y-angle and vector to dual quaternion"
  (aa-tf-xxyz2duqu y-angle x y z s)
  s)

(defcfun aa-tf-zxyz2duqu :void
  (z-angle :double)
  (x :double)
  (y :double)
  (z :double)
  (s dual-quaternion-t))
(defun tf-zxyz2duqu (z-angle x y z &optional (s (make-matrix 8 1)))
  "z-angle and vector to dual quaternion"
  (aa-tf-zxyz2duqu z-angle x y z s)
  s)


(defcfun aa-tf-duqu-conj :void
  (x dual-quaternion-t)
  (y dual-quaternion-t))
(defun tf-duqu-conj (x &optional (y (make-matrix 8 1)))
  "Dual quaternion conjugate"
  (aa-tf-duqu-conj x y)
  y)

(def-dq2 (aa-tf-duqu-inv tf-duqu-inv) "Dual quaternion inverse")
(def-dq2 (aa-tf-duqu-ln tf-duqu-ln) "Dual quaternion natural logarithm")
(def-dq2 (aa-tf-duqu-exp tf-duqu-exp) "Dual quaternion exponential")

(def-dq3 (aa-tf-duqu-mul tf-duqu-mul) "Dual quaternion multiply: c = a*b")
(def-dq3 (aa-tf-duqu-cmul tf-duqu-cmul) "Dual quaternion multiply: c = conj(a)*b")
(def-dq3 (aa-tf-duqu-mulc tf-duqu-mulc) "Dual quaternion multiply: c = a*conj(b)")


(defcfun aa-tf-duqu-normalize :void
  (y dual-quaternion-t))
(defun tf-duqu-normalize (x &optional (y (make-matrix 8 1)))
  "Dual quaternion normalization"
  (matrix-copy x y)
  (aa-tf-duqu-normalize y)
  y)

;;; Convenience

(defun tf-rotation (tf)
  (matrix-block tf 0 0 3 3))

(defun tf-translation (tf)
  (matrix-block tf 0 3 3 1))

(defun make-tf (&key r (x 0d0) (y 0d0) (z 0d0))
  (let* ((tf (make-matrix 3 4))
         (a (matrix-data tf)))
    (setf (aref a 9) (coerce x 'double-float)
          (aref a 10) (coerce y 'double-float)
          (aref a 11) (coerce z 'double-float))
    (if r
        (dotimes (i 9)
          (setf (aref a i)
                (aref (matrix-data r) i)))
        (setf (aref a 0) 1d0
              (aref a 4) 1d0
              (aref a 8) 1d0))
    tf))

;; (defun tf (a b)
;;   (declare (type matrix a b))
;;   (cond
;;     ((transformation-matrix-p a)
;;      (cond
;;        ((transformation-matrix-p b)
;;         (tf-12chain a b))
;;        ((vector-3-p b)
;;         (tf-12 a b))
;;        (t (error "Can't transform ~A * ~A" a b))))
;;     (t (error "Can't transform ~A * ~A" a b))))
