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

;;; Geometric types ;;;;



(defun expand-type (value var body type)
  (with-gensyms (x y ptr0)
    `(let ((,x ,value))
       (let ((,y (etypecase ,x
                   (matrix ,x)
                   ((simple-array double-float) (wrap-col-vector ,x)))))
         (check-type ,y ,type)
         (with-pointer-to-vector-data (,ptr0 (matrix-data ,y))
           (let ((,var (inc-pointer ,ptr0 (* 8 (matrix-offset ,y)))))
             ,@body))))))


;;; Transformation Matrix
(defun transformation-matrix-p (x)
  (and (= 3 (matrix-rows x))
       (<= 4 (matrix-cols x))
       (= 3 (matrix-stride x))
       (<= 12 (- (length (matrix-data x))
                 (matrix-offset x)))
       (eq (array-element-type (matrix-data x))
           'double-float)))
(deftype transformation-matrix ()
  '(and matrix
    (satisfies transformation-matrix-p)))
(define-foreign-type transformation-matrix-t ()
  ()
  (:simple-parser transformation-matrix-t)
  (:actual-type :pointer))
(defmethod expand-to-foreign-dyn (value var body (type transformation-matrix-t))
  (expand-type value var body 'transformation-matrix))

;;; Rotation Matrix
(defun rotation-matrix-p (x)
  (and (= 3 (matrix-rows x))
       (<= 3 (matrix-cols x))
       (= 3 (matrix-stride x))
       (<= 9 (- (length (matrix-data x))
                (matrix-offset x)))
       (eq (array-element-type (matrix-data x))
           'double-float)))
(deftype rotation-matrix ()
  '(and matrix
    (satisfies rotation-matrix-p)))


(define-foreign-type rotation-matrix-t ()
  ()
  (:simple-parser rotation-matrix-t)
  (:actual-type :pointer))
(defmethod expand-to-foreign-dyn (value var body (type rotation-matrix-t))
  (expand-type value var body 'rotation-matrix))

;;; Point 3
(defun point-3-p (x)
  (matrix-vector-n-p x 3 1))
(deftype point-3 ()
  '(and matrix
    (satisfies point-3-p)))
(define-foreign-type point-3-t ()
  ()
  (:simple-parser point-3-t)
  (:actual-type :pointer))
(defmethod expand-to-foreign-dyn (value var body (type point-3-t))
  (expand-type value var body 'point-3))

;;; Quaternion
(defun quaternion-p (x)
  (matrix-vector-n-p x 4 1))
(deftype quaternion ()
  '(and matrix
    (satisfies quaternion-p)))
(define-foreign-type quaternion-t ()
  ()
  (:simple-parser quaternion-t)
  (:actual-type :pointer))
(defmethod expand-to-foreign-dyn (value var body (type quaternion-t))
  (expand-type value var body 'quaternion))

;;; Dual Quaternion
(defun dual-quaternion-p (x)
  (matrix-vector-n-p x 8 1))
(deftype dual-quaternion ()
  '(and matrix
    (satisfies dual-quaternion-p)))
(define-foreign-type dual-quaternion-t ()
  ()
  (:simple-parser dual-quaternion-t)
  (:actual-type :pointer))
(defmethod expand-to-foreign-dyn (value var body (type dual-quaternion-t))
  (expand-type value var body 'dual-quaternion))


;;; Wrappers


(defcfun aa-tf-12 :void
  (tf transformation-matrix-t)
  (p0 point-3-t)
  (p1 point-3-t))
(defun tf-12 (tf p0 &optional (p1 (make-matrix 3 1)))
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

(defcfun aa-tf-qinv :void
  (q quaternion-t)
  (qi quaternion-t))
(defun tf-qinv (q &optional (q-i (make-matrix 4 1)))
  (aa-tf-qinv q q-i)
  q-i)


(defcfun aa-tf-qnormalize :void
  (q quaternion-t))
(defun tf-qnorm (q &optional (q-n (make-matrix 4 1)))
  (replace (matrix-data q-n) (matrix-data q))
  (aa-tf-qnormalize q-n)
  q-n)

(defcfun aa-tf-rotmat2quat :void
  (r rotation-matrix-t)
  (q quaternion-t))
(defun tf-rotmat2quat (r &optional (q (make-matrix 4 1)))
  (aa-tf-rotmat2quat r q)
  q)


(defcfun aa-tf-qslerp :void
  (r :double)
  (q0 quaternion-t)
  (q1 quaternion-t)
  (q quaternion-t))
(defun tf-qslerp (r q0 q1 &optional (q (make-matrix 4 1)))
  (aa-tf-qslerp r q0 q1 q)
  q)


;;; Dual quaternion
(defcfun aa-tf-duqu-trans :void
  (d dual-quaternion-t)
  (x point-3-t))
(defun tf-duqu-trans (d &optional (x (make-matrix 3 1)))
  "Extract dual quaternion translation"
  (aa-tf-duqu-trans d x)
  x)


(defcfun aa-tf-qv2duqu :void
  (q quaternion-t)
  (v point-3-t)
  (d dual-quaternion-t))
(defun tf-qv2duqu (q v &optional (d (make-matrix 8 1)))
  "Convert unit quaternion and translation vector to dual quaternion"
  (aa-tf-qv2duqu q v d)
  d)

(defcfun aa-tf-duqu2qv :void
  (d dual-quaternion-t)
  (q quaternion-t)
  (v point-3-t))
(defun tf-duqu2qv (d &optional
                   (q (make-matrix 4 1))
                   (v (make-matrix 3 1)))
  "Convert dual quaternion to unit quaternion and translation vector"
  (aa-tf-duqu2qv d q v)
  (values q v))

(defcfun aa-tf-duqu-conj :void
  (x dual-quaternion-t)
  (y dual-quaternion-t))
(defun tf-duqu-conj (x &optional (y (make-matrix 8 1)))
  "Dual quaternion conjugate"
  (aa-tf-duqu-conj x y)
  y)

(defcfun aa-tf-duqu-inv :void
  (x dual-quaternion-t)
  (y dual-quaternion-t))
(defun tf-duqu-inv (x &optional (y (make-matrix 8 1)))
  "Dual quaternion inverse"
  (aa-tf-duqu-inv x y)
  y)

(defcfun aa-tf-duqu-normalize :void
  (y dual-quaternion-t))
(defun tf-duqu-normalize (x &optional (y (make-matrix 8 1)))
  "Dual quaternion normalization"
  (matrix-copy x y)
  (aa-tf-duqu-normalize y)
  y)

(defcfun aa-tf-duqu-mul :void
  (a dual-quaternion-t)
  (b dual-quaternion-t)
  (c dual-quaternion-t))
(defun tf-duqu-mul (a b &optional (c (make-matrix 8 1)))
  "Dual quaternion multiply: c = a*b"
  (aa-tf-duqu-mul a b c)
  c)

(defcfun aa-tf-duqu-cmul :void
  (a dual-quaternion-t)
  (b dual-quaternion-t)
  (c dual-quaternion-t))
(defun tf-duqu-cmul (a b &optional (c (make-matrix 8 1)))
  "Dual quaternion multiply: c = conj(a)*b"
  (aa-tf-duqu-cmul a b c)
  c)

(defcfun aa-tf-duqu-mulc :void
  (a dual-quaternion-t)
  (b dual-quaternion-t)
  (c dual-quaternion-t))
(defun tf-duqu-mulc (a b &optional (c (make-matrix 8 1)))
  "Dual quaternion multiply: c = a*conj(b)"
  (aa-tf-duqu-mulc a b c)
  c)

(defcfun aa-tf-duqu-ln :void
  (x dual-quaternion-t)
  (y dual-quaternion-t))
(defun tf-duqu-ln (x &optional (y (make-matrix 8 1)))
  "Dual quaternion logarithm"
  (aa-tf-duqu-ln x y)
  y)

(defcfun aa-tf-duqu-exp :void
  (x dual-quaternion-t)
  (y dual-quaternion-t))
(defun tf-duqu-exp (x &optional (y (make-matrix 8 1)))
  "Dual quaternion exponential"
  (aa-tf-duqu-exp x y)
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

(defun tf (a b)
  (declare (type matrix a b))
  (cond
    ((transformation-matrix-p a)
     (cond
       ((transformation-matrix-p b)
        (tf-12chain a b))
       ((point-3-p b)
        (tf-12 a b))
       (t (error "Can't transform ~A * ~A" a b))))
    (t (error "Can't transform ~A * ~A" a b))))
