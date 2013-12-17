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


(defgeneric quaternion (x))
(defgeneric dual-quaternion (x))

(defgeneric rotation-matrix (x))
(defgeneric transformation-matrix (x))


;;; Quaternion
(defmethod quaternion ((x axis-angle))
  (tf-axang2quat x))

(defmethod quaternion ((x matrix))
  (assert (rotation-matrix-p x))
  (tf-rotmat2quat x))

(defmethod quaternion ((x euler-zyx))
  (let ((data (euler-angle-data x)))
    (tf-eulerzyx2quat (aref data 0)
                      (aref data 1)
                      (aref data 2))))

(defmethod quaternion ((x x-angle))
  (tf-xangle2quat (principal-angle-value x)))
(defmethod quaternion ((x y-angle))
  (tf-yangle2quat (principal-angle-value x)))
(defmethod quaternion ((x z-angle))
  (tf-zangle2quat (principal-angle-value x)))

;;; Dual-Quaternion
(defmethod dual-quaternion ((x array))
  (assert (= 8 (length x)))
  (make-dual-quaternion :data (replace (make-vec 8) x)))

;;; Quaternion-Translation
(defmethod quaternion-translation ((x dual-quaternion))
  (tf-duqu2qutr x))

;;; Rotation Matrix
(defmethod rotation-matrix ((x quaternion))
  (tf-quat2rotmat x))
(defmethod rotation-matrix ((x axis-angle))
  (tf-axang2rotmat x))
(defmethod rotation-matrix ((x x-angle))
  (tf-xangle2rotmat (principal-angle-value x)))
(defmethod rotation-matrix ((x y-angle))
  (tf-yangle2rotmat (principal-angle-value x)))
(defmethod rotation-matrix ((x z-angle))
  (tf-zangle2rotmat (principal-angle-value x)))

;;; Transformation Matrix


;;; Multiplies

(defmethod g* ((a quaternion) (b quaternion))
  (tf-qmul a b))

(defmethod g* ((a number) (b quaternion))
  (make-quaternion :data (dscal (coerce a 'double-float)
                                (vec-copy (quaternion-data b)))))

(defmethod g* ((a quaternion) (b axis-angle))
  (tf-qmul a (quaternion b)))

(defmethod g* ((a quaternion) (b principal-angle))
  (tf-qmul a (quaternion b)))

(defmethod g* ((a quaternion) (b euler-angle))
  (tf-qmul a (quaternion b)))



(defmethod g* ((a dual-quaternion) (b dual-quaternion))
  (tf-duqu-mul a b))


(defmethod g* ((a quaternion-translation) (b quaternion-translation))
  (tf-qutr-mul a b))

;; Transformations
(defgeneric transform (tf point))

(defmethod transform ((a quaternion) (b point3))
  (tf-qrot a b))

(defmethod transform ((a matrix) (b point3))
  (let ((rows (matrix-rows a))
        (cols (matrix-cols a)))
    (assert (= 3 rows))
    (ecase cols
      (3 (tf-9 a b))
      (4 (tf-12 a b)))))

(defmethod transform ((a principal-angle) (b point3))
  (transform (rotation-matrix a) b))

(defmethod transform ((a euler-angle) (b point3))
  (transform (rotation-matrix a) b))
