;;;; -*- mode: lisp -*-
;;;;
;;;; Copyright (c) 2013, Georgia Tech Research Corporation
;;;; Copyright (c) 2015, Rice University
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

(defgeneric rotation (x))
(defgeneric translation (x))

(defgeneric normalize (x))

(defmethod normalize ((x quaternion-translation))
  (make-quaternion-translation :quaternion (tf-qnormalize (quaternion-translation-quaternion x))
                               :translation (quaternion-translation-translation x)))


(defmethod rotation ((x quaternion-translation))
  (quaternion-translation-quaternion x))

(defmethod translation ((x quaternion-translation))
  (quaternion-translation-translation x))

(defgeneric quaternion (x))
(defgeneric rotation-matrix (x))

(defgeneric euler-zyx (x))
(defgeneric euler-rpy (x))

(defgeneric dual-quaternion (x))
(defgeneric quaternion-translation (x))
(defgeneric transformation-matrix (x))

(defgeneric dual-quaternion-2 (r x))
(defgeneric quaternion-translation-2 (r x))
(defgeneric transformation-matrix-2 (r x))

;;; Quaternion
(defmethod quaternion ((x quaternion)) x)

(defmethod quaternion ((x (eql nil)))
  (quaternion* 0d0 0d0 0d0 1d0))

(defmethod quaternion ((x number))
  (quaternion* 0d0 0d0 0d0 x))

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

(defmethod quaternion ((x array))
  (check-type x (array double-float (4)))
  (quaternion* (aref x 0)
               (aref x 1)
               (aref x 2)
               (aref x 3)))

(defmethod quaternion ((x cons))
  (unless (= 4 (length x))
    (error "Invalid length for quaternion"))
  (apply #'quaternion* x))

(defmethod quaternion ((x x-angle))
  (tf-xangle2quat (principal-angle-value x)))
(defmethod quaternion ((x y-angle))
  (tf-yangle2quat (principal-angle-value x)))
(defmethod quaternion ((x z-angle))
  (tf-zangle2quat (principal-angle-value x)))

;;; Axis-Angle
(defgeneric axis-angle (x))

(defmethod axis-angle ((x quaternion))
  (tf-quat2axang x))


(defmethod axis-angle ((x x-angle))
  (axis-angle* 1 0 0 (principal-angle-value x)))
(defmethod axis-angle ((x y-angle))
  (axis-angle* 0 1 0 (principal-angle-value x)))
(defmethod axis-angle ((x z-angle))
  (axis-angle* 0 0 1 (principal-angle-value x)))

;;; Translation

(defgeneric vec3 (x))

(defmethod vec3 ((x vec3))
  x)

(defmethod vec3 ((x (eql nil)))
  (vec3* 0 0 0))

(defmethod vec3 ((x cons))
  (unless (= 3 (length x))
    (error "Invalid length for vec3"))
  (apply #'vec3* x))

(defmethod vec3 ((x array))
  (check-type x (array t (3)))
  (vec3* (aref x 0)
         (aref x 1)
         (aref x 2)))

(defmethod vec3 ((x cons))
  (vec3* (elt x 0)
         (elt x 1)
         (elt x 2)))

;;; Euler ZYX

(defmethod euler-zyx ((x (eql nil)))
  (euler-zyx* 0d0 0d0 0d0))

(defmethod euler-zyx ((x quaternion))
  (tf-quat2eulerzyx x))

(defmethod euler-zyx ((x array))
  (euler-zyx* (aref x 0)
              (aref x 1)
              (aref x 2)))

(defmethod euler-zyx ((x cons))
  (apply #'euler-zyx* x))

(defmethod euler-zyx ((x x-angle))
  (euler-zyx* 0d0
              0d0
              (x-angle-value x)))

(defmethod euler-zyx ((x y-angle))
  (euler-zyx* 0d0
              (y-angle-value x)
              0d0))

(defmethod euler-zyx ((x z-angle))
  (euler-zyx*  (z-angle-value x)
               0d0
               0d0))

;;; Euler RPY
;; These are just reversed ZYX angles

(defmethod euler-rpy ((x array))
  (euler-rpy* (aref x 0)
              (aref x 1)
              (aref x 2)))

(defmethod euler-rpy ((x cons))
  (apply #'euler-rpy* x))

(defmethod euler-rpy ((x (eql nil)))
  (euler-zyx x))
(defmethod euler-rpy ((x quaternion))
  (euler-zyx x))
(defmethod euler-rpy ((x x-angle))
  (euler-zyx x))
(defmethod euler-rpy ((x y-angle))
  (euler-zyx x))
(defmethod euler-rpy ((x z-angle))
  (euler-zyx x))

;;; Dual-Quaternion
(defmethod dual-quaternion ((x dual-quaternion)) x)

(defmethod dual-quaternion ((x (eql nil)))
  (make-dual-quaternion :data (vec 0d0 0d0 0d0 1d0
                                   0d0 0d0 0d0 0d0)))

(defmethod dual-quaternion ((x quaternion))
  (make-dual-quaternion :data (replace (make-vec 8) (quaternion-data x))))


(defmethod dual-quaternion ((x quaternion-translation))
  (tf-qv2duqu (quaternion-translation-quaternion x)
              (quaternion-translation-translation x)))

(defmethod dual-quaternion ((x array))
  (assert (= 8 (length x)))
  (make-dual-quaternion :data (replace (make-vec 8) x)))

(defmethod dual-quaternion-2 ((r quaternion) (x vec3))
  (tf-qv2duqu r x))

(defmethod dual-quaternion-2 ((r principal-angle) (x vec3))
  (tf-qv2duqu (quaternion r) x))

(defmethod dual-quaternion-2 ((r euler-angle) (x vec3))
  (tf-qv2duqu (quaternion r) x))

(defmethod dual-quaternion-2 ((r axis-angle) (x vec3))
  (tf-qv2duqu (quaternion r) x))

(defmethod dual-quaternion-2 ((r matrix) (x vec3))
  (tf-qv2duqu (quaternion r) x))

;;; Quaternion-Translation
(defmethod quaternion-translation ((x quaternion-translation))
  x)


(defmethod quaternion-translation ((x array))
  (check-type x (array double-float (7)))
  (make-quaternion-translation :quaternion (make-quaternion :data (subseq x 0 4))
                               :translation (vec3* (aref x 4) (aref x 5) (aref x 6))))


(defmethod quaternion-translation ((x (eql nil)))
  (make-quaternion-translation :quaternion (quaternion nil)
                               :translation (make-vec3 :data (vec 0d0 0d0 0d0))))

(defmethod quaternion-translation ((x dual-quaternion))
  (tf-duqu2qutr x))


(defmethod quaternion-translation-2 ((r quaternion) (x vec3))
  (make-quaternion-translation :quaternion r
                               :translation x))

(defmethod quaternion-translation-2 (r x)
  (make-quaternion-translation :quaternion (quaternion r)
                               :translation (vec3 x)))

(defun tf (rotation translation)
  (quaternion-translation-2 rotation translation))

(defmethod vec-array ((obj quaternion-translation) &optional (array (make-vec 7)) (start 0))
  (replace array (real-array-data (quaternion-translation-quaternion obj)) :start1 start)
  (replace array (real-array-data (quaternion-translation-translation obj)) :start1 (+ start 4)))

(defmethod matrix->list ((matrix quaternion-translation))
  (list (matrix->list (quaternion-translation-quaternion matrix))
        (matrix->list (quaternion-translation-translation matrix))))

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

(defmethod g* ((a number) (b vec3))
  (make-vec3 :data (dscal-copy a (vec3-data b))))

(defmethod g* ((a vec3) (b number))
  (g* b a))

(defmethod g* ((a quaternion) (b quaternion))
  (tf-qmul a b))

(defmethod g* ((a number) (b quaternion))
  (make-quaternion :data (dscal-copy a b)))

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

(defmethod g* ((a dual-quaternion) (b quaternion-translation))
  (tf-duqu-mul a (dual-quaternion b)))

;; Transformations
(defgeneric transform (tf point))

(defmethod transform ((a quaternion) (b vec3))
  (tf-qrot a b))

(defmethod transform ((a quaternion-translation) (b vec3))
  (let ((c (make-vec3)))
    (aa-tf-tf-qv (quaternion-translation-quaternion a)
                 (quaternion-translation-translation a)
                 b
                 c)
    c))

(defmethod transform ((a matrix) (b vec3))
  (let ((rows (matrix-rows a))
        (cols (matrix-cols a)))
    (assert (= 3 rows))
    (ecase cols
      (3 (tf-9 a b))
      (4 (tf-12 a b)))))

(defmethod transform ((a principal-angle) (b vec3))
  (transform (rotation-matrix a) b))

(defmethod transform ((a euler-angle) (b vec3))
  (transform (rotation-matrix a) b))

(defun g-chain (&rest args)
  (reduce #'g* args))

;; Transformations
(defgeneric c-float-array (x))

(defmethod c-float-array ((x list))
  (format t "~&{~{~F~^,~}}"
          x))

(defmethod c-float-array ((x quaternion-translation))
  (c-float-array
   (append (map 'list #'identity (real-array-data (quaternion-translation-quaternion x)))
           (map 'list #'identity (real-array-data (quaternion-translation-translation x))))))

;; Inverse
(defgeneric inverse (x))

(defmethod inverse ((x quaternion-translation))
  (let ((qc (make-quaternion))
        (vc (make-vec3)))
    (aa-tf-qv-conj (quaternion-translation-quaternion x)
                   (quaternion-translation-translation x)
                   qc vc)
    (quaternion-translation-2 qc vc)))


;;; Tagged TFs

(defmethod g* ((a tf-tag) (b tf-tag))
  (assert (eql (tf-tag-child a)
               (tf-tag-parent b)))
  (tf-tag (tf-tag-parent a)
          (g* (tf-tag-tf a)
              (tf-tag-tf b))
          (tf-tag-child b)))
