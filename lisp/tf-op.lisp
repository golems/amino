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

(defgeneric rotation (x)
  (:documentation "Extract the rotation part of X"))

(defgeneric translation (x)
  (:documentation "Extract the translation part of X"))

(defmethod normalize ((x quaternion-translation))
  "Make the rotation component a unit quaternion.

Translation is not altered."
  (make-quaternion-translation :quaternion (tf-qnormalize (quaternion-translation-quaternion x))
                               :translation (quaternion-translation-translation x)))


(defmethod rotation ((x quaternion-translation))
"Return the rotation quaternion part."
  (quaternion-translation-quaternion x))

(defmethod translation ((x quaternion-translation))
"Return the translation quaternion part."
  (quaternion-translation-translation x))

(defgeneric quaternion (x)
  (:documentation "Convert orientation X to a quaternion."))

(defgeneric rotation-matrix (x)
  (:documentation "Convert orientation X to a rotation matrix."))

(defgeneric euler-zyx (x)
  (:documentation "Convert orientation X to Euler angle ZYX form."))

(defgeneric euler-rpy (x)
  (:documentation "Convert orientation X to Euler angle roll-pitch-yaw form."))

(defgeneric dual-quaternion (x)
  (:documentation "Convert transform X to dual quaternion form."))

(defgeneric quaternion-translation (x)
  (:documentation "Convert transform X to quaternion-translation form."))

(defgeneric transformation-matrix (x)
  (:documentation "Convert transform X to a trasnformation matrix form."))

(defgeneric dual-quaternion-2 (r x)
  (:documentation "Convert rotation R and translation X to dual quaternion form."))

(defgeneric quaternion-translation-2 (r x)
  (:documentation "Convert rotation R and translation X to quaternion-translation form."))

(defgeneric transformation-matrix-2 (r x)
  (:documentation "Convert rotation R and translation X to transformation matrix form."))

(defun quaternion-translation* (r x)
  "Convert transform X to quaternion-translation form."
  (quaternion-translation-2 r x))

(defgeneric vec3 (x)
  (:documentation "Convert X to a 3-element vector."))

(defmethod generic- ((a vec3) (b vec3))
  (with-vec3 (a-x a-y a-z) a
    (with-vec3 (b-x b-y b-z) b
      (vec3* (- a-x b-x)
             (- a-y b-y)
             (- a-z b-z)))))

;;; TF Interface
(defun make-tf (&key
                  (quaternion (quaternion* 0d0 0d0 0d0 1d0))
                  (translation (vec3* 0d0 0d0 0d0)))
  "Create a transform"
  (make-quaternion-translation :quaternion quaternion
                               :translation translation))


(defun tf-quaternion (transform)
  "Return the rotation part as a quaternion"
  (quaternion-translation-quaternion transform))

(defun tf-translation (transform)
  "Return the translation part."
  (quaternion-translation-translation transform))

(defun tf-inverse (transform &optional (inverse (make-tf)))
  "Compute the inverse of the transform."
  (aa-tf-qv-conj (tf-quaternion transform)
                 (tf-translation transform)
                 (tf-quaternion inverse)
                 (tf-translation inverse))
  inverse)

(deftype tf ()
  'quaternion-translation)


(defstruct tf-readable
  rotation translation)

(defun tf-readable* (rotation translation)
  (make-tf-readable :rotation (euler-zyx rotation)
                    :translation (vec3 translation)))

(defun tf-readable (tf)
  (let* ((tf (tf tf)))
    (tf-readable* (tf-quaternion tf)
                  (tf-translation tf))))

(defmethod print-object ((tf tf-readable) stream)
  (let ((e (euler-zyx-data (tf-readable-rotation tf)))
        (v (vec3-data (tf-readable-translation tf))))
    (write `(tf* (euler-rpy* ,(aref e 2)
                             ,(aref e 1)
                             ,(aref e 1))
                 (vec3* ,(aref v 0)
                        ,(aref v 1)
                        ,(aref v 2)))
           :stream stream)))

(defmethod print-object ((tf quaternion-translation) stream)
  (with-quaternion (qx qy qz qw) (tf-quaternion tf)
    (with-vec3  (vx vy vz) (tf-translation tf)
      (write `(tf* (quaternion* ,qx ,qy ,qz ,qw)
                   (vec3* ,vx ,vy ,vz))
             :stream stream))))


(defun tf (transform)
  "Convert TRANSFORM to TF type"
  (quaternion-translation transform))


(defun tf-array (tf &optional (array (make-vec 7)))
  "Convert transform object to an array."
  (let ((tf (tf tf)))
    (replace array (quaternion-data (tf-quaternion tf)))
    (replace array (vec3-data (tf-translation tf)) :start1 4)))

(defun tf* (rotation translation)
  "Convert ROTATION and TRANSLATION to TF type"
  (make-tf :quaternion (quaternion rotation)
           :translation (vec3 translation)))

(defun tf-mul (tf-0 tf-1 &optional (result (make-tf)))
  "Multiply TF-0 and TF-1, storing in RESULT."
  (tf-qutr-mul tf-0 tf-1 result))

(defun tf-copy (transform &optional (result (make-tf)))
  "Deeply copy TRANFORM into RESULT."
  (deep-copy-quaternion (tf-quaternion transform)
                        (tf-quaternion result))
  (deep-copy-vec3 (tf-translation transform)
                  (tf-translation result))
  result)

(defun tf-normalize (transform &optional (result (make-tf)))
  "Normalize TRANSFORM, storing into RESULT."
  (deep-copy-vec3 (tf-translation transform)
                  (tf-translation result))
  (tf-qnormalize (tf-quaternion transform)
                 (tf-quaternion result))
  result)

;;; Quaternion
(defmethod quaternion ((x quaternion)) x)

(defmethod quaternion ((x (eql nil)))
  (quaternion* 0d0 0d0 0d0 1d0))

(defmethod quaternion ((x real))
  (quaternion* 0d0 0d0 0d0 x))

(defmethod quaternion ((x complex))
  (quaternion* (imagpart x) 0d0 0d0 (realpart x)))

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
  (assert (= (length x) 4))
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

(defmethod inverse ((x quaternion))
  (tf-qinv x))

(defmethod inverse ((x principal-angle))
  (tf-qinv (quaternion x)))

;;; Axis-Angle
(defgeneric axis-angle (x)
  (:documentation "Convert orientation X to axis-angle form."))

(defmethod axis-angle ((x quaternion))
  (tf-quat2axang x))


(defmethod axis-angle ((x x-angle))
  (axis-angle* 1 0 0 (principal-angle-value x)))
(defmethod axis-angle ((x y-angle))
  (axis-angle* 0 1 0 (principal-angle-value x)))
(defmethod axis-angle ((x z-angle))
  (axis-angle* 0 0 1 (principal-angle-value x)))

;;; Translation


(defmethod vec3 ((x vec3))
  x)

(defmethod vec3 ((x (eql nil)))
  (vec3* 0 0 0))

(defmethod vec3 ((x cons))
  (unless (= 3 (length x))
    (error "Invalid length for vec3"))
  (apply #'vec3* x))

(defmethod vec3 ((x array))
  (assert (= (length x) 3))
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

(defmethod quaternion-translation ((x tf-readable))
  (make-quaternion-translation :quaternion (quaternion (tf-readable-rotation x))
                               :translation (tf-readable-translation x)))

(defmethod quaternion-translation ((x array))
  (check-type x (array double-float (7)))
  (make-quaternion-translation :quaternion (make-quaternion :data (subseq x 0 4))
                               :translation (vec3* (aref x 4) (aref x 5) (aref x 6))))


(defmethod quaternion-translation ((x (eql nil)))
  (make-quaternion-translation :quaternion (quaternion nil)
                               :translation (identity-vec3)))

(defmethod quaternion-translation ((x dual-quaternion))
  (tf-duqu2qutr x))

(defmethod quaternion-translation ((x vec3))
  (make-quaternion-translation :quaternion (identity-quaternion)
                               :translation x))

(defmethod quaternion-translation ((x quaternion))
  (make-quaternion-translation :quaternion x
                               :translation (identity-vec3)))

(defmethod quaternion-translation ((x x-angle))
  (make-quaternion-translation :quaternion (tf-xangle2quat (principal-angle-value x))
                               :translation (identity-vec3)))

(defmethod quaternion-translation ((x y-angle))
  (make-quaternion-translation :quaternion (tf-yangle2quat (principal-angle-value x))
                               :translation (identity-vec3)))

(defmethod quaternion-translation ((x z-angle))
  (make-quaternion-translation :quaternion (tf-yangle2quat (principal-angle-value x))
                               :translation (identity-vec3)))

(defmethod quaternion-translation ((x axis-angle))
  (make-quaternion-translation :quaternion (tf-axang2quat x)
                               :translation (identity-vec3)))

(defmethod quaternion-translation ((x euler-zyx))
  (let ((data (euler-angle-data x)))
    (make-quaternion-translation :quaternion (tf-eulerzyx2quat (aref data 0)
                                                               (aref data 1)
                                                               (aref data 2))
                                 :translation (identity-vec3))))

(defmethod quaternion-translation-2 ((r quaternion) (x vec3))
  (make-quaternion-translation :quaternion r
                               :translation x))

(defmethod quaternion-translation-2 (r x)
  (make-quaternion-translation :quaternion (quaternion r)
                               :translation (vec3 x)))

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


(defmethod generic* ((a quaternion) (b quaternion))
  (tf-qmul a b))

(defmethod generic* ((a quaternion) (b axis-angle))
  (tf-qmul a (quaternion b)))

(defmethod generic* ((a quaternion) (b principal-angle))
  (tf-qmul a (quaternion b)))

(defmethod generic* ((a principal-angle) (b principal-angle))
  (tf-qmul (quaternion a) (quaternion b)))

(defmethod generic* ((a quaternion) (b euler-angle))
  (tf-qmul a (quaternion b)))

(defmethod generic* ((a dual-quaternion) (b dual-quaternion))
  (tf-duqu-mul a b))

(defmethod generic* ((a quaternion-translation) (b quaternion-translation))
  (tf-qutr-mul a b))

(defmethod generic* ((a dual-quaternion) (b quaternion-translation))
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

(defmethod inverse ((x quaternion-translation))
  (let ((qc (make-quaternion))
        (vc (make-vec3)))
    (aa-tf-qv-conj (quaternion-translation-quaternion x)
                   (quaternion-translation-translation x)
                   qc vc)
    (quaternion-translation-2 qc vc)))


;;; Tagged TFs

(defmethod generic* ((a tf-tag) (b tf-tag))
  (assert (eql (tf-tag-child a)
               (tf-tag-parent b)))
  (tf-tag (tf-tag-parent a)
          (g* (tf-tag-tf a)
              (tf-tag-tf b))
          (tf-tag-child b)))
