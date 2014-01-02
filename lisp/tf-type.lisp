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
  (with-gensyms (x ptr0)
    `(with-matrix (,x ,value)
       (check-type ,x ,type)
       (with-pointer-to-vector-data (,ptr0 (matrix-data ,x))
         (let ((,var (inc-pointer ,ptr0 (* 8 (matrix-offset ,x)))))
           ,@body)))))

(defun expand-vector (value var body length)
  "Get the data pointer for value, checking storage and size"
  (with-gensyms (x ptr0 body-fun)
    `(flet ((,body-fun (,var) ,@body))
       (let ((,x ,value))
         (etypecase ,x
           (matrix
            (if (matrix-vector-n-p ,x ,length 1)
                ;; valid type
                (with-pointer-to-vector-data (,ptr0 (matrix-data ,x))
                  (,body-fun (inc-pointer ,ptr0 (* 8 (matrix-offset ,x)))))
                ;; invalid type, throw error
                (matrix-storage-error "Invalid matrix size or storage of ~D: ~A" ,length ,x)))
           (real-array
            (if (= ,length (length (real-array-data ,x)))
                (with-pointer-to-vector-data (,ptr0 (real-array-data ,x))
                  (,body-fun ,ptr0))
                (matrix-storage-error "Invalid matrix size of ~D: ~A" ,length ,x)))
           ((simple-array double-float (*))
            (if (= (length ,x) ,length)
                ;; valid type
                (with-pointer-to-vector-data (,ptr0 ,x)
                  (,body-fun ,ptr0))
                ;; invalid type
                (matrix-storage-error "Invalid matrix size of ~D: ~A" ,length ,x))))))))

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

(defun make-transformation-matrix ()
  (make-matrix 3 4))

;;; Rotation Matrix

;; TODO: check determinant and norms
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

(defun make-rotation-matrix ()
  (make-matrix 3 3))

;;; Point 3

;; or should this just be an array deftype?
(defstruct (vec3 (:include real-array
                           (data (make-vec 3) :type  (simple-array double-float (3))))))
(defun vec3 (x y z)
  (make-vec3 :data (vec x y z)))


(defstruct (point3 (:include vec3)))

(defun point3 (x y z)
  (make-point3 :data (vec x y z)))

(define-foreign-type vector-3-t ()
  ()
  (:simple-parser vector-3-t)
  (:actual-type :pointer))
(defmethod expand-to-foreign-dyn (value var body (type vector-3-t))
  (expand-vector value var body 3))

;;; Axis-Angle
(defstruct (axis-angle (:include real-array
                                 (data (make-vec 4) :type  (simple-array double-float (4))))))

(defun axis-angle (x y z theta)
  (let ((n (sqrt (+ (* x x) (* y y) (* z z)))))
    (make-axis-angle :data (vec (/ x n) (/ y n) (/ z n) theta))))


(define-foreign-type axis-angle-t ()
  ()
  (:simple-parser axis-angle-t)
  (:actual-type :pointer))
(defmethod expand-to-foreign-dyn (value var body (type axis-angle-t))
  (expand-vector value var body 4))


;;; Quaternion
(defstruct (quaternion (:include real-array
                                 (data (make-vec 4) :type  (simple-array double-float (4))))))
(defun quaternion-x (q)
  (aref (quaternion-data q) +x+))
(defun quaternion-y (q)
  (aref (quaternion-data q) +y+))
(defun quaternion-z (q)
  (aref (quaternion-data q) +z+))
(defun quaternion-w (q)
  (aref (quaternion-data q) +w+))

(define-foreign-type quaternion-t ()
  ()
  (:simple-parser quaternion-t)
  (:actual-type :pointer))
(defmethod expand-to-foreign-dyn (value var body (type quaternion-t))
  (expand-vector value var body 4))

;; Rotation Vector
(defstruct (rotation-vector (:include vec3)))

(defun rotation-vector (x y z)
  (make-rotation-vector :data (vec x y z)))

(define-foreign-type rotation-vector-t ()
  ()
  (:simple-parser rotation-vector-t)
  (:actual-type :pointer))
(defmethod expand-to-foreign-dyn (value var body (type rotation-vector-t))
  (expand-vector value var body 3))


;; Principal Angle
(defstruct principal-angle
  (value 0d0 :type double-float))

(defstruct (x-angle (:include principal-angle)
                    (:constructor %x-angle (value))))
(defstruct (y-angle (:include principal-angle)
                    (:constructor %y-angle (value))))
(defstruct (z-angle (:include principal-angle)
                    (:constructor %z-angle (value))))

(defun x-angle (value)
  (%x-angle (coerce value 'double-float)))
(defun y-angle (value)
  (%y-angle (coerce value 'double-float)))
(defun z-angle (value)
  (%z-angle (coerce value 'double-float)))

(defstruct (euler-angle (:include real-array
                                  (data (make-vec 3) :type  (simple-array double-float (3))))))

(defstruct (euler-zyx (:include euler-angle)))
(defun euler-zyx (z y x)
  (make-euler-zyx :data (vec z y x)))

;;; Dual Quaternion
(defstruct (dual-quaternion (:include real-array
                                 (data (make-vec 8) :type  (simple-array double-float (8))))))
;; (defun dual-quaternion (q v)
;;   ;; TODO
;;   (assert 0))

(define-foreign-type dual-quaternion-t ()
  ()
  (:simple-parser dual-quaternion-t)
  (:actual-type :pointer))
(defmethod expand-to-foreign-dyn (value var body (type dual-quaternion-t))
  (expand-vector value var body 8))

;;; Implicit dual quaternion
(defstruct quaternion-translation
  (quaternion (make-quaternion) :type  quaternion)
  (translation (make-vec3) :type  vec3))
