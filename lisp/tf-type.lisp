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

;;; Geometric types ;;;;



(defun expand-type (value var body type)
  (with-gensyms (x ptr0)
    `(with-matrix (,x ,value)
       (check-type ,x ,type)
       (with-pointer-to-vector-data (,ptr0 (matrix-data ,x))
         (let ((,var (inc-pointer ,ptr0 (* 8 (matrix-offset ,x)))))
           ,@body)))))

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


;;; Specialized Real Arrays
(defmacro def-specialized-array (thing length cffi-type)
  (let ((make-thing (intern (concatenate 'string "MAKE-" (string thing))))
        (deep-copy-thing (intern (concatenate 'string "DEEP-COPY-" (string thing))))
        (thing-data (intern (concatenate 'string (string thing) "-DATA"))))
    `(progn
       (defstruct (,thing (:include real-array
                                    (data (make-vec ,length)
                                          :type  (simple-array double-float (,length))))))
       (defun ,deep-copy-thing (thing &optional (result (,make-thing)))
         (replace (,thing-data result)
                  (,thing-data thing))
         result)
       (defmethod generic* ((a real) (b ,thing))
         (let ((c (,deep-copy-thing b)))
           (dscal (coerce a 'double-float)
                  (,thing-data c))
           c))
       (defmethod generic* ((a ,thing) (b real))
         (generic* b a))
       (defmethod generic/ ((a ,thing) (b real))
         (let ((c (,deep-copy-thing a)))
           (dscal (/ 1d0 (coerce b 'double-float))
                  (,thing-data c))
           c))
       (define-foreign-type ,cffi-type ()
         ()
         (:simple-parser ,cffi-type)
         (:actual-type :pointer))
       (defmethod expand-to-foreign-dyn (value var body (type ,cffi-type))
         (list* 'with-foreign-fixed-vector var value ,length :input
                body)))))



;;; Point 2
(def-specialized-array vec2 2 vector-2-t)

;;; Point 3

;; or should this just be an array deftype?
(def-specialized-array vec3 3 vector-3-t)

(defun vec3* (x y z)
  (make-vec3 :data (vec x y z)))

(defmacro with-vec3 ((x y z) vec3 &body body)
  (with-gensyms (value)
    `(let ((,value (vec3 ,vec3)))
       (let ((,x (vecref ,value +x+))
             (,y (vecref ,value +y+))
             (,z (vecref ,value +z+)))
         ,@body))))

(defun vec3-normalize (v)
  (vec-normalize v (make-vec3)))

(defun vec3-identity-p (vec3)
  (with-vec3 (x y z) vec3
    (= x y z 0d0)))

;;; Axis-Angle
(def-specialized-array axis-angle 4 axis-angle-t)

(defun axis-angle* (x y z theta)
  (let ((n (sqrt (+ (* x x) (* y y) (* z z)))))
    (make-axis-angle :data (vec (/ x n) (/ y n) (/ z n) theta))))

;;; Quaternion
(def-specialized-array quaternion 4 quaternion-t)

(defun quaternion-x (q)
  (aref (quaternion-data q) +x+))
(defun quaternion-y (q)
  (aref (quaternion-data q) +y+))
(defun quaternion-z (q)
  (aref (quaternion-data q) +z+))
(defun quaternion-w (q)
  (aref (quaternion-data q) +w+))

(defun quaternion* (x y z w)
  (make-quaternion :data (vec x y z w)))

(defmacro with-quaternion ((x y z w) quaternion &body body)
  (with-gensyms (value)
    `(let ((,value (quaternion ,quaternion)))
       (let ((,x (vecref ,value +x+))
             (,y (vecref ,value +y+))
             (,z (vecref ,value +z+))
             (,w (vecref ,value +w+)))
         ,@body))))

(defun quaternion-identity-p (quaternion)
  (with-quaternion (x y z w) quaternion
    (and (= x y z 0d0)
         (= w 1d0))))

;; Rotation Vector
(def-specialized-array rotation-vector 3 rotation-vector-t)

(defun rotation-vector (x y z)
  (make-rotation-vector :data (vec x y z)))

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

(defun degrees (value)
  (* value (/ pi 180d0)))

(defun pi-rad (value)
  (* value pi))

(defstruct (euler-angle (:include real-array
                                  (data (make-vec 3) :type  (simple-array double-float (3))))))

(defstruct (euler-zyx (:include euler-angle)))

(defun euler-zyx* (z y x)
  (make-euler-zyx :data (vec z y x)))

(define-foreign-type euler-zyx-t ()
  ()
  (:simple-parser euler-zyx-t)
  (:actual-type :pointer))
(defmethod expand-to-foreign-dyn (value var body (type euler-zyx-t))
  `(with-foreign-fixed-vector ,var ,value 3 :input ,@body))


(defun euler-rpy* (r p y)
  "Alias for ZYX euler angles."
  (euler-zyx* y p r))

;;; Dual Quaternion
(defstruct (dual-quaternion (:include real-array
                                 (data (make-vec 8) :type  (simple-array double-float (8))))))

(define-foreign-type dual-quaternion-t ()
  ()
  (:simple-parser dual-quaternion-t)
  (:actual-type :pointer))
(defmethod expand-to-foreign-dyn (value var body (type dual-quaternion-t))
  `(with-foreign-fixed-vector ,var ,value 8 :input ,@body))

;;; Implicit dual quaternion
(defstruct quaternion-translation
  (quaternion (make-quaternion) :type  quaternion)
  (translation (make-vec3) :type  vec3))


(defstruct (tf-tag (:constructor tf-tag (parent tf child)))
  parent
  tf
  child)


;;;;;;;;;;;;;;;
;;; TF Tree ;;;
;;;;;;;;;;;;;;;

;; TODO: Store child sets
;;       Prevent orphaned frames


(defun tf-tree-frame-compare (a b)
  (if (null a)
      (if (null b)
          0
          -1)
      (if (null b)
          1
          (sycamore-util:string-compare a b))))

(defstruct (tf-tree (:constructor %make-tf-tree))
  (relative-map (sycamore:make-tree-map #'tf-tree-frame-compare)
                :type sycamore::tree-map)
  (frame-set( sycamore:make-tree-set #'tf-tree-frame-compare)
            :type sycamore::tree-set))

(defun make-tf-tree ()
  "Create a new TF tree.
The tree is a map from the frame name to its relative transform."
  (%make-tf-tree :relative-map (sycamore:make-tree-map #'tf-tree-frame-compare)
                 :frame-set (sycamore:make-tree-set #'sycamore-util:string-compare)))

(defun tf-tree-split-insert (tree parent tf child)
  "Add tagged TF to tree."
  (let ((frame-set (tf-tree-frame-set tree)))
    ;; Intern parent and child
    (when parent
      (multiple-value-setq (frame-set parent)
        (sycamore:tree-set-intern frame-set parent)))
    (when child
      (multiple-value-setq (frame-set child)
        (sycamore:tree-set-intern frame-set child)))
    ;; Insert
    (%make-tf-tree :relative-map (sycamore:tree-map-insert (tf-tree-relative-map tree)
                                                           child (tf-tag parent tf child))
                   :frame-set frame-set)))

(defun tf-tree-insert (tree tf-tag)
  "Add tagged TF to tree."
  (tf-tree-split-insert tree
                        (tf-tag-parent tf-tag)
                        (tf-tag-tf tf-tag)
                        (tf-tag-child tf-tag)))

(defun tf-tree-remove (tree frame)
  "Remove FRAME from tree"
  (%make-tf-tree :relative-map (sycamore:tree-map-remove (tf-tree-relative-map tree) frame)
                 :frame-set (sycamore:tree-set-remove (tf-tree-frame-set tree) frame)))

(defun tf-tree-find (tree frame)
  "Find the TF in the tree"
  (let ((tf (sycamore:tree-map-find (tf-tree-relative-map tree) frame)))
    (when tf (assert (equal frame (tf-tag-child tf))))
    tf))

(defun tf-tree-absolute-map (relative-tree &optional frame)
  "Compute the absolute transforms."
  (labels ((rec-insert (absolute-tree frame)
             (let* ((rel-tf (tf-tree-find relative-tree frame))
                    (parent-frame (tf-tag-parent rel-tf)))
               (if (null parent-frame)
                   (tf-tree-insert absolute-tree rel-tf)
                   (let* ((absolute-tree (rec absolute-tree parent-frame))
                          (parent-tf (tf-tree-find absolute-tree parent-frame)))
                     (tf-tree-insert absolute-tree (g* parent-tf rel-tf))))))
           (rec (absolute-tree frame)
                (if (null frame)
                    absolute-tree
                    (let ((abs-tf (tf-tree-find absolute-tree frame)))
                      (if abs-tf
                          absolute-tree
                          (rec-insert absolute-tree frame))))))
    (let ((empty-tree (make-tf-tree)))
      (if frame
          (rec empty-tree frame)
          (sycamore:fold-tree-map (lambda (absolute-tree frame rel-tf)
                                    (assert (equal frame (tf-tag-child rel-tf)))
                                    (rec absolute-tree frame))
                                  (make-tf-tree)
                                  (tf-tree-relative-map relative-tree))))))

(defun tf-tree-absolute-tf (relative-tree frame)
  "Compute the absolute transform of FRAME."
  (let ((absolute-tree (tf-tree-absolute-map relative-tree frame)))
    (tf-tree-find absolute-tree frame)))
