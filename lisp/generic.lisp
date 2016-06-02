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


(defgeneric normalize (x))
(defmethod normalize ((x real))
  1)


(defgeneric inverse (x))
(defmethod inverse ((x real))
  (/ 1 x))


(defgeneric copy (obj))
(defmethod copy ((x real))
  x)

(defmethod copy ((x cons))
  (copy-list x))

(defmethod copy ((x simple-array))
  (etypecase x
    ((simple-array double-float (*))
     (replace (make-array (length x) :element-type 'double-float)
              x))
    (t (replace (make-array (length x) :element-type (array-element-type x))
                x))))

(defgeneric vec-array (obj &optional array start))

(defmethod vec-array ((obj array) &optional (array (make-vec (length obj))) (start 0))
  (check-type obj (array double-float (*)))
  (replace array obj :start1 start))


(defmethod vec-array ((obj real-array) &optional (array (make-vec (length (real-array-data obj)))) (start 0))
  (replace array (real-array-data obj) :start1 start))



(defmethod normalize ((x array))
  (etypecase x
    ((simple-array double-float (*))
     (vec-normalize x))))

(defmethod normalize ((x list))
  (vec-normalize x
                 (make-list (length x))))


(defmacro def-generic-binop (name binop op &key element-wise)
  `(progn
     ;; generic
     (defgeneric ,binop (a b))
     ;; dispatch
     (defun ,name (first &rest rest)
       (declare (dynamic-extent rest))
       (reduce #',binop rest :initial-value first))
     ;; number method
     (defmethod ,binop ((a number) (b number))
       (,op a b))
     ;; cons methods
     (defmethod ,binop ((a number) (b cons))
       (loop for x in b collect (,op a x)))
     (defmethod ,binop ((a cons) (b number))
       (loop for x in a collect (,op x b)))
     ;; array methods
     (defmethod ,binop ((a number) (b simple-array))
       (let ((n (length b)))
         (etypecase b
           ((simple-array double-float (*))
            (let ((c (make-array n :element-type 'double-float)))
              (dotimes (i n)
                (setf (aref c i)
                      (,op a (aref b i))))
              c)))))
     (defmethod ,binop ((a simple-array) (b number))
       (let ((n (length a)))
         (etypecase a
           ((simple-array double-float (*))
            (let ((c (make-array n :element-type 'double-float)))
              (dotimes (i n)
                (setf (aref c i)
                      (,op (aref a i)
                           b)))
              c)))))
     ;; element-wise methods
     ,@(when element-wise
             `((defmethod ,binop ((a cons) (b cons))
                 (loop
                    for x in a
                    for y in b
                    collect (,op x y)))
               (defmethod ,binop ((a simple-array) (b simple-array))
                 (let ((la (length a))
                       (lb (length b)))
                   (check-matrix-dimensions la lb)
                   (etypecase a
                     ((simple-array double-float (*))
                      (let ((c (make-array la :element-type 'double-float)))
                        (dotimes (i la)
                          (setf (aref c i)
                                (,op (aref a i) (aref b i))))
                        c)))))
               )
             )))

(def-generic-binop g* generic* *)
(def-generic-binop g/ generic/ /)
(def-generic-binop g+ generic+ + :element-wise t)
(def-generic-binop g- generic- - :element-wise t)

;; scalar-vector

(defgeneric matrix->list (matrix))

(defmethod matrix->list ((matrix array))
  (loop for x across matrix
     collect x))

(defmethod matrix->list ((matrix real-array))
  (matrix->list (real-array-data matrix)))

;;;;;;;;;;;;;;;;;;;;;
;;; MATRIX-VECTOR ;;;
;;;;;;;;;;;;;;;;;;;;;

;; Misc
(defmethod dot-product (a b))

(defmethod dot-product ((a array) (b array))
  (assert (= (length a) (length b)))
  (let ((c 0))
    (dotimes (i (length a))
      (setq c (+ c (* (aref a i)
                      (aref b i)))))
    c))

(defmethod generic+ ((a vec3) (b vec3))
  (make-vec3 :data
             (g+ (vec3-data a)
                 (vec3-data b))))
