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

(defgeneric vec-array (obj &optional array start))

(defmethod vec-array ((obj array) &optional (array (make-vec (length obj))) (start 0))
  (check-type obj (array double-float (*)))
  (replace array obj :start1 start))


(defmethod vec-array ((obj real-array) &optional (array (make-vec (length (real-array-data obj)))) (start 0))
  (replace array (real-array-data obj) :start1 start))

(defmacro def-generic-binop (name binop)
  `(progn
     (defgeneric ,binop (a b))
     (defun ,name (first &rest rest)
       (reduce #',binop rest :initial-value first))))

(def-generic-binop g* generic*)
(def-generic-binop g/ generic/)
(def-generic-binop g+ generic+)
(def-generic-binop g- generic-)

;; Scalar
(defmethod generic* ((a number) (b number))
  (* a b))

(defmethod generic- ((a number) (b number))
  (- a b))

(defmethod generic+ ((a number) (b number))
  (+ a b))

(defmethod generic/ ((a number) (b number))
  (/ a b))


;; scalar-vector
(defmethod generic* ((a number) (b cons))
  (loop for x in b
     collect (* a x)))

(defmethod generic* ((a cons) (b number))
  (g* b a))

(defun dscal-copy (alpha x)
  (dscal (coerce alpha 'double-float)
         (vec-copy x)))

(defmethod generic* ((a number) (b simple-array))
  (etypecase b
    ((simple-array double-float (*))
       (dscal-copy a b))))

(defmethod generic* ((a simple-array) (b number))
  (g* b a))

;; Vector-Vector
(defmethod generic- ((a simple-array) (b simple-array))
  (etypecase a
    ((simple-array double-float (*))
     (etypecase b
       ((simple-array double-float (*))
        (%simple-array-double-float-op- a b (make-array (length a) :element-type 'double-float)))))))

(defmethod generic+ ((a simple-array) (b simple-array))
  (etypecase a
    ((simple-array double-float (*))
     (etypecase b
       ((simple-array double-float (*))
        (%simple-array-double-float-op+ a b (make-array (length a) :element-type 'double-float)))))))

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
