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



(defgeneric g* (a b))
(defgeneric g- (a b))
(defgeneric g+ (a b))
(defgeneric g/ (a b))

;; Scalar
(defmethod g* ((a number) (b number))
  (* a b))

(defmethod g- ((a number) (b number))
  (- a b))

(defmethod g+ ((a number) (b number))
  (+ a b))

(defmethod g/ ((a number) (b number))
  (/ a b))


;; Scalar-Vector
(defmethod g* ((a number) (b simple-array))
  (ecase (array-element-type b)
    (double-float (dscal (coerce a 'double-float) (vec-copy b)))))

(defmethod g* ((a simple-array) (b number))
  (g* b a))



;; Vector-Vector
(defmethod g- ((a simple-array) (b simple-array))
  (etypecase a
    ((simple-array double-float (*))
     (etypecase b
       ((simple-array double-float (*))
        (%simple-array-double-float-op- a b (make-array (length a) :element-type 'double-float)))))))

(defmethod g+ ((a simple-array) (b simple-array))
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
