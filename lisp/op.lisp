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




;; (defmacro def-simple-array-binop (name op type)
;;   (with-gensyms (a b c n-a n-b n-c i)
;;   `(defun ,(intern (concatenate 'string "%simple-array-" type "-op" (string op)))
;;        (,a ,b ,c)
;;      (declare (type (simple-array ,type (*)) ,a ,b ,c))
;;      (let ((,n-a (length ,a))
;;            (,n-b (length ,b))
;;            (,n-c (length ,c)))
;;        (assert (= ,n-a ,n-b ,n-c))
;;        (dotimes (,i ,n-a)
;;          (setf (aref ,c i) (- (aref ,a ,i)
;;                               (aref ,b ,i)))

(in-package :amino)

(defun %simple-array-double-float-op- (a b c)
  (declare (type (simple-array double-float (*)) a b c))
  (let ((n-a (length a))
        (n-b (length b))
        (n-c (length c)))
    (assert (= n-a n-b n-c))
    (replace c a)
    (daxpy -1d0 b c))
  c)

(defun %simple-array-double-float-op+ (a b c)
  (declare (type (simple-array double-float (*)) a b c))
  (let ((n-a (length a))
        (n-b (length b))
        (n-c (length c)))
    (assert (= n-a n-b n-c))
    (replace c a)
    (daxpy 1d0 b c))
  c)

;; (defun vec-ssd (a b)
;;   (let ((c 0))
;;     (dotimes (i (min (length a) (length b)))
;;       (let ((x (- (aref a i))

(defun row-matrix (&rest rows)
  "Create a matrix from the given rows."
  (declare (dynamic-extent rows))
  (let ((m (length rows))
        (n (vec-length (car rows))))
    (let ((matrix (make-matrix m n)))
      (with-foreign-dmat (dmat matrix :inout)
        (cffi:with-foreign-object (rowvec '(:struct aa-dvec))
          (loop
             for row in rows
             for i from 0
             do
               (%aa-dmat-row-vec dmat i rowvec)
               (%aa-dvec-copy-to-foreign row rowvec))))
        matrix)))

(defun col-matrix (&rest cols)
  "Create a matrix from the given columns."
  (declare (dynamic-extent cols))
  (let ((n (length cols))
        (m (vec-length (car cols))))
    (let ((matrix (make-matrix m n)))
      (with-foreign-dmat (dmat matrix :inout)
        (cffi:with-foreign-object (colvec '(:struct aa-dvec))
          (loop
             for col in cols
             for j from 0
             do
               (%aa-dmat-col-vec dmat j colvec)
               (%aa-dvec-copy-to-foreign col colvec))))
        matrix)))

(defun row-vector (&rest args)
  (declare (dynamic-extent args))
  (row-matrix args))

(defun col-vector (&rest args)
  (declare (dynamic-extent args))
  (col-matrix args))


(defun matrix-copy (matrix &optional
                    (copy (make-matrix (matrix-rows matrix)
                                       (matrix-cols matrix))))
  "Create a copy of MATRIX."
  (aa-dmat-copy matrix copy)
  copy)
