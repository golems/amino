;;;; -*- mode: lisp -*-
;;;;
;;;; Copyright (c) 2012, Georgia Tech Research Corporation
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

(in-package :amino-type)

;;;;;;;;;;;;;;;;;;;
;;; Array Types ;;;
;;;;;;;;;;;;;;;;;;;

(deftype vec (length)
  "Type alias for a double floating point vector."
  `(simple-array double-float (,length)))

(defstruct real-array
  (data nil :type  (simple-array double-float (*))))

(defstruct (matrix (:include real-array)
                   (:constructor %make-matrix)
                   (:conc-name %matrix-))
  "Descriptor for a matrix following LAPACK conventions."
  (offset 0 :type (integer 0 #.most-positive-fixnum))
  (stride 0 :type (integer 1 #.most-positive-fixnum))
  (cols 0 :type (integer 1 #.most-positive-fixnum))
  (rows 0 :type (integer 1 #.most-positive-fixnum)))

;;;;;;;;;;;;;;;;;;
;;; Conditions ;;;
;;;;;;;;;;;;;;;;;;


(define-condition matrix-storage (error)
  ((message
    :initarg :message)))

(defmethod print-object ((object matrix-storage) stream)
  (print-unreadable-object (object stream :type t :identity t)
    (format stream ": ~A"
            (slot-value object 'message))))

(defun matrix-storage-error (format &rest args)
  (error 'matrix-storage
         :message (apply #'format nil format args)))

(define-condition matrix-dimension (error)
  ((message
    :initarg :message)))

(defmethod print-object ((object matrix-dimension) stream)
  (print-unreadable-object (object stream :type t :identity t)
    (format stream ": ~A"
            (slot-value object 'message))))

(defun matrix-dimension-error (format &rest args)
  (error 'matrix-storage
         :message (apply #'format nil format args)))

;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Checking Functions ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun matrix-counts-in-bounds-p (data-length offset stride rows cols)
  "Check if given counts are within array bounds."
  (declare (type fixnum data-length offset stride rows cols))
  (and (>= stride rows)
       (<= (+ offset
              (* stride cols))
           data-length)))

(defun check-matrix-bounds (data offset stride rows columns)
  (unless (matrix-counts-in-bounds-p (length data) offset stride rows columns)
    (matrix-storage-error "Matrix is out of bounds")))


(defun check-matrix-dimensions (&rest dimensions)
  ;(declare (dynamic-extent dimensions))
  (unless (apply #'= dimensions)
    (matrix-dimension-error "Mismatched matrix dimensions")))
