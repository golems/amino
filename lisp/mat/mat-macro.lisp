;;;; -*- mode: lisp -*-
;;;;
;;;; Copyright (c) 2018, Colorado School of Mines
;;;; All rights reserved.
;;;;
;;;; Author(s): Neil T. Dantam <ndantam@mines.edu>
;;;;
;;;;   Redistribution and use in source and binary forms, with or
;;;;   without modification, are permitted provided that the following
;;;;   conditions are met:
;;;;
;;;;   * Redistributions of source code must retain the above
;;;;     copyright notice, this list of conditions and the following
;;;;     disclaimer.
;;;;   * Redistributions in binary form must reproduce the above
;;;;     copyright notice, this list of conditions and the following
;;;;     disclaimer in the documentation and/or other materials
;;;;     provided with the distribution.
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


;; descriptor binding
(in-package :amino)

(defmacro with-foreign-dvec ((var value intent) &body body)
  (alexandria:with-gensyms (len ptr inc)
    `(cffi:with-foreign-object (,var '(:struct aa-dvec))
       (amino-ffi:with-foreign-vector (,ptr ,inc ,len) ,value ,intent
         (setf (foreign-slot-value ,var '(:struct aa-dvec) 'data) ,ptr
               (foreign-slot-value ,var '(:struct aa-dvec) 'len) ,len
               (foreign-slot-value ,var '(:struct aa-dvec) 'inc) ,inc)
         ,@body))))

(defmacro with-foreign-dmat ((var value intent) &body body)
  (alexandria:with-gensyms (ptr ld rows cols)
    `(cffi:with-foreign-object (,var '(:struct aa-dmat))
       (amino-ffi:with-foreign-matrix (,ptr ,ld ,rows ,cols)  ,value ,intent
         (setf (foreign-slot-value ,var '(:struct aa-dmat) 'data) ,ptr
               (foreign-slot-value ,var '(:struct aa-dmat) 'rows) ,rows
               (foreign-slot-value ,var '(:struct aa-dmat) 'cols) ,cols
               (foreign-slot-value ,var '(:struct aa-dmat) 'ld)   ,ld)
         ,@body))))


;; CFFI Expanders
(define-foreign-type dvec-input ()
  ()
  (:simple-parser dvec-input)
  (:actual-type :pointer))
(define-foreign-type dmat-input ()
  ()
  (:simple-parser dmat-input)
  (:actual-type :pointer))
(defmethod expand-to-foreign-dyn (value var body (type dvec-input))
  `(with-foreign-dvec (,var ,value :input) ,@body))
(defmethod expand-to-foreign-dyn (value var body (type dmat-input))
  `(with-foreign-dmat (,var ,value :input) ,@body))

(define-foreign-type dvec-output ()
  ()
  (:simple-parser dvec-output)
  (:actual-type :pointer))
(define-foreign-type dmat-output ()
  ()
  (:simple-parser dmat-output)
  (:actual-type :pointer))
(defmethod expand-to-foreign-dyn (value var body (type dvec-output))
  `(with-foreign-dvec (,var ,value :output) ,@body))
(defmethod expand-to-foreign-dyn (value var body (type dmat-output))
  `(with-foreign-dmat (,var ,value :output) ,@body))

(define-foreign-type dvec-inout ()
  ()
  (:simple-parser dvec-inout)
  (:actual-type :pointer))
(define-foreign-type dmat-inout ()
  ()
  (:simple-parser dmat-inout)
  (:actual-type :pointer))
(defmethod expand-to-foreign-dyn (value var body (type dvec-inout))
  `(with-foreign-dvec (,var ,value :inout) ,@body))
(defmethod expand-to-foreign-dyn (value var body (type dmat-inout))
  `(with-foreign-dmat (,var ,value :inout) ,@body))
