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

;;; Libraries ;;;

(define-foreign-library libblas
  (:unix "libblas.so")
  (t (:default "libblas")))

(define-foreign-library liblapack
  (:unix "liblapack.so")
  (t (:default "liblapack")))

(define-foreign-library libamino
  (:unix (:or #.(concatenate 'string (namestring (user-homedir-pathname)) "/lib/libamino.so")
              #.(concatenate 'string (namestring (user-homedir-pathname)) "/local/lib/libamino.so")
              "libamino.so"))
  (t (:default "libamino")))

(define-foreign-library libamino-xerbla
  (:unix (:or #.(concatenate 'string (namestring (user-homedir-pathname)) "/lib/libamino_xerbla_nop.so")
              #.(concatenate 'string (namestring (user-homedir-pathname)) "/local/lib/libamino_xerbla_nop.so")
              "libamino_xerbla_nop.so"))
  (t (:default "libamino_xerbla_nop")))

(defctype blas-size-t :int)

(use-foreign-library libblas)
(use-foreign-library liblapack)
(use-foreign-library libamino)

(use-foreign-library libamino-xerbla)


(defcfun "xerbla_" :void
  (srname :string)
  (position :pointer)
  (namelen :long))


(defun xerbla (name position)
  (with-foreign-object (pposition :int)
    (setf (mem-ref pposition :int) position)
    (xerbla-  name pposition (length name))))


(defconstant +transpose+ (char-code #\T))
(defconstant +no-transpose+ (char-code #\N))

;; Wrapper macros
;; - args: (name type &rest SIZES)
;; - Magic types -- :vector and matrix
;;   - Include the inc-FOO or ld-FOO params in the arglist

(defun check-matrix-bounds (data offset stride rows columns)
  (unless (matrix-counts-in-bounds-p (length data) offset stride rows columns)
    (matrix-storage-error "Matrix is out of bounds")))


(defun check-matrix-dimensions (&rest dimensions)
  ;(declare (dynamic-extent dimensions))
  (unless (apply #'= dimensions)
    (matrix-dimension-error "Mismatched matrix dimensions")))

;;;;;;;;;;;;;;;;;;
;;; BLAS TYPES ;;;
;;;;;;;;;;;;;;;;;;
(def-ref-type blas-size-ref-t blas-size-t)

(define-foreign-type transpose-t ()
  ()
  (:simple-parser transpose-t)
  (:actual-type :pointer))

(defmethod expand-to-foreign-dyn (value var body (type transpose-t))
  `(with-reference (,var (if ,value +transpose+ +no-transpose+) :uint8)
     ,@body))

;;;;;;;;;;;;;;;;;;;;;;;
;;; REFERENCE TYPES ;;;
;;;;;;;;;;;;;;;;;;;;;;;
(defun ref-true-type (type by-reference)
  (if by-reference
      (ecase type
        (:double 'double-ref-t)
        (:float 'float-ref-t)
        (:int 'int-ref-t)
        (size-t 'size-ref-t)
        (blas-size-t 'blas-size-ref-t)
        (transpose-t 'transpose-t))
      type))


(defmacro with-foreign-vector ((pointer increment length) vector intent &body body)
  "Bind POINTER and LENGTH to corresponding values of VECTOR, then evaluate BODY.

INTENT: One of (or :input :output :inout). When value
        is (or :output :inout), return the vector after evaluating
        BODY.
"
  (with-gensyms (ptr-fun array-offset-fun array-fun value data offset stride rows columns)
    (let ((maybe-value (ecase intent
                         (:input nil)
                         ((:output :inout) (list value)))))
      `(labels ((,ptr-fun (,@maybe-value ,pointer ,increment ,length) ,@body ,@maybe-value )
                (,array-offset-fun (,@maybe-value ,data ,offset ,increment ,length)
                  (with-pointer-to-vector-data (,pointer ,data)
                    (,ptr-fun ,@maybe-value
                              (mem-aptr ,pointer :double ,offset)
                              ,increment ,length)))
                (,array-fun (,@maybe-value ,data)
                  (with-pointer-to-vector-data (,pointer ,data)
                    (,ptr-fun ,@maybe-value ,pointer 1 (length ,data)))))
         (let ((,value ,vector))
           (etypecase ,value
             (matrix
              (let ((,data (%matrix-data ,value))
                    (,offset (%matrix-offset ,value))
                    (,stride (%matrix-stride ,value))
                    (,rows (%matrix-rows ,value))
                    (,columns (%matrix-cols ,value)))
                (check-matrix-bounds ,data ,offset ,stride ,rows ,columns)
                (cond ((= 1 ,columns)
                       (,array-offset-fun ,@maybe-value ,data ,offset 1 ,rows))
                      ((= 1 ,rows)
                       (,array-offset-fun ,@maybe-value ,data ,offset ,stride ,columns))
                      (t (matrix-storage-error "Matrix ~A is not a vector" ',vector)))))
             ((simple-array double-float (*))
              (,array-fun ,@maybe-value ,value))
             (real-array
              (,array-fun ,@maybe-value (real-array-data ,value)))))))))

(defmacro with-foreign-simple-vector ((pointer length) vector intent &body body)
  "Bind POINTER and LENGTH to corresponding values of VECTOR, then evaluate BODY.

VECTOR must have an increment of 1.

INTENT: One of (or :input :output :inout). When value
        is (or :output :inout), return the vector after evaluating
        BODY.
"
  (with-gensyms (increment)
    `(with-foreign-vector (,pointer ,increment ,length) ,vector ,intent
       (unless (= 1 ,increment) (matrix-storage-error "Not a simple vector"))
       ,@body)))

(defmacro with-foreign-matrix ((pointer stride rows columns) matrix intent
                               &body body)
"Evaluate BODY with matrix pointer and counts bound to POINTER, STRIDE, ROWS, and COLUNMS.

INTENT: One of (or :input :output :inout). When value
        is (or :output :inout), return the vector after evaluating
        BODY.
"
  (with-gensyms (ptr-fun vector-fun value data offset)
    (let ((maybe-value (ecase intent
                         (:input nil)
                         ((:output :inout) (list value)))))
    `(labels ((,ptr-fun (,@maybe-value ,pointer ,stride ,rows ,columns) ,@body ,@maybe-value)
              (,vector-fun (,@maybe-value ,data)
                (with-pointer-to-vector-data (,pointer ,data)
                  (,ptr-fun ,@maybe-value ,pointer (length ,data) (length ,data) 1))))
     (let ((,value ,matrix))
       (etypecase ,value
         (matrix
          (let ((,data (%matrix-data ,value))
                (,offset (%matrix-offset ,value))
                (,stride (%matrix-stride ,value))
                (,rows (%matrix-rows ,value))
                (,columns (%matrix-cols ,value)))
            (check-matrix-bounds ,data ,offset ,stride ,rows ,columns)
            (with-pointer-to-vector-data (,pointer ,data)
              (,ptr-fun ,@maybe-value (mem-aptr ,pointer :double ,offset)
                        ,stride ,rows ,columns))))
         ((simple-array double-float (*))
          (,vector-fun ,@maybe-value ,value))
         (real-array
          (,vector-fun ,@maybe-value (real-array-data ,value)))))))))

;;; Foreign Binding ;;;

(defun prefix-gensym (prefix var)
  (gensym (concatenate 'string prefix "-" (string var) "-")))

(defun extra-arg-symbol (var type)
  (case type
    (:vector
     (prefix-gensym "INC" var))
    (:matrix
     (prefix-gensym "LD" var))))

;; Bind the C function
(defmacro def-la-cfun ((c-name lisp-name &key
                               by-reference
                               (size-type 'size-t)
                               (pointer-type 'foreign-array-t))
                        result-type &rest args)
  `(defcfun (,c-name ,lisp-name) ,result-type
     ,@(loop for (name type &rest checks) in args
          append (case type
                   ((:vector :matrix)
                    `((,name ,pointer-type)
                      (,(extra-arg-symbol name type) ,(ref-true-type size-type by-reference))))
                   (otherwise (list (list name (ref-true-type type by-reference))))))))

(defmacro def-blas-cfun ((name lisp-name) result-type &rest args)
  `(def-la-cfun (,name ,lisp-name
                       :by-reference t :size-type blas-size-t :pointer-type :pointer)
       ,result-type
     ,@args))
