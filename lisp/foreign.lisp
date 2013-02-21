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

(define-foreign-library libamino
  (:unix "libamino.so")
  (t (:default "libamino")))

(define-foreign-library libamino-xerbla
  (:unix "libamino_xerbla_nop.so")
  (t (:default "libamino_xerbla_nop")))

(define-foreign-library libblas
  (:unix "libblas.so")
  (t (:default "libblas")))

(defctype blas-size-t :int)

(use-foreign-library libamino-xerbla)
(use-foreign-library libamino)


(defcfun "xerbla_" :void
  (srname :string)
  (position :pointer)
  (namelen :long))


(defun xerbla (name position)
  (with-foreign-object (pposition :int)
    (setf (mem-ref pposition :int) position)
    (xerbla-  name pposition (length name))))

;; Wrapper macros
;; - args: (name type &rest SIZES)
;; - Magic types -- :vector and matrix
;;   - Include the inc-FOO or ld-FOO params in the arglist
;; - SIZES: (or (:length var) (:rows var) (:cols var))
;;   - automatically get these size vars and check validity

(defun prefix-gensym (prefix var)
  (gensym (concatenate 'string prefix "-" (string var) "-")))

(defun extra-arg-symbol (var type)
  (case type
    (:vector
     (prefix-gensym "INC" var))
    (:matrix
     (prefix-gensym "LD" var))))


(defmacro def-la-cfun ((c-name lisp-name &key by-reference) result-type &rest args)
  `(defcfun (,c-name ,lisp-name) ,result-type
     ,@(loop for (name type &rest checks) in args
          append (case type
                   ((:vector :matrix)
                    `((,name :pointer)
                      (,(extra-arg-symbol name type) ,(if by-reference
                                                         :pointer 'size-t))))
                   (otherwise (list (list name (if by-reference
                                                         :pointer type))))))))

(defmacro with-pointer-to ((var value type) &body body)
  (with-gensyms (ptr)
    `(let ((,ptr ,value))
       (cffi:with-foreign-object (,var ',type)
         (setf (cffi:mem-ref ,var ',type) ,ptr)
         ,@body))))

(defun make-var-sym (name thing)
  (cons (cons name thing) (prefix-gensym (string thing) name)))

(defun var-sym (alist name thing)
  (cdr (assoc (cons name thing) alist :test #'equal)))

(defun count-source-var (alist type params)
  (case type
    ((blas-size-t size-t)
     (when-let (source (find-if (lambda (x) (find (car x) '(:length :rows :cols))) params))
       (var-sym alist (second source) (first source))))))

(defun bind-counts (var-alist var type body)
  (let ((data (var-sym var-alist var :data))
        (offset (var-sym var-alist var :offset))
        (stride (var-sym var-alist var :stride))
        (rows (var-sym var-alist var :rows))
        (cols (var-sym var-alist var :cols))
        (inc (var-sym var-alist var :inc))
        (length (var-sym var-alist var :length)))
    (case type
      (:matrix
       `((let ((,rows (matrix-rows ,var))
               (,cols (matrix-cols ,var))
               (,stride (matrix-stride ,var))
               (,offset (matrix-offset ,var))
               (,data (matrix-data ,var)))
           (declare (type fixnum ,offset ,stride ,rows ,cols))
           ;; check overflow
           (unless (matrix-counts-in-bounds-p
                    (length ,data) ,offset ,stride ,rows ,cols)
             (matrix-storage-error "Argument ~A is out of bounds" ',var))
           ,@body)))
      (:vector
       (bind-counts var-alist var :matrix
                    `((let ((,inc (matrix-counts-vector-increment ,rows ,cols ,stride))
                            (,length (* ,rows ,cols)))
                        (declare (type fixnum ,inc ,length))
                        ,@body))))
      (otherwise body))))


(defun append-body (bindings body)
  (list (append bindings body)))

(defun append-list-body (bindings body)
  (append-body (car bindings) body))

(defun bind-refs (var-alist var type params by-reference body)
  (let ((thing (case type
                 (:matrix (var-sym var-alist var :stride))
                 (:vector (var-sym var-alist var :inc))))
        (data (var-sym var-alist var :data)))
    (cond
      ((find type '(:matrix :vector))
       (append-body
        `(cffi-sys:with-pointer-to-vector-data (,var ,data))
        (if by-reference
            (append-body `(with-pointer-to (,thing ,thing :int))
                         body)
            body)))
      (by-reference
       (let ((var (or (count-source-var var-alist type params)
                      var)))
         `((with-pointer-to (,var ,var ,type)
             ,@body))))
      (t body))))

(defmacro def-la-wrapper ((c-name lisp-name &key by-reference) result-type &rest args)
  (declare (ignore result-type)) ;; todo: optimize with declarations
  (let ((normal-arglist (loop for (name type . params) in args
                           when (or (null params)
                                    (equal '(:inout) params))
                           collect name))
        (inout-args (loop for (name type . params) in args
                         when (find :inout params)
                         collect name))
        (matrix-args (loop for (name type) in args
                        when (or (eq type :vector) (eq type :matrix))
                        collect name))
        (var-alist (loop for (name type) in args
                      append (append
                              (when (eq type :vector)
                                (list (cons (cons name :inc) (prefix-gensym "INC" name))
                                      (cons (cons name :length) (prefix-gensym "LENGTH" name))))
                              (when (or (eq type :matrix) (eq type :vector))
                                (list (cons (cons name :rows) (prefix-gensym "ROWS" name))
                                      (cons (cons name :offset) (prefix-gensym "OFFSET" name))
                                      (cons (cons name :cols) (prefix-gensym "COLS" name))
                                      (cons (cons name :data) (prefix-gensym "DATA" name))
                                      (cons (cons name :stride) (prefix-gensym "STRIDE" name))))))))
    (labels ((var-symbol (name type)
               (cdr (assoc (cons name type) var-alist :test #'equal)))
             (true-arg (name type params)
               (cond
                 ((eq type :vector)
                  (list name (var-symbol name :inc)))
                 ((eq type :matrix)
                  (list name (var-symbol name :stride)))
                 (params
                  (dolist (p params)
                    (when (and (consp p) (find (first p) '(:rows :length :cols)))
                      (return-from true-arg (list (var-symbol (second p) (first p))))))
                  (list name))
                 (t (list name)))))
      `(defun ,lisp-name ,normal-arglist
         ;; declare types
         ,@(when matrix-args `((declare (type matrix ,@matrix-args))))
         ;; bind counts
         ,@(reduce
            (lambda (body arg)
              (destructuring-bind (name type &rest params) arg
                (declare (ignore params))
                (bind-counts var-alist name type body)))
            args
            :initial-value
            ;; check sizes
            `(,@(loop
                   for (name type . params) in args
                   for length-checks = (loop for param in params
                                          when (and (consp param) (find (first param) '(:length :rows :cols)))
                                          collect (var-symbol (second param) (first param)))
                   when (cdr length-checks)
                   collect `(assert (= ,@length-checks)))
                ,@(reduce
                   (lambda (body arg)
                     (destructuring-bind (name type &rest params) arg
                       (bind-refs var-alist name type params by-reference body)))
                   args
                   :initial-value
                   `(x (,c-name ,@(loop for (name type . params) in args
                                     append (true-arg name type params)))))))
         ;;output
         ,@(cond
            ((null inout-args) nil)
            ((= 1 (length inout-args)) inout-args)
            (t (list (cons 'values inout-args))))))))

(defmacro def-la ((c-name lisp-name &key by-reference) result-type &rest args)
  (let ((c-lisp-name (gensym (string lisp-name))))
    `(progn (def-la-cfun (,c-name ,c-lisp-name :by-reference ,by-reference) ,result-type
              ,@args)
            (def-la-wrapper (,c-lisp-name ,lisp-name :by-reference ,by-reference) ,result-type
              ,@args))))
