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
(defctype cblas-size-t :int)

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


(defmacro def-la-cfun (name result-type &rest args)
  `(defcfun ,name ,result-type
     ,@(loop for (name type &rest checks) in args
          append (case type
                   ((:vector :matrix)
                    `((,name :pointer)
                      (,(extra-arg-symbol name type) size-t)))
                   (otherwise (list (list name type)))))))

(defmacro def-la-wrapper ((c-name lisp-name) result-type &rest args)
  (declare (ignore result-type)) ;; todo: optimize with declarations
  (let ((normal-arglist (loop for (name type . params) in args
                           when (null params)
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
                                      (cons (cons name :data-length) (prefix-gensym "DATA-LENGTH" name))
                                      (cons (cons name :stride) (prefix-gensym "STRIDE" name))))))))
    (labels ((var-symbol (name type)
               (cdr (assoc (cons name type) var-alist :test #'equal)))
             (bind-count (name type function)
               (list (var-symbol name type)
                     (list function name)))
             (expand-bind-counts (args body)
               (if (null args)
                   body
                   (destructuring-bind (name type &rest params) (car args)
                     (declare (ignore params))
                     (let ((data (var-symbol name :data))
                           (data-length (var-symbol name :data-length))
                           (offset (var-symbol name :offset))
                           (stride (var-symbol name :stride))
                           (rows (var-symbol name :rows))
                           (cols (var-symbol name :cols))
                           (inc (var-symbol name :inc))
                           (length (var-symbol name :length)))
                       (case type
                         (:matrix
                          `((let (,(bind-count name :rows 'matrix-rows)
                                  ,(bind-count name :cols 'matrix-cols)
                                   ,(bind-count name :stride 'matrix-stride)
                                   ,(bind-count name :offset 'matrix-offset)
                                   (,data (matrix-data ,name)))
                              (declare (type fixnum ,offset ,stride ,rows ,cols))
                              (let ((,(var-symbol name :data-length) (length ,data)))
                                (declare (type fixnum ,data-length))
                                ;; check overflow
                                (unless (matrix-counts-in-bounds-p
                                         ,data-length ,offset ,stride ,rows ,cols)
                                  (matrix-storage-error "Argument ~A is out of bounds" ',name))
                                (cffi-sys:with-pointer-to-vector-data (,name ,data)
                                  ,@(expand-bind-counts (cdr args) body))))))
                         (:vector
                          (expand-bind-counts
                           `((,name :matrix))
                           `((let ((,inc (matrix-counts-vector-increment ,rows ,cols ,stride))
                                   (,length (* ,rows ,cols)))
                               ,@(expand-bind-counts (cdr args) body)))))
                         (otherwise
                          (expand-bind-counts (cdr args) body)))))))
             (true-arg (name type params)
               (cond
                 ((eq type :vector)
                  (list name (var-symbol name :inc)))
                 ((eq type :matrix)
                  (list name (var-symbol name :stride)))
                 (params
                  (dolist (p params)
                    (when (find (first p) '(:rows :length :cols))
                      (return-from true-arg (list (var-symbol (second p) (first p))))))
                  (list name))
                 (t (list name)))))
      `(defun ,lisp-name ,normal-arglist
         ;; declare types
         ,@(when matrix-args `((declare (type matrix ,@matrix-args))))
         ;; bind counts
         ,@(expand-bind-counts
            args
            `( ;; check sizes
              ,@(loop
                   for (name type . params) in args
                   for length-checks = (loop for param in params
                                          when (find (first param) '(:length :rows :cols))
                                          collect (var-symbol (second param) (first param)))
                   when (cdr length-checks)
                   collect `(assert (= ,@length-checks)))
                ;; make the damn call
                (,c-name ,@(loop for (name type . params) in args
                              append (true-arg name type params)))))))))

(def-la-wrapper (aa-la-d-angle d-angle) :double
  (n size-t (:length x) (:length y))
  (x :vector)
  (y :vector))

(def-la-cfun "cblas_dscal" :void
  (n cblas-size-t)
  (alpha :double)
  (x :vector))

(def-la-wrapper (cblas-dscal dscal) :void
  (n cblas-size-t (:length x))
  (alpha :double)
  (x :vector))

(def-la-cfun "aa_la_d_angle" :double
  (n size-t)
  (x :vector)
  (y :vector))



;; (defcfun "aa_la_d_angle" :double
;;   (x :pointer)
;;   (incx size-t)
;;   (y :pointer)
;;   (yinc size-t))
