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

(in-package :amino-ffi)


;;;;;;;;;;;;;
;;; LIBC  ;;;
;;;;;;;;;;;;;

(defcfun ("atof" libc-atof) :double
  (nptr :string))


(defcfun ("malloc" libc-malloc) :pointer
  (size size-t))

(defcfun ("free" libc-free) :void
  (ptr :pointer))

(defcfun ("realloc" libc-realloc) :pointer
  (ptr :pointer)
  (size size-t))

(defcfun ("memcpy" libc-memcpy) :pointer
  (destination :pointer)
  (source :pointer)
  (size size-t))



(in-package :amino)

(defcfun aa-mem-ftoa size-t
  (buf :pointer)
  (n size-t)
  (f :double))

;;; atof() is locale-dependent

;; (defun parse-float (string)
;;   (amino-ffi::libc-atof string))

(defun parse-float (string)
  "Parse a floating point number"
  (coerce (read-from-string string)
          'double-float))


(defun float-to-string (float)
  (with-foreign-object (buf :char 256)
    (let* ((r (aa-mem-ftoa buf 256 float))
           (s (make-string r)))
      (dotimes (i r)
        (setf (aref s i)
              (code-char (mem-aref buf :char i))))
      s)))


;; (defcfun "aa_la_d_angle" :double
;;   (x :pointer)
;;   (incx size-t)
;;   (y :pointer)
;;   (yinc size-t))
