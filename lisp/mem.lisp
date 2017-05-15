;;;; -*- mode: lisp -*-
;;;;
;;;; Copyright (c) 2014, Georgia Tech Research Corporation
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

(amino-ffi::def-foreign-container mem-region mem-region-t)

(defcfun aa-mem-region-local-get mem-region-t)

(defcfun aa-mem-region-destroy :void
  (ptr :pointer))

(defcfun aa-mem-region-create mem-region-t
  (size size-t))

(defcfun aa-mem-region-alloc :pointer
  (reg mem-region-t)
  (size size-t))

(defcfun aa-mem-region-local-release :void
  (reg mem-region-t))

(defcfun aa-mem-region-pop :void
  (reg mem-region-t)
  (ptr :pointer))

(defcfun aa-mem-region-local-pop :void
  (ptr :pointer))


(defun mem-region-finalize (region)
  (let ((ptr (mem-region-pointer region)))
    (sb-ext:finalize region (lambda () (aa-mem-region-destroy ptr))))
  region)

(defmacro with-mem-region ((var &optional (size 4096)) &body body)
  (with-gensyms (fun)
    `(flet ((,fun (,var) ,@body))
       (let ((,var (aa-mem-region-create ,size)))
         (unwind-protect (,fun ,var)
           (aa-mem-region-destroy ,var))))))
