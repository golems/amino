;;;; -*- mode: lisp -*-
;;;;
;;;; Copyright (c) 2017, Rice University
;;;; All rights reserved.
;;;;
;;;; Author(s): Neil T. Dantam <ntd@gatech.edu>
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
;;;;   * Neither the name of copyright holder the names of its
;;;;     contributors may be used to endorse or promote products
;;;;     derived from this software without specific prior written
;;;;     permission.
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


;;;;;;;;;;;;;;;;
;; Point List ;;
;;;;;;;;;;;;;;;;

(amino-ffi::def-foreign-container ct-pt-list ct-pt-list-t
  :slots ((region nil :type (or null mem-region))))

(defcfun aa-ct-pt-list-create ct-pt-list-t
  (region mem-region-t))

(defun make-ct-pt-list ()
  (let* ((region (aa-mem-region-create 4096))
         (obj (aa-ct-pt-list-create region)))
    (setf (ct-pt-list-region obj) region)
    (mem-region-finalize region)
    obj))

(defcfun aa-ct-pt-list-add-qutr :void
  (pt-list ct-pt-list-t)
  (e :pointer))

(defun ct-pt-list-add-tf (pt-list tf)
  (with-tf-foreign-array (tf tf)
    (aa-ct-pt-list-add-qutr pt-list tf))
  pt-list)

(defcfun aa-ct-pt-list-add-q :void
  (pt-list ct-pt-list-t)
  (n size-t)
  (q :pointer))

(defun ct-pt-list-add-q (pt-list q)
  (with-foreign-simple-vector (q length) q :input
    (aa-ct-pt-list-add-q pt-list length q))
  pt-list)

(defun make-ct-pt-list-tf (tfs)
  (fold #'ct-pt-list-add-tf (make-ct-pt-list) tfs))

;;;;;;;;;;;;;;
;; Seg List ;;
;;;;;;;;;;;;;;


(amino-ffi::def-foreign-container ct-seg-list ct-seg-list-t
  :slots ((region nil :type (or null mem-region))))

(defcfun aa-ct-tjx-slerp-generate ct-seg-list-t
  (region mem-region-t)
  (pt-list ct-pt-list-t))

(defcfun aa-ct-tjq-lin-generate ct-seg-list-t
  (region mem-region-t)
  (pt-list ct-pt-list-t)
  (limits ct-limits-t))

(defcfun aa-ct-seg-list-duration :double
  (seg-list ct-seg-list-t))

(defun ct-tjq-lin (pt-list limits)
  (let* ((region (ct-pt-list-region pt-list))
         (segs (aa-ct-tjq-lin-generate region pt-list limits)))
    (setf (ct-seg-list-region segs) region)
    segs))

(defun ct-tjx-slerp (pt-list)
  (let* ((region (ct-pt-list-region pt-list))
         (segs (aa-ct-tjx-slerp-generate region pt-list)))
    (setf (ct-seg-list-region segs) region)
    segs))

(defcfun aa-ct-seg-list-eval-q :int
  (seg-list ct-seg-list-t)
  (time :double)
  (n size-t)
  (q :pointer))
