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

(defun map-lines (function file)
  (etypecase file
    (string (with-open-file (s file :direction :input)
              (map-lines function s)))
    (stream
     (loop for line = (read-line file nil nil)
        while line
        collect (funcall function line)))))

(defcfun aa-io-d-parse size-t
  (string :string)
  (reg mem-region-t)
  (X :pointer)
  (lendptr :pointer))

(defun parse-vector (string)
  (with-foreign-object (x :pointer)
    (let* ((reg (aa-mem-region-local-get))
           (size (aa-io-d-parse string reg x (null-pointer))))
      (assert (>=  size 0))
      (let ((ptr (mem-ref x :pointer))
            (array (make-vec size)))
        (dotimes (i size)
          (setf (aref array i)
                (mem-aref ptr :double i)))
        (aa-mem-region-pop reg x)
        array))))

(defun read-vectors (file)
  (loop for x in (map-lines #'parse-vector file)
     when (> (length x) 0)
     collect x))

(defun write-float-lists (lists file &key comment if-exists)
  (etypecase file
    (string
     (with-open-file (s file :direction :output :if-exists if-exists)
       (write-float-lists lists s :comment comment)))
    (stream
     (format file "~{~&# ~A~}"
             (etypecase comment
               (list comment)
               (string (list comment))))
     (format file "~{~&~{~F~^ ~}~}"
             lists))))

(defun write-vectors (vectors file &key comment if-exists)
  (write-float-lists (map 'list #'vec-list vectors) file
                     :comment comment
                     :if-exists if-exists))
