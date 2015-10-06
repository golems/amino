;;;; -*- mode: lisp -*-
;;;;
;;;; Copyright (c) 2015, Rice University
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

(in-package :robray)


(defun load-trajectory-data (place)
  (etypecase place
    (rope (with-open-file (stream (rope-string place) :direction :input)
            (load-trajectory-data stream)))
    (stream
     (loop for line = (read-line place nil nil)
        while line
        collect (parse-float-sequence line)))))


(defun %load-trajectory (stream names time time-step default-map clip-start)
  ;; find header
  (let ((lineno 0))
    (unless names
      (block find-header
        (loop for line = (progn (incf lineno)
                                (read-line stream nil nil))
           while line
           for line2 = (strip-hash-comment line)
           do (when line2
                (setq names (split-spaces line2))
                (return-from find-header))))
      (when (equalp (car names) "time")
        (setq names (cdr names))))
    ;; load data
    (keyframe-set
     (loop for line = (progn (incf lineno)
                             (read-line stream nil nil))
        for i from 0
        while line
        for line2 = (strip-hash-comment line)
        when line2
        collect (let ((line-data (parse-float-sequence line2)))
                  (multiple-value-bind (time config)
                      (if time-step
                          (values (+ time (* i time-step))
                                  line-data)
                          (values (car line-data)
                                  (cdr line-data)))
                    (joint-keyframe time
                                    (pairlist-configuration-map names config default-map)))))
     :clip-start clip-start)))

(defun load-trajectory (place &key names default-map (time 0d0) time-step clip-start)
  (labels ((helper (stream)
             (%load-trajectory stream names time time-step default-map clip-start)))
    (etypecase place
      (rope
       (with-open-file (stream (rope-string place) :direction :input)
         (helper stream)))
      (stream
       (helper place)))))
