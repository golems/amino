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


(defun motion-plan-configuration-function-time (motion-plan)
  (let* ((pts (amino::make-ct-pt-list))
         (m-sg (motion-plan-mutable-scene-graph motion-plan))
         (names (mutable-scene-graph-config-name-array m-sg))
         (n-pts (motion-plan-point-count motion-plan)))
    (dotimes (i n-pts)
      (amino::ct-pt-list-add-q pts (motion-plan-refarray motion-plan i)))
    (let* ((segs (amino::ct-tjq-lin pts (aa-rx-ct-sg-limits (amino::ct-pt-list-region pts)
                                                            m-sg)))
           (duration (amino::aa-ct-seg-list-duration segs)))
      (lambda (time)
        (when (<= time duration)
          (let ((q (make-vec (mutable-scene-graph-config-count m-sg))))
            (with-foreign-simple-vector (q n-q) q :output
              (amino::aa-ct-seg-list-eval-q segs time n-q q))
            (pairlist-configuration-map names q)))))))

(defun motion-plan-configuration-function-frame (motion-plan &key
                                                               (fps 30))
  (let ((time-function (motion-plan-configuration-function-time motion-plan)))
    (lambda (frame)
      (funcall time-function (coerce (/ frame fps) 'double-float)))))

(defun render-motion-plan (motion-plan
                          &key
                            (camera-tf (tf nil))
                            include
                            render
                            include-text
                            (options robray::*render-options*))
    (scene-graph-time-animate (motion-plan-configuration-function-time motion-plan)
                              :camera-tf camera-tf
                              :options options
                              :encode-video t
                              :render-frames render
                              :include include
                              :include-text include-text
                              :scene-graph (motion-plan-scene-graph motion-plan)))

(defun render-motion-plans (plans
                            &key
                              (camera-tf (tf nil))
                              include
                              include-text
                              render
                              encode-video
                              (directory *robray-tmp-directory*)
                              (options *render-options*))

  (map nil #'delete-file (frame-files directory))
  ;; Write povray files
  (loop
     with fps = (get-render-option options :frames-per-second)
     for plan in plans
     do
       (scene-graph-frame-animate (motion-plan-configuration-function-frame plan :fps fps)
                                  :camera-tf camera-tf
                                  :append t
                                  :options options
                                  :render-frames nil
                                  :include include
                                  :include-text include-text
                                  :scene-graph (motion-plan-scene-graph plan)))


  (finish-render :output-directory directory
                 :render-frames render
                 :encode-video encode-video
                 :options options))
