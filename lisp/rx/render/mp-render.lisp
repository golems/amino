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

(defun render-motion-plan (motion-plan
                          &key
                            (delta-t .25d0)
                            config-velocity
                            (camera-tf (tf nil))
                            include
                            render
                            include-text
                            (options robray::*render-options*))
  (let* ((n (motion-plan-length motion-plan))
         (keyframes (keyframe-set (loop
                                     for i below n
                                     for config = (motion-plan-refmap motion-plan i)
                                     for array = (motion-plan-refarray motion-plan i)
                                     for time = 0d0 then (if config-velocity
                                                             (+ time (/ (vec-dist array last-array)
                                                                        config-velocity))
                                                             (+ time delta-t))
                                     for last-array = array
                                     collect (joint-keyframe time config))))
         (sg (motion-plan-scene-graph motion-plan)))
    (scene-graph-time-animate (keyframe-configuration-function keyframes)
                              :camera-tf camera-tf
                              :options options
                              :encode-video t
                              :render-frames render
                              :include include
                              :include-text include-text
                              :scene-graph sg)))

(defun render-motion-plans (plans
                            &key
                              (delta-t .10d0)
                              ;config-velocity
                              (camera-tf (tf nil))
                              include
                              include-text
                              render
                              (directory *robray-tmp-directory*)
                              (options *render-options*))

  (map nil #'delete-file (frame-files directory))
  ;; Write povray files
  (loop
     with fps = (get-render-option options :frames-per-second)
     with period = (/ 1d0 fps)
     for time = 0d0 then (keyframe-set-end keyframes)
     ;for t0 = time
     for plan in plans
     for keyframes =
       (keyframe-set (loop
                        for i below (motion-plan-length plan)
                        for config = (motion-plan-refmap plan i)
                        for array = (motion-plan-refarray plan i)
                        for last-array = array
                        collect (prog1 (joint-keyframe time config)
                                  (incf time delta-t))))
     for function = (let* ((t0 (keyframe-set-start keyframes))
                           (t1 (keyframe-set-end keyframes))
                           (fun (keyframe-configuration-function keyframes)))
                      (lambda (i)
                        (let ((tt (* i period)))
                          ;;(format t "~&~D (~As, ~A, ~A) [~A, ~A]" i tt fps period t0 t1)
                          (funcall fun tt))))

     do
       (scene-graph-frame-animate function
                                   :camera-tf camera-tf
                                   :append t
                                   :options options
                                   :encode-video nil
                                   :render-frames nil
                                   :include include
                                   :include-text include-text
                                   :scene-graph (motion-plan-scene-graph plan)))
  ;; Render
  (when render
    (net-render :directory directory
                :options options)))
