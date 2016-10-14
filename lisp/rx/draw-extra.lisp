;;;; -*- mode: lisp -*-
;;;;
;;;; Copyright (c) 2016, Rice University
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

(defun draw-e-paper (parent name &key
                                   tf
                                   (options (draw-options-default))
                                   (delta .1)
                                   (major-width (* .05 delta))
                                   (minor-width (* .05 delta .5))
                                   (offset 1e-3)
                                   (x 1)
                                   (y 1)
                                   (paper-color (vec .91 .96 .88))
                                   (grid-color (vec 0 .5 0)))
  (let ((grid-major (rope name "-grid-major"))
        (grid-minor (rope name "-grid-minor"))
        (grid-options (merge-draw-options (draw-options :color grid-color)
                                          options)))
    (scene-graph
     (scene-frame-fixed parent name
                        :geometry (when paper-color (scene-geometry-box (merge-draw-options (draw-options :color paper-color)
                                                                                            options)
                                                                        (vec3* x y offset)))
                        :tf (tf tf))
     (scene-frame-fixed name grid-major
                        :geometry (scene-geometry-grid grid-options
                                                       :dimension (vec (/ x 2) (/ y 2))
                                                       :delta (vec (* 5 delta) (* 5 delta))
                                                       :width major-width)
                        :tf (tf* nil (vec3* 0 0 1e-3)))
     (scene-frame-fixed grid-major grid-minor
                        :geometry (scene-geometry-grid grid-options
                                                       :dimension (vec (/ x 2) (/ y 2))
                                                       :delta (vec delta delta)
                                                       :width minor-width)
                        :tf (tf* nil nil)))))

(defun draw-logo-arm (parent name
                      &key
                        (length-0 1d0)
                        (length-1 1d0)
                        (link-specular 2d0)
                        (radius .05))
  (let* ((link-specular (coerce link-specular 'double-float))
         (options (draw-options-default :color '(.5 .5 .5)
                                        :visual t
                                        :specular (list link-specular
                                                        link-specular
                                                        link-specular)))
         (joint-options (merge-draw-options (draw-options :color '(0 0 0)
                                                          :specular '(.5 .5 .5)) options))
         (finger-radius (* .33 radius))
         (finger-length (* 3 radius))
         (hand-radius (* 2 radius))
         (finger-base (- hand-radius (* 1.5 finger-radius))))
    (flet ((finger (angle name)
             (scene-graph
              (scene-frame-revolute "hand" (rope "f" name)
                                    :axis '(0 -1 0)
                                    :geometry (list (scene-geometry-cylinder options
                                                                             :height finger-length
                                                                             :radius finger-radius)
                                                    (scene-geometry-sphere joint-options
                                                                           (* 1.5 finger-radius)))

                                    :tf (tf* (z-angle angle)
                                             (vec3* (* finger-base (cos angle))
                                                    (* finger-base (sin angle))
                                                    0)))

              (scene-frame-fixed (rope "f" name) (rope "end" name)
                                 :tf (tf* nil (vec3* 0 0 finger-length))
                                 :geometry (scene-geometry-sphere options
                                                                  finger-radius))
              )))
      (scene-graph
       ;; roll-pitch-yaw
       (scene-frame-fixed parent name)
       (scene-frame-revolute name "s0"
                             :axis '(1 0 0)
                             :geometry (scene-geometry-sphere joint-options (* 1.5 radius)))
       (scene-frame-revolute "s0" "s1"
                             :axis '(0 1 0))
       (scene-frame-revolute "s1" "s2"
                             :axis '(1 0 0))

       (scene-frame-revolute "s2" "e"
                             :axis '(0 1 0)
                             :tf (tf* nil (vec3* length-0 0 0))
                             )

       (scene-frame-fixed "e" "e-joint-link"
                          :tf (tf* (x-angle (/ pi 2)) (vec3* 0 radius 0))
                          :geometry (scene-geometry-cylinder  joint-options
                                                              :radius (* 1.25 radius)
                                                              :height (* 2 radius)))

       (scene-frame-revolute "e" "w0"
                             :axis '(1 0 0)
                             :tf (tf* nil (vec3* length-1 0 0)))
       (scene-frame-revolute "w0" "w1"
                             :axis '(0 1 0))
       (scene-frame-revolute "w1" "w2"
                             :axis '(0 0 1)
                             :geometry (scene-geometry-sphere joint-options (* 1.5 radius)))

       (scene-frame-fixed "s2" "upper-link"
                          :geometry (scene-geometry-cylinder options
                                                             :height length-0
                                                             :radius radius)
                          :tf (tf* (y-angle (/ pi 2)) nil))
       (scene-frame-fixed "e" "lower-link"
                          :geometry (scene-geometry-cylinder options
                                                             :height length-1
                                                             :radius radius)
                          :tf (tf* (y-angle (/ pi 2)) nil))



       (scene-frame-fixed "w2" "hand"
                          :geometry (scene-geometry-cone options
                                                         :height (- radius)
                                                         :end-radius radius
                                                         :start-radius hand-radius)
                          :tf (tf* (y-angle (/ pi 2)) (vec3* (* 2 radius) 0 0)))

       ;; (scene-frame-fixed nil "label"
       ;;                    :tf (tf* (x-angle (/ pi -2)) (vec3* -.565 -.6 .01))
       ;;                    :geometry (scene-geometry-text (draw-options-default :scale .45 :color '(.1 .1 .1)
       ;;                                                                         :no-shadow t)
       ;;                                                   "AMINO" :thickness .5))


       (finger (* 1.5 pi) "0")
       (finger (* (+ 1.5 (/ 2 3)) pi) "1")
       (finger (* (- 1.5 (/ 2 3)) pi)  "2"))
      )))
