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


(defpackage :amino-baxter
  (:use :cl :amino :robray :sycamore-util :sycamore)
  (:export
   :draw-baxter)

(in-package :amino-baxter)

(defparameter *baxter-allowed-collision*
  '(("right_w0_fixed" . "right_wrist-collision")))

(defparameter *baxter-config-names*
  '("right_s0" "right_s1"
    "right_e0" "right_e1"
    "right_w0" "right_w1" "right_w2"))


(defun draw-baxter (parent name &key
                                  (urdf "package://baxter_description/urdf/baxter.urdf")
                                  (ros-package-path "/opt/ros/indigo/share"))
  (declare (ignore name parent))
  ;; set package path
  (when ros-package-path
    #+sbcl
    (sb-posix:setenv "ROS_PACKAGE_PATH" ros-package-path 1))
  (labels ((right-joint-config (values)
             (robray::configuration-map-pairs '("right_s0" "right_s1"
                                                "right_e0" "right_e1"
                                                "right_w0" "right_w1" "right_w2")
                                                values)))
    (let ((base (load-scene-file urdf)))
      (fold #'robray::scene-graph-allow-configuration
            (scene-graph-allow-collisions base *baxter-allowed-collision*)
            (list nil
                  (right-joint-config '(0.375973 -1.44985 0.555649
                                        2.54396 -0.133194 0.498291 0.260089))))
      )))
