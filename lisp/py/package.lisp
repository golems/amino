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

(defpackage aminopy
  (:use :cl :alexandria :amino :sycamore :sycamore-util :robray)
  (:nicknames |aminopy|)
  (:export
   ;;; Transforms
   |inverse|
   |tf|
   |rotation|
   |translation|
   |tf2|
   |vec|
   |xangle|
   |yangle|
   |zangle|

   |vec3|
   |quat|

   |mul|

   ;;; Scene Graphs
   ;;|scene_frame_tf|
   |load_scene|

   |scene_tf_abs|
   |scene_tf_rel|
   |frame_fixed_tf|
   |map_frames|

   |scene_chain|

   ;;; frame functions
   |frame_isa|


   ;;; Shapes
   |shape_is_box|
   |shape_is_sphere|
   |shape_is_cylinder|
   |shape_is_cone|
   |shape_is_grid|
   |shape_is_text|
   |shape_is_mesh|

   ;;; Window
   |win_set_scene|
   |win_run_sync|
   |win_run_async|
   |win_view_collision|
   |win_view_visual|

   ;; Motion Planning
   ;;|motion_plan_ws|
   |motion_plan|
   |motion_plan_endpoint|
   ))
