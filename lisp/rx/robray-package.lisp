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

(defpackage :robray
  (:use :cl :alexandria :amino :amino-ffi :sycamore :sycamore-util :sycamore-cgen)
  (:export

   ;; Frame Types
   :scene-frame-fixed
   :scene-frame-revolute
   :scene-frame-prismatic

   ;; Geometry
   :scene-box
   :scene-sphere
   :scene-cylinder
   :scene-cone


   ;; Draw interface
   :draw-options
   :draw-options-default
   :merge-draw-options
   :draw-option

   :draw-tf-axis

   :item-cylinder-axis
   :item-cone-axis
   :item-frame-axis

   ;; Render options
   :*render-options*
   :get-render-option
   :render-options-default
   :render-options
   :merge-render-options
   :render-options-fast
   :render-options-medium
   :render-options-full-hd

   ;; Scene Graph Manipulation
   :scene-graph
   :scene-graph-f
   :scene-graph-remove-frame
   :scene-graph-reparent
   :scene-graph-tf-absolute
   :*scene-directory*
   :load-scene-file


   ;; variables
   :*render-host-alist
   :*urdf-package-alist


   ;; sub-scene-graph
   :sub-scene-graph
   :sub-scene-graph-scene-graph
   :scene-graph-chain

   ;; motion-plan
   :motion-plan
   :motion-plan-endpoint-map
   :motion-plan-endpoint-array

   ))
