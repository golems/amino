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

;; options
(amino-ffi::def-foreign-container rx-geom-opt rx-geom-opt-t
  :destructor aa-rx-geom-opt-destroy)

;; shapes
(amino-ffi::def-foreign-container scene-box rx-shape-box-t
  :struct-type shape-box)
(amino-ffi::def-foreign-container scene-sphere rx-shape-sphere-t
  :struct-type shape-sphere)
(amino-ffi::def-foreign-container scene-cylinder rx-shape-cylinder-t
  :struct-type shape-cylinder)
(amino-ffi::def-foreign-container scene-cone rx-shape-cone-t
  :struct-type shape-cone)
(amino-ffi::def-foreign-container scene-grid rx-shape-grid-t
  :struct-type shape-grid)

;; mesh
(amino-ffi::def-foreign-container rx-mesh rx-mesh-t
  :destructor rx-mesh-destroy)

;; geometry
(amino-ffi::def-foreign-container rx-geom rx-geom-t
  :destructor rx-geom-destroy)

;; scene graph
(amino-ffi::def-foreign-container mutable-scene-graph rx-sg-t
  :destructor aa-rx-sg-destroy
  :slots ((scene-graph)
          (config-name-array)
          (config-index-map)))

;; GL
(amino-ffi::def-foreign-container rx-gl-globals rx-gl-globals-t
  :destructor aa-rx-gl-globals-destroy)

;; sdl
(amino-ffi::def-foreign-container rx-win rx-win-t
  :slots ((mutable-scene-graph)
          (config-vector)))


;; Subgraph
(amino-ffi::def-foreign-container sub-scene-graph rx-sg-sub-t
  :destructor aa-rx-sg-sub-destroy
  :slots ((mutable-scene-graph)
          (config-index-map)
          (config-name-array)))

;; Collision
(amino-ffi::def-foreign-container rx-cl-set rx-cl-set-t
  :destructor aa-rx-cl-set-destroy
  :slots ((mutable-scene-graph)))


(amino-ffi::def-foreign-container rx-cl rx-cl-t
  :destructor aa-rx-cl-destroy
  :slots ((mutable-scene-graph)))

;; Planning
(amino-ffi::def-foreign-container rx-mp rx-mp-t
  :destructor aa-rx-mp-destroy
  :slots ((sub-scene-graph)))

;; Inverse Kinematics
(amino-ffi::def-foreign-container rx-ksol-opts rx-ksol-opts-t
  :destructor aa-rx-ksol-opts-destroy)

(amino-ffi::def-foreign-container rx-ik-jac-cx rx-ik-jac-cx-t
  :destructor aa-rx-ik-jac-cx-destroy)
