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

(defmethod object-rope ((object scene-graph))
  (rope (map-scene-graph-frames 'list #'object-rope object)))

(defmacro scenefile-indent-string (n)
  (make-string n :initial-element #\Tab))

(defun scenefile-sequence-array (sequence)
  (format nil "[~{~F~^, ~}]"
          (vec-list sequence)))

(defun scenefile-array (&rest values)
  (scenefile-sequence-array values))

(defmethod frame-property-rope (name value)
  (rope (scenefile-indent-string 1)
        name " " value ";" #\Newline))

(defun frame-rope (object)
  (rope "frame " (scene-frame-name object) #.(format nil " {~%")
        ;; parent
        (when-let ((v (scene-frame-parent object)))
          (frame-property-rope "parent" v))
        ;; type
        (frame-property-rope "type" (etypecase object
                                      (scene-frame-fixed "fixed")
                                      (scene-frame-prismatic "prismatic")
                                      (scene-frame-revolute "revolute")))
        ;; TODO: limits
        ;; TODO: inertial
        ;; tf
        (when-let ((tf (scene-frame-tf object)))
          (let ((q (tf-quaternion tf))
                (v (tf-translation tf)))
            (list
             (unless (vec3-identity-p v)
               (with-vec3 (x y z) v
                 (frame-property-rope "translation" (scenefile-array x y z))))
             (unless (quaternion-identity-p q)
               (with-quaternion (x y z w) q
                 (frame-property-rope "quaternion" (scenefile-array x y z w)))))))
        ;; joint stuff
        (when (scene-frame-joint-p object)
          (list (frame-property-rope "variable" (scene-frame-joint-configuration-name object))
                (frame-property-rope "axis" (scenefile-sequence-array (scene-frame-joint-axis object)))
                (frame-property-rope "offset" (scene-frame-joint-configuration-offset object))))
        (map 'list #'object-rope (scene-frame-geometry object))
        ;; geometry
        #.(format nil "}~%~%")))

(defmethod object-rope ((object scene-frame-revolute))
  (frame-rope object))

(defmethod object-prismatic ((object scene-frame-revolute))
  (frame-rope object))

(defmethod object-rope ((object scene-frame-fixed))
  (frame-rope object))

(defmethod geometry-property-rope (name value)
  (rope (scenefile-indent-string 2) name " " value #.(format nil ";~%")))

(defgeneric geometry-rope (geometry shape))

(defmethod object-rope ((object scene-geometry))
  (let ((options (scene-geometry-options object)))
    (rope #.(format nil "~Ageometry {~%" (scenefile-indent-string 1))
          (unless (draw-option options :visual)
            (geometry-property-rope "visual" 0))
          (unless (draw-option options :collision)
            (geometry-property-rope "collision" 0))
          (geometry-rope object (scene-geometry-shape object))
          (when-let ((color (draw-option options :color)))
            (geometry-property-rope "color" (scenefile-sequence-array color)))
          (when-let ((alpha (draw-option options :alpha)))
            (geometry-property-rope "alpha" alpha))
        #.(format nil "~A}~%" (scenefile-indent-string 1)))))

(defmethod geometry-rope ((geometry scene-geometry) (shape scene-box))
  (rope (geometry-property-rope "shape" "box")
        (geometry-property-rope "dimension" (scenefile-sequence-array (scene-box-dimension shape)))))

(defmethod geometry-rope ((geometry scene-geometry) (shape scene-grid))
  (rope (geometry-property-rope "shape" "grid")
        (geometry-property-rope "dimension" (scenefile-sequence-array (scene-grid-dimension shape)))
        (geometry-property-rope "thickness" (scene-grid-thickness shape))))

(defmethod geometry-rope ((geometry scene-geometry) (shape scene-sphere))
  (rope (geometry-property-rope "shape" "sphere")
        (geometry-property-rope "radius" (scene-sphere-radius shape))))

(defmethod geometry-rope ((geometry scene-geometry) (shape scene-cylinder))
  (rope (geometry-property-rope "shape" "cylinder")
        (geometry-property-rope "radius" (scene-cylinder-radius shape))
        (geometry-property-rope "height" (scene-cylinder-height shape))))

(defmethod geometry-rope ((geometry scene-geometry) (shape scene-mesh))
  (rope (geometry-property-rope "mesh" (rope #\" (scene-mesh-source-file shape) #\"))))
