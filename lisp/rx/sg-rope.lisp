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

(defun scenefile-array (&rest values)
  (format nil "[~{~F~^, ~}]" values))


(defmethod frame-property-rope (name value)
  (rope (scenefile-indent-string 1)
        name " " value ";" #\Newline))


(defmethod object-rope ((object scene-frame-fixed))
  (rope "frame " (scene-frame-name object) #.(format nil " {~%")
        ;; parent
        (when-let ((v (scene-frame-parent object)))
          (frame-property-rope "parent" v))
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
        (map 'list #'object-rope (scene-frame-geometry object))
        ;; geometry
        #.(format nil "}~%~%")))


(defmethod geometry-property-rope (name value)
  (rope (scenefile-indent-string 2) name " " value #.(format nil ";~%")))

(defgeneric geometry-rope (geometry shape))

(defmethod object-rope ((object scene-geometry))
  (let ((options (scene-geometry-options object)))
    (rope #.(format nil "~Ageometry {~%" (scenefile-indent-string 1))
          (geometry-rope object (scene-geometry-shape object))
          (when-let ((color (draw-option options :color)))
            (geometry-property-rope "color" (scenefile-array (vecref color 0)
                                                             (vecref color 1)
                                                             (vecref color 2))))
          (when-let ((alpha (draw-option options :alpha)))
            (geometry-property-rope "alpha" alpha))
        #.(format nil "~A}~%" (scenefile-indent-string 1)))))


(defmethod geometry-rope ((geometry scene-geometry) (shape scene-box))
  (rope (geometry-property-rope "shape" "box")
        (geometry-property-rope "dimension" (scenefile-array (scene-box-x shape)
                                                             (scene-box-y shape)
                                                             (scene-box-z shape)))))
