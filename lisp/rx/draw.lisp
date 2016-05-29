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

;;; GEOMETRY DRAWING ;;;

(defun octet-color (r g b)
  (list (/ r 255)
        (/ g 255)
        (/ b 255)))

(defun draw-tf-axis (axis &optional (translation (identity-vec3)))
  (tf* (quaternion-from-vectors (vec 0d0 0d0 1d0)
                                axis)
                      translation))

(defun draw-subframe (parent name)
  (format nil "~A/~A" parent name))

(defun item-cylinder-axis (parent name &key height radius axis (translation (identity-vec3))
                                  options)
  (scene-frame-fixed parent name
                     :tf (draw-tf-axis axis translation)
                     :geometry (scene-geometry-cylinder options  :height height :radius radius)))


(defun item-cone-axis (parent name
                       &key height start-radius (end-radius 0d0) axis (translation (identity-vec3))
                         options)
  (scene-frame-fixed parent name
                     :tf (draw-tf-axis axis translation)
                     :geometry (scene-geometry-cone options :height height
                                                    :start-radius start-radius
                                                    :end-radius end-radius)))

(defun item-arrow-axis (parent name
                        &key
                          axis
                          length
                          width
                          end-arrow
                          start-arrow
                          (end-arrow-start-width (* 2 width))
                          (end-arrow-end-width 0d0)
                          (end-arrow-length width)
                          (start-arrow-start-width (* 2 width))
                          (start-arrow-end-width 0d0)
                          (start-arrow-length width)
                          (translation (identity-vec3))
                          options)

  (let ((body-length (- length
                        (if start-arrow start-arrow-length 0)
                        (if end-arrow end-arrow-length 0))))

    (nconc (when start-arrow
             (list (item-cone-axis parent (draw-subframe name "start-arrow")
                                   :options options
                                   :height start-arrow-length
                                   :start-radius (/ start-arrow-start-width 2)
                                   :end-radius (/ start-arrow-end-width 2)
                                   :axis axis
                                   :translation  (g+ (g* axis start-arrow-length)
                                                             translation))))
           (list (item-cylinder-axis parent (draw-subframe name "body")
                                     :options options
                                     :height body-length :radius (/ width 2)
                                     :axis axis
                                     :translation (if start-arrow
                                                      (g+ (g* start-arrow-length axis)
                                                          translation)
                                                      translation)))
           (when end-arrow
             (list (item-cone-axis parent (draw-subframe name "end-arrow")
                                   :options options
                                   :height end-arrow-length
                                   :start-radius (/ end-arrow-start-width 2)
                                   :end-radius (/ end-arrow-end-width 2)
                                   :axis axis
                                   :translation  (g+ (g* axis (- length end-arrow-length))
                                                     translation)))))))

(defun item-frame-marker (parent name
                          &key
                            length
                            width
                            (arrow-width (* 2 width))
                            (arrow-length (* 1 arrow-width))
                            options)
  (flet ((helper (subname axis color)
           (item-arrow-axis parent (draw-subframe name subname)
                            :options (merge-draw-options (draw-options :color color)
                                                         options)
                            :axis axis
                            :length length
                            :width width
                            :end-arrow t
                            :end-arrow-start-width arrow-width
                            :end-arrow-length arrow-length)))

  (append (helper "x" (vec3* 1 0 0) '(1 0 0) )
          (helper "y" (vec3* 0 1 0) '(0 1 0) )
          (helper "z" (vec3* 0 0 1) '(0 0 1) ))))


;; (defun draw-geometry (scene-graph parent name
;;                       &key
;;                         geometry
;;                         tf
;;                         (actual-parent parent)
;;                         (options *draw-options*))
;;   (scene-graph-add-tf scene-graph (tf-tag parent tf name)
;;                       :actual-parent actual-parent
;;                       :geometry (scene-geometry geometry options)))

;; (defun draw-items (scene-graph parent items
;;                    &key
;;                      (options *draw-options*))
;;   (fold (lambda (scene-graph x)
;;           (let ((frame (copy-structure x)))
;;             (unless (scene-frame-parent frame)
;;               (setf (scene-frame-parent frame)
;;                     parent))
;;             (setf (scene-frame-geometry frame)
;;                   (map 'list (lambda (g)
;;                                (scene-geometry (scene-geometry-shape g)
;;                                                (merge-draw-options (merge-draw-options (scene-geometry-options g)
;;                                                                                        options))))
;;                        (scene-frame-geometry frame)))
;;             (scene-graph scene-graph frame)))
;;         scene-graph
;;         (ensure-list items)))
