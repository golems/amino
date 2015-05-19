(in-package :robray)

;;; OPTIONS ;;;
(defparameter *draw-options*
  '((:no-shadow . nil)
    (:color . (0 0 0))
    (:alpha . 1d0)
    (:visual . t)
    (:collision . t)))

(defun get-draw-option (draw-options option)
  (cdr (assoc option draw-options)))

(defun draw-options-default (&key
                               (options *draw-options*)
                               (no-shadow (get-draw-option options :no-shadow))
                               (color (get-draw-option options :color))
                               (alpha (get-draw-option options :alpha))
                               (visual (get-draw-option options :visual))
                               (collision (get-draw-option options :collision)))
  (list* (cons :no-shadow no-shadow)
         (cons :color color)
         (cons :alpha alpha)
         (cons :visual visual)
         (cons :collision collision)
         options))

(defun draw-options (&rest options-plist)
  (plist-alist options-plist))

(defun merge-draw-options (new-options &optional (base-options *draw-options*))
  (append new-options base-options))


;;; GEOMETRY DRAWING ;;;

(defun draw-geometry (scene-graph parent name
                      &key
                        geometry
                        tf
                        (actual-parent parent)
                        (options *draw-options*))
  (scene-graph-add-tf scene-graph (tf-tag parent tf name)
                      :actual-parent actual-parent
                      :geometry (scene-geometry geometry options)))

(defun draw-tf-axis (axis &optional (translation (identity-vec3)))
  (tf* (quaternion-from-vectors (vec 0d0 0d0 1d0)
                                axis)
                      translation))



(defun draw-subframe (parent name)
  (format nil "~A/~A" parent name))

(defstruct draw-item
  name
  geometry
  tf
  options)

(defun draw-items (scene-graph parent items
                   &key
                     (options *draw-options*))
  (fold (lambda (scene-graph x)
          ;(print scene-graph)
          ;(print (draw-item-name x))
          (draw-geometry scene-graph parent
                         (draw-item-name x)
                         :options (merge-draw-options (draw-item-options x) options)
                         :geometry (draw-item-geometry x)
                         :tf (draw-item-tf x)))
        scene-graph
        (ensure-list items)))

(defun item-cylinder-axis (name &key height radius axis (translation (identity-vec3))
                                  options)
  (make-draw-item :name name
                  :geometry (scene-cylinder :height height
                                            :radius radius)
                  :tf (draw-tf-axis axis translation)
                  :options options))

(defun item-cone-axis (name &key height start-radius (end-radius 0d0) axis (translation (identity-vec3))
                              options)
  (make-draw-item :name name
                  :geometry (scene-cone :height height
                                        :start-radius start-radius
                                        :end-radius end-radius)
                  :tf (draw-tf-axis axis translation)
                  :options options))

(defun item-arrow-axis (name &key
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
             (list (item-cone-axis (robray::draw-subframe name "start-arrow")
                                   :options options
                                   :height start-arrow-length
                                   :start-radius (/ start-arrow-start-width 2)
                                   :end-radius (/ start-arrow-end-width 2)
                                   :axis axis
                                   :translation  (g+ (g* axis start-arrow-length)
                                                             translation))))
           (list (item-cylinder-axis (robray::draw-subframe name "body")
                                     :options options
                                     :height body-length :radius (/ width 2)
                                     :axis axis
                                     :translation (if start-arrow
                                                      (g+ (g* start-arrow-length axis)
                                                          translation)
                                                      translation)))
           (when end-arrow
             (list (item-cone-axis (robray::draw-subframe name "end-arrow")
                                   :options options
                                   :height end-arrow-length
                                   :start-radius (/ end-arrow-start-width 2)
                                   :end-radius (/ end-arrow-end-width 2)
                                   :axis axis
                                   :translation  (g+ (g* axis (- length end-arrow-length))
                                                     translation)))))))

(defun item-frame-marker (name &key
                                 length
                                 width
                                 (arrow-width (* 2 width))
                                 (arrow-length (* 1 arrow-width)))
  (flet ((helper (subname axis color)
           (item-arrow-axis (draw-subframe name subname)
                            :options (draw-options :color color)
                            :axis axis
                            :length length
                            :width width
                            :end-arrow t
                            :end-arrow-start-width arrow-width
                            :end-arrow-length arrow-length)))

  (append (helper "x" (vec3* 1 0 0) '(1 0 0) )
          (helper "y" (vec3* 0 1 0) '(0 1 0) )
          (helper "z" (vec3* 0 0 1) '(0 0 1) ))))
