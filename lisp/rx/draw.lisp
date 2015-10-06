(in-package :robray)

;;; GEOMETRY DRAWING ;;;

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
                     :geometry (scene-geometry (scene-cylinder :height height :radius radius)
                                               options)))


(defun item-cone-axis (parent name
                       &key height start-radius (end-radius 0d0) axis (translation (identity-vec3))
                         options)
  (scene-frame-fixed parent name
                     :tf (draw-tf-axis axis translation)
                     :geometry (scene-geometry (scene-cone :height height
                                                           :start-radius start-radius
                                                           :end-radius end-radius)
                                               options)))

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
