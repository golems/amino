(in-package :robray)

(defvar *genframe-count*)

(defun draw-genframe (&optional (thing "ROBRAY_GENFRAME__"))
  (unless (boundp '*genframe-count*)
    (setq *genframe-count* 0))
  (format nil "~A~D" thing (incf *genframe-count*)))

(defun draw-options (&key
                       no-shadow
                       (color '(0 0 0))
                       (alpha 1d0)
                       (visual t)
                       (collision t))
  (list (cons :no-shadow no-shadow)
        (cons :color color)
        (cons :alpha alpha)
        (cons :visual visual)
        (cons :collision collision)))

(defparameter *draw-options* (draw-options))

(defun draw-option (draw-options option)
  (cdr (assoc option draw-options)))

(defun draw-geometry (scene-graph parent name
                      &key
                        geometry
                        tf
                        (options *draw-options*)
                        (no-shadow (draw-option options :no-shadow))
                        (color (draw-option options :color))
                        (alpha (draw-option options :alpha))
                        (visual (draw-option options :visual))
                        (collision (draw-option options :collision)))
  (declare (ignore collision))
  ;; Add frame
  (setq scene-graph
        (scene-graph-add-frame scene-graph
                               (scene-frame-fixed parent name
                                                  :tf tf)))
  ;; Add visual
  (when visual
    (setq scene-graph
          (scene-graph-add-visual scene-graph name
                                  (make-scene-visual :geometry geometry
                                                     :color color
                                                     :alpha alpha
                                                     :modifiers (when no-shadow
                                                                  '(:no-shadow))))))
  scene-graph)


(defun draw-tf-axis (axis &optional (translation (vec3* 0d0 0d0 0d0)))
  (tf* (quaternion-from-vectors (vec 0d0 0d0 1d0)
                                axis)
                      translation))
