(in-package :robray)

(defun export-scene-graph-moveit (scene-graph
                                  &key
                                    (name "noname")
                                    (stream *standard-output*))
  (format stream "~&(~A)" name)
  ;; * NAME
  ;; 1
  ;; type
  ;; size
  ;; translation
  ;; quaternion
  ;; RGBA
  (do-scene-graph-visuals ((frame-name  visual) scene-graph)
    (format stream "~&* ~A" frame-name)
    (format stream "~&1")
    (let ((g (scene-visual-geometry visual)))
      (etypecase g
        (scene-box
         (with-vec3 (x y z) (scene-box-dimension g)
           (format stream "~&box~&~F ~F ~F" x y z)))))
    (let* ((tf (scene-graph-tf-absolute scene-graph frame-name))
           (v (tf-translation tf))
           (q (tf-quaternion tf)))
      ;; translation
      (format stream "~&~F ~F ~F"
              (vecref v 0)
              (vecref v 1)
              (vecref v 2))
      ;; quaternion
      (format stream "~&~F ~F ~F ~F"
              (quaternion-x q)
              (quaternion-y q)
              (quaternion-z q)
              (quaternion-w q)))
    (let ((color (scene-visual-color visual))
          (alpha (scene-visual-alpha visual)))
      ;; quaternion
      (format stream "~&~F ~F ~F ~F"
              (vecref color 0)
              (vecref color 1)
              (vecref color 2)
              alpha))))
