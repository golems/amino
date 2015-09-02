(in-package :robray)


(defun scene-frame-genc (argument-name frame)
  (labels ((static-double-array (name size values)
             (cgen-stmt (rope "static const double " name "[" size "] = "
                              (double-initializer values))))
           (double-initializer (thing)
             (let* ((sequence (etypecase thing
                                  (sequence thing)
                                  (amino::real-array
                                   (amino::real-array-data thing))))
                    (doubles (map 'list #'cgen-double-float sequence)))
               (cgen-array-initializer doubles))))
    (let* ((tf (scene-frame-tf frame))
           (name (scene-frame-name frame))
           (name-string (cgen-string (scene-frame-name frame)))
           (parent-string (cgen-string (scene-frame-parent frame)))
           (q (tf-quaternion tf))
           (v (tf-translation tf)))
      (list
       (cgen-line-comment (rope "FRAME: " name)))
      (cgen-block
       (static-double-array "q" 4 q)
       (static-double-array "v" 3 v)
       (when (scene-frame-joint-p frame)
         (cgen-stmt (static-double-array "axis" 3 (scene-frame-joint-axis frame))))
       (cgen-stmt
        (etypecase frame
          (scene-frame-fixed (cgen-call "aa_rx_sg_add_frame_fixed"
                                        argument-name
                                        parent-string name-string
                                        "q" "v"))
          (scene-frame-joint
           (cgen-stmt (cgen-call (etypecase frame
                                   (scene-frame-prismatic "aa_rx_sg_add_frame_prismatic")
                                   (scene-frame-revolute "aa_rx_sg_add_frame_revolute"))
                                 argument-name
                                 parent-string name-string
                                 "q" "v"
                                 (cgen-string (scene-frame-joint-configuration-name frame))
                                 "axis"
                                 (scene-frame-joint-configuration-offset frame))))))))))

(defun scene-graph-genc (scene-graph)
  (let ((argument-name "sg")
        (function-name "generate_scenegraph")
        (stmts))
    (labels ((item (x) (push x stmts)))
      ;; Lazily create object
      (item (cgen-if (cgen-equal "NULL" argument-name)
                     (cgen-stmt (cgen-assign argument-name
                                             (cgen-call "aa_rx_sg_create")))))
      ;; Map Frames
      (item (map-scene-graph-frames 'list (lambda (frame)
                                            (scene-frame-genc argument-name frame))
                                    scene-graph))

      ;; Return create object
      (item (cgen-return argument-name))
      ;; Ropify
      (sycamore-cgen::make-cgen-block
             :header (rope "struct aa_rx_sg *" function-name
                           (rope-parenthesize (rope "struct aa_rx_sg *" argument-name)))
             :stmts (flatten (reverse stmts))))))
