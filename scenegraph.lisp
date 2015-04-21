(in-package :robray)

(defvar *scene-graph*)


(defstruct scene-mesh
  name
  file)

(defstruct scene-box
  dimension)

(defstruct scene-sphere
  radius)

(defstruct scene-visual
  geometry
  color
  alpha)

(defstruct scene-frame
  name
  parent
  visual
  collision)

(defstruct (scene-frame-fixed (:include scene-frame))
  tf)

(defstruct (scene-frame-joint (:include scene-frame))
  axis
  (offset 0d0 :type double-float))

(defstruct (scene-frame-revolute (:include scene-frame-joint))
  translation)

(defstruct (scene-frame-prismatic (:include scene-frame-joint))
  rotation)


(defstruct scene-graph
  (frame-map (make-hash-table :test #'equal)))

(defun scene-graph-add-frame (scene-graph frame)
  (setf (gethash (scene-frame-name frame) (scene-graph-frame-map scene-graph))
        frame)
  scene-graph)

(defun scene-graph (frames)
  (fold #'scene-graph-add-frame
        (make-scene-graph)
        frames))

(defmacro do-scene-graph-frames ((frame-variable scene-graph &optional result) &body body)
  `(progn
     (loop for ,frame-variable being the hash-values of (scene-graph-frame-map ,scene-graph)
        do (progn ,@body))
     ,result))

(defun fold-scene-graph-frames (function initial-value scene-graph)
  (let ((value initial-value))
    (do-scene-graph-frames (frame scene-graph value)
      (setq value (funcall function value frame)))))

(defun scene-graph-lookup (scene-graph frame-name)
  (gethash frame-name (scene-graph-frame-map scene-graph)))

;; Find the relative transform of the scene frame
(defgeneric scene-frame-tf (object))


(defun scene-mesh-inc (mesh-file)
  "Return the include file for the mesh file"
  (concatenate 'string
               *robray-tmp-directory*
               "/povray"
               mesh-file ".inc"))

(defun scene-graph-resolve-mesh (scene-graph)
  (let ((mesh-files  ;; filename => (list mesh-nodes)
         (make-hash-table :test #'equal)))
    (labels ((resolve-mesh (mesh)
               (push mesh
                     (gethash (scene-mesh-file mesh) mesh-files))))
      ;; collect mesh files
      (do-scene-graph-frames (frame scene-graph)
        (loop
           for v in (scene-frame-visual frame)
           for g = (scene-visual-geometry v)
           when (and g (scene-mesh-p g))
           do (resolve-mesh g))))
    (maphash (lambda (mesh-file mesh-nodes)
               (format *standard-output* "~&Converting ~A..." mesh-file)
               (let* ((dom (dom-load mesh-file))
                      (inc-file (scene-mesh-inc mesh-file))
                      (geom-name (collada-geometry-name dom)))
                 (ensure-directories-exist inc-file)
                 (collada-povray :dom dom
                                 :file inc-file)
                 (dolist (mesh-node mesh-nodes)
                   (setf (scene-mesh-name mesh-node) geom-name))))
             mesh-files)))


(defun scene-frame-tf-relative (frame configuration-map default-configuration)
  "Find the relative TF for this frame"
  (etypecase frame
    (scene-frame-fixed (scene-frame-fixed-tf frame))
    (scene-frame-joint
     (let* ((config (+ (scene-frame-joint-offset frame)
                       (tree-map-find configuration-map (scene-frame-name frame)
                                     default-configuration)))
            (axis (scene-frame-joint-axis frame)))
       (etypecase frame
         (scene-frame-revolute
          (tf (with-vec3 (x y z) axis
                (axis-angle* x y z config))
              (scene-frame-revolute-translation frame)))
         (scene-frame-prismatic
          (tf (scene-frame-prismatic-rotation frame)
              (g* config axis))))))))

(defun scene-graph-tf-relative (scene-graph configuration-map
                                &key (default-configuration 0d0))
  (fold-scene-graph-frames (lambda (hash frame)
                             (setf (gethash (scene-frame-name frame) hash)
                                   (scene-frame-tf-relative frame configuration-map default-configuration))
                             hash)
                           (make-hash-table :test #'equal)
                           scene-graph))

(defun scene-graph-tf-absolute (scene-graph configuration-map
                                &key (default-configuration 0d0))
  (let ((tf-relative (scene-graph-tf-relative scene-graph configuration-map
                                              :default-configuration default-configuration)))
    (labels ((contains (hash frame)
               (nth-value 1 (gethash (scene-frame-name frame) hash)))
             (rec (tf-abs frame)
               (unless (contains tf-abs frame)
                 (let ((parent (scene-frame-parent frame))
                       (name (scene-frame-name frame)))
                   (if parent
                       (progn
                         (rec tf-abs (scene-graph-lookup scene-graph parent))
                         (setf (gethash name tf-abs)
                               (normalize (g* (gethash parent tf-abs)
                                              (gethash name tf-relative)))))
                       (setf (gethash name tf-abs)
                             (gethash name tf-relative)))))
               tf-abs))
      (fold-scene-graph-frames #'rec (make-hash-table :test #'equal) scene-graph))))


(defun scene-graph-configurations (scene-graph)
  "Returns the configuration variables in the scene graph."
  (fold-scene-graph-frames (lambda (list frame)
                             (if (scene-frame-joint-p frame)
                                 (cons (scene-frame-name frame) list)
                                 list))
                           nil
                           scene-graph))



;;;;;;;;;;;;;;
;;; POVRAY ;;;
;;;;;;;;;;;;;;

(defun scene-visual-pov (geometry tf)
  (let ((modifiers (list tf)))
    (etypecase geometry
      (scene-mesh (pov-mesh2 :mesh (scene-mesh-name geometry)
                             :modifiers modifiers))
      (scene-box
       (pov-box-center (scene-box-dimension geometry)
                       :modifiers modifiers))
      (scene-sphere
       (pov-sphere (vec3* 0 0 0)
                   (scene-sphere-radius geometry)
                   modifiers)))))

(defun scene-graph-transform-name (frame-name)
  (concatenate 'string
               "TF_"
               (name-mangle frame-name)))

(defun scene-graph-pov-configuration-transform (scene-graph configuration-map)
  (let ((tf-abs (scene-graph-tf-absolute scene-graph configuration-map
                                         :default-configuration 0d0))
        (result))
    (do-scene-graph-frames (frame scene-graph result)
      (let ((name (scene-frame-name frame)))
        (when (scene-frame-visual frame)
          (push (pov-declare (scene-graph-transform-name name)
                             (pov-transform* (pov-matrix (gethash name tf-abs))))
                result))))))

(defun scene-graph-pov-frame-transform (scene-graph frame-configuration-function start end)
  (pov-switch "frame_number"
              (loop for frame-number from start to end
                 collect (pov-case frame-number
                                   (scene-graph-pov-configuration-transform scene-graph
                                                                            (funcall frame-configuration-function
                                                                                     frame-number))))))

(defun scene-graph-pov-frame (scene-graph
                              &key
                                configuration-map
                                frame-start
                                frame-end
                                frame-configuration-function
                                output
                                directory
                                include
                                (default-configuration 0d0))
  (let ((pov-things))
    (labels ((thing (thing)
               (push thing pov-things))
             (include (file)
               (thing (pov-include file)))
             (tf (name)
               (scene-graph-transform-name name)))
      ;; push frame geometry
      (do-scene-graph-frames (frame scene-graph)
        (let* ((name (scene-frame-name frame)))
          (dolist (vis (scene-frame-visual frame))
            (let ((g (scene-visual-geometry vis)))
              (thing (scene-visual-pov g (pov-transform* (pov-value (tf name)))))))))
      ;; push frame transforms
      (when configuration-map
        (map nil #'thing
             (scene-graph-pov-configuration-transform scene-graph configuration-map)))
      (when frame-configuration-function
        (thing (scene-graph-pov-frame-transform scene-graph frame-configuration-function
                                                frame-start frame-end)))

      ;; push mesh include files
      (map-tree-set nil #'include
                    (fold-scene-graph-frames
                     (lambda (set frame)
                       (fold (lambda (set visual)
                               (let ((g (scene-visual-geometry visual)))
                                 (if (scene-mesh-p g)
                                     (tree-set-insert set
                                                      (scene-mesh-inc (scene-mesh-file g)))
                                     set)))
                             set
                             (scene-frame-visual frame)))
                     (make-tree-set #'string-compare)
                     scene-graph))
      (map nil #'include (reverse (ensure-list include))))
    ;; result
    (output (pov-sequence pov-things)
            output
            :directory directory)))
