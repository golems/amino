(in-package :robray)

(defvar *scene-graph*)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; SCENE FRAMES STRUCTURES ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; GEOMETRY


(defstruct scene-mesh
  name
  file)

(defstruct scene-box
  dimension)
(defun scene-box (dimension)
  (make-scene-box :dimension dimension))

(defstruct scene-sphere
  radius)
(defun scene-sphere (radius)
  (make-scene-sphere :radius radius))


(defstruct (scene-cylinder (:constructor scene-cylinder))
  height
  radius)

(defstruct (scene-cone (:constructor scene-cone))
  height
  start-radius
  end-radius)

(defstruct scene-visual
  geometry
  color
  (alpha 1d0 :type double-float)
  modifiers)

(defstruct scene-frame
  name
  parent
  visual
  collision)

(defstruct (scene-frame-fixed (:include scene-frame))
  tf)

(defun scene-frame-fixed (parent name &key tf)
  (make-scene-frame-fixed :name name
                          :parent parent
                          :tf tf))


(defstruct (scene-frame-joint (:include scene-frame))
  axis
  (offset 0d0 :type double-float))

(defstruct (scene-frame-revolute (:include scene-frame-joint))
  translation)

(defun scene-frame-revolute (parent name &key
                                           axis
                                           (offset 0d0)
                                           (translation (vec3* 0d0 0d0 0d0)))
  (assert axis)
  (make-scene-frame-revolute :name name
                             :parent parent
                             :axis axis
                             :offset offset
                             :translation translation))

(defstruct (scene-frame-prismatic (:include scene-frame-joint))
  rotation)

(defun scene-frame-prismatic (parent name &key
                                            axis
                                            (offset 0d0)
                                            (rotation (vec3* 0d0 0d0 0d0)))
  (assert axis)
  (make-scene-frame-prismatic :name name
                              :parent parent
                              :axis axis
                              :offset offset
                              :rotation rotation))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; SCENE GRAPH STRUCTURE ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct scene-graph
  (frame-map (make-tree-map #'frame-name-compare)))

(defun scene-graph-add-frame (scene-graph frame)
  "Add a frame to the scene."
  (make-scene-graph :frame-map (tree-map-insert (scene-graph-frame-map scene-graph)
                                                (scene-frame-name frame)
                                                frame)))
(defun scene-graph-remove-frame (scene-graph frame-name)
  "Remove a frame from the scene."
  (make-scene-graph :frame-map (tree-map-remove (scene-graph-frame-map scene-graph)
                                                frame-name )))

(defun scene-graph-add-visual (scene-graph frame-name visual)
  "Bind visual geometry to a frame in the scene."
  (let ((frame (copy-structure (scene-graph-lookup scene-graph frame-name))))
    (push visual (scene-frame-visual frame))
    (scene-graph-add-frame scene-graph frame)))

(defun scene-graph (frames)
  (fold #'scene-graph-add-frame
        (make-scene-graph)
        frames))

(defun scene-graph-merge (scene-graph-1 scene-graph-2)
  (make-scene-graph
   :frame-map
   (fold-tree-map (lambda (map name frame)
                    (assert (not (tree-map-find map name)) () "Duplicate frame in merged graphs.")
                    (tree-map-insert map name frame))
                  (scene-graph-frame-map scene-graph-1)
                  (scene-graph-frame-map scene-graph-2))))

(defmacro do-scene-graph-frames ((frame-variable scene-graph &optional result) &body body)
  (with-gensyms (key)
    `(progn (map-tree-map :inorder nil (lambda (,key ,frame-variable)
                                         (declare (ignore ,key))
                                         ,@body)
                          (scene-graph-frame-map ,scene-graph))
            ,result)))

(defmacro do-scene-graph-visuals (((frame-name-variable visual-variable)
                                   scene-graph &optional result)
                                  &body body)
  (with-gensyms (frame fun)
    `(do-scene-graph-frames (,frame ,scene-graph ,result)
       (flet ((,fun (,frame-name-variable ,visual-variable)
                ,@body))
         (declare (dynamic-extent (function ,fun)))
         (let ((,visual-variable (scene-frame-visual ,frame))
               (,frame-name-variable (scene-frame-name ,frame)))
           (etypecase ,visual-variable
             (list
              (dolist (,visual-variable ,visual-variable)
                (,fun ,frame-name-variable ,visual-variable)))
             (scene-visual
              (,fun ,frame-name-variable ,visual-variable))))))))

(defun fold-scene-graph-frames (function initial-value scene-graph)
  (fold-tree-map (lambda (accum key value)
                   (declare (ignore key))
                   (funcall function accum value))
                 initial-value
                 (scene-graph-frame-map scene-graph)))


(defun scene-graph-lookup (scene-graph frame-name)
  (tree-map-find (scene-graph-frame-map scene-graph)
                 frame-name))


;; Find the relative transform of the scene frame
(defgeneric scene-frame-tf (object))


(defun scene-mesh-inc (mesh-file)
  "Return the include file for the mesh file"
  (clean-pathname (concatenate 'string
                               "povray/"
                               mesh-file ".inc")))

(defun scene-graph-resolve-mesh (scene-graph)
  (let ((mesh-files  ;; filename => (list mesh-nodes)
         (make-hash-table :test #'equal)))
    (labels ((resolve-mesh (mesh)
               (push mesh
                     (gethash (scene-mesh-file mesh) mesh-files))))
      ;; collect mesh files
      (do-scene-graph-visuals ((frame-name visual) scene-graph)
        (declare (ignore frame-name))
        (let ((g (scene-visual-geometry visual)))
          (when (and g (scene-mesh-p g))
            (resolve-mesh g)))))
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
             mesh-files)
    scene-graph))


(defun scene-frame-tf-relative (frame configuration-map default-configuration)
  "Find the relative TF for this frame"
  (etypecase frame
    (scene-frame-fixed (scene-frame-fixed-tf frame))
    (scene-frame-joint
     (assert configuration-map ()
             "Cannot find joint frames without configuration variables")
     (let* ((config (+ (scene-frame-joint-offset frame)
                       (tree-map-find configuration-map (scene-frame-name frame)
                                     default-configuration)))
            (axis (scene-frame-joint-axis frame)))
       (etypecase frame
         (scene-frame-revolute
          (tf* (with-vec3 (x y z) axis
                (axis-angle* x y z config))
               (scene-frame-revolute-translation frame)))
         (scene-frame-prismatic
          (tf* (scene-frame-prismatic-rotation frame)
               (g* config axis))))))))


(defun scene-graph-tf-relative (scene-graph frame-name
                                &key
                                  configuration-map
                                  (default-configuration 0d0))
  (scene-frame-tf-relative (scene-graph-lookup scene-graph frame-name)
                           configuration-map
                           default-configuration))

;; (defun scene-graph-tf-relative-map (scene-graph configuration-map
;;                                     &key (default-configuration 0d0))
;;   (fold-scene-graph-frames (lambda (hash frame)
;;                              (setf (gethash (scene-frame-name frame) hash)
;;                                    (scene-graph-tf-relative scene-graph (scene-frame-name frame)
;;                                                             :configuration-map configuration-map
;;                                                             :default-configuration default-configuration))
;;                              hash)
;;                            (make-hash-table :test #'equal)
;;                            scene-graph))

(defun scene-graph-tf-absolute (scene-graph frame-name
                                &key
                                  configuration-map
                                  (tf-absolute-map (make-string-hash-table))
                                  (default-configuration 0d0))
  (declare (type hash-table tf-absolute-map))
  (labels ((rec (frame-name)
             (cond
               ((null frame-name) ; global frame
                (tf nil))
               ((gethash frame-name tf-absolute-map) ; already in hash
                (gethash frame-name tf-absolute-map))
               (t
                (let* ((frame (scene-graph-lookup scene-graph frame-name))
                       (parent-name (scene-frame-parent frame))
                       (tf-frame (scene-frame-tf-relative frame
                                                          configuration-map
                                                          default-configuration)))
                  (if parent-name
                      (normalize (g* (rec (scene-frame-parent frame))
                                     tf-frame))
                      tf-frame))))))
    (rec frame-name)))

(defun scene-graph-tf-absolute-map (scene-graph configuration-map
                                    &key (default-configuration 0d0))
  (labels ((rec (tf-abs frame)
             (let ((name (scene-frame-name frame)))
               (setf (gethash name tf-abs)
                     (scene-graph-tf-absolute scene-graph name
                                              :configuration-map configuration-map
                                              :tf-absolute-map tf-abs
                                              :default-configuration default-configuration)))
             tf-abs))
    (fold-scene-graph-frames #'rec (make-hash-table :test #'equal) scene-graph)))


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

(defun scene-visual-pov (visual tf)
  (let ((geometry (scene-visual-geometry visual))
        (color (scene-visual-color visual))
        (alpha (scene-visual-alpha visual))
        (modifiers (list tf)))
    (when (or color alpha)
      (push (pov-pigment (append (when color
                                   (list (pov-color color)))
                                 (when alpha
                                   (list (pov-alpha alpha)))))
            modifiers))
    (when (find :no-shadow (scene-visual-modifiers visual))
      (push (pov-value "no_shadow" )
            modifiers))
    (etypecase geometry
      (scene-mesh (pov-mesh2 :mesh (scene-mesh-name geometry)
                             :modifiers modifiers))
      (scene-box
       (pov-box-center (scene-box-dimension geometry)
                       :modifiers modifiers))
      (scene-cylinder
       (pov-cylinder-axis (vec 0 0 (scene-cylinder-height geometry))
                          (scene-cylinder-radius geometry)
                          modifiers))
      (scene-cone
       (pov-cone-axis (vec 0 0 (scene-cone-height geometry))
                      (scene-cone-start-radius geometry)
                      (scene-cone-end-radius geometry)
                      modifiers))
      (scene-sphere
       (pov-sphere (vec3* 0 0 0)
                   (scene-sphere-radius geometry)
                   modifiers)))))

(defun scene-graph-pov-frame (scene-graph
                              &key
                                configuration-map
                                output
                                directory
                                include
                                (default-configuration 0d0))
"Generate the POV-ray scene for the given scene-graph."
  (let ((pov-things)
        (tf-abs (scene-graph-tf-absolute-map scene-graph configuration-map
                                             :default-configuration default-configuration)))
    (labels ((thing (thing)
               (push thing pov-things))
             (include (file)
               (thing (pov-include file)))
             (visual (vis name)
               (thing (scene-visual-pov vis
                                        (pov-transform* (pov-matrix (gethash name tf-abs)))))
               (thing (pov-line-comment (format nil "FRAME: ~A" name)))))
      ;; push frame geometry
      (do-scene-graph-visuals ((name vis) scene-graph)
        (visual vis name))
      ;; push mesh include files
      (let ((mesh-set (make-tree-set #'string-compare)))
        (do-scene-graph-visuals ((frame-name visual) scene-graph)
          (declare (ignore frame-name))
          (let ((geom (scene-visual-geometry visual)))
            (when (scene-mesh-p geom)
              (tree-set-insertf mesh-set
                                (scene-mesh-inc (scene-mesh-file geom))))))
        (map-tree-set nil #'include mesh-set))
      (map nil #'include (reverse (ensure-list include)))
      ;; version
      (thing (pov-version 3.7)))

    ;; result
    (output (pov-sequence pov-things)
            output
            :directory directory)))
