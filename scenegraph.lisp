(in-package :robray)

(defvar *scene-graph*)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; SCENE FRAMES STRUCTURES ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; GEOMETRY ;;;


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

;;; FRAMES ;;;

(defstruct scene-frame
  (name nil :type frame-name)
  (parent nil :type frame-name)
  visual
  collision
  tf)

(defstruct (scene-frame-fixed (:include scene-frame)))

(defun scene-frame-fixed (parent name &key tf)
  (make-scene-frame-fixed :name name
                          :parent parent
                          :tf tf))


(defstruct (scene-frame-joint (:include scene-frame))
  axis
  (offset 0d0 :type double-float))

(defstruct (scene-frame-revolute (:include scene-frame-joint)))

(defun scene-frame-revolute (parent name &key
                                           axis
                                           (offset 0d0)
                                           tf)
  (assert axis)
  (make-scene-frame-revolute :name name
                             :parent parent
                             :axis axis
                             :offset offset
                             :tf tf))

(defstruct (scene-frame-prismatic (:include scene-frame-joint)))

(defun scene-frame-prismatic (parent name &key
                                            axis
                                            (offset 0d0)
                                            tf)
  (assert axis)
  (make-scene-frame-prismatic :name name
                              :parent parent
                              :axis axis
                              :offset offset
                              :tf tf))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; SCENE GRAPH STRUCTURE ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct scene-graph
  (frame-map (make-tree-map #'frame-name-compare)))

;;; Basic Operations ;;;

(defun scene-graph-lookup (scene-graph frame-name)
  "Find a frame"
  (tree-map-find (scene-graph-frame-map scene-graph)
                 frame-name))

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
    (assert (null (scene-frame-visual frame)))
    (setf (scene-frame-visual frame)
          visual)
    (scene-graph-add-frame scene-graph frame)))

(defun scene-graph-add-collision (scene-graph frame-name geometry)
  "Bind collision geometry to a frame in the scene."
  (let ((frame (copy-structure (scene-graph-lookup scene-graph frame-name))))
    (assert (null (scene-frame-collision frame)))
    (setf (scene-frame-collision frame)
          geometry)
    (scene-graph-add-frame scene-graph frame)))

(defun scene-graph (frames)
  (fold #'scene-graph-add-frame
        (make-scene-graph)
        frames))

(defun scene-graph-merge (scene-graph-1 scene-graph-2)
  "Combine two scene graphs."
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
             (null)
             (scene-visual
              (,fun ,frame-name-variable ,visual-variable))))))))


(defun fold-scene-graph-frames (function initial-value scene-graph)
  (fold-tree-map (lambda (accum key value)
                   (declare (ignore key))
                   (funcall function accum value))
                 initial-value
                 (scene-graph-frame-map scene-graph)))

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
                     (gethash (scene-mesh-file mesh) mesh-files)))
             (test-geom (g)
               (when (and g (scene-mesh-p g))
                 (resolve-mesh g))))
      ;; collect mesh files
      (do-scene-graph-frames (frame scene-graph)
        (when-let ((v (scene-frame-visual frame)))
          (test-geom (scene-visual-geometry v)))
        (test-geom (scene-frame-collision frame))))
    (maphash (lambda (mesh-file mesh-nodes)
               (format *standard-output* "~&Converting ~A..." mesh-file)
               (let* ((dom (dom-load mesh-file))
                      (inc-file (scene-mesh-inc mesh-file))
                      (geom-name (collada-geometry-name dom)))
                 (format *standard-output* "~&    to  ~A..." inc-file)
                 (ensure-directories-exist inc-file)
                 (collada-povray :dom dom
                                 :file inc-file)
                 (dolist (mesh-node mesh-nodes)
                   (setf (scene-mesh-name mesh-node) geom-name))))
             mesh-files)
    scene-graph))


(defun scene-frame-tf-local (frame configuration-map default-configuration)
  "Find the local TF from its parent to FRAME"
  (etypecase frame
    (scene-frame-fixed (scene-frame-fixed-tf frame))
    (scene-frame-joint
     (assert configuration-map ()
             "Cannot find joint frames without configuration variables")
     (let* ((config (+ (scene-frame-joint-offset frame)
                       (tree-map-find configuration-map (scene-frame-name frame)
                                     default-configuration)))
            (axis (scene-frame-joint-axis frame)))
       (tf-mul (scene-frame-tf frame)
               (etypecase frame
                 (scene-frame-revolute
                  (tf* (with-vec3 (x y z) axis
                         (axis-angle* x y z config))
                       (identity-vec3)))
                 (scene-frame-prismatic
                  (tf* (identity-quaternion)
                       (g* config axis)))))))))


;; (defun scene-graph-tf-relative (scene-graph frame-name
;;                                 &key
;;                                   configuration-map
;;                                   (default-configuration 0d0))
;;   (scene-frame-tf-relative (scene-graph-lookup scene-graph frame-name)
;;                            configuration-map
;;                            default-configuration))

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
                (let ((frame (scene-graph-lookup scene-graph frame-name)))
                  (assert frame () "Frame ~A not found in scene graph"
                          frame-name)
                  (let* ((parent-name (scene-frame-parent frame))
                         (tf-frame (scene-frame-tf-local frame
                                                         configuration-map
                                                         default-configuration)))
                    (if parent-name
                        (normalize (g* (rec (scene-frame-parent frame))
                                       tf-frame))
                        tf-frame)))))))
    (rec frame-name)))


(defun scene-graph-tf-relative (scene-graph parent child
                                &key
                                  configuration-map
                                  (default-configuration 0d0)
                                  (tf-absolute-map (make-string-hash-table)))
  "Find the relative TF from PARENT to CHILD"
  (let ((g-tf-parent(scene-graph-tf-absolute scene-graph parent
                                             :configuration-map configuration-map
                                             :tf-absolute-map tf-absolute-map
                                             :default-configuration default-configuration))
        (g-tf-child (scene-graph-tf-absolute scene-graph child
                                             :configuration-map configuration-map
                                             :tf-absolute-map tf-absolute-map
                                             :default-configuration default-configuration)))
    (tf-mul (tf-inverse g-tf-parent)
            g-tf-child)))



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


(defun scene-graph-reparent (scene-graph new-parent frame-name
                             &key
                               tf
                               (tf-absolute-map (make-string-hash-table))
                               configuration-map
                               (default-configuration 0d0))
  "Change the parent of frame FRAME-NAME to NEW-PARENT"
  (declare (type frame-name new-parent frame-name))
  (let ((tf (if tf
                tf
                (scene-graph-tf-relative scene-graph new-parent frame-name
                                         :configuration-map configuration-map
                                         :default-configuration default-configuration
                                         :tf-absolute-map tf-absolute-map)))
        (old-frame (scene-graph-lookup scene-graph frame-name)))
    (check-type old-frame scene-frame-fixed)
    (let ((new-frame (copy-scene-frame-fixed old-frame)))
      (setf (scene-frame-parent new-frame) new-parent
            (scene-frame-fixed-tf new-frame) tf)
      (scene-graph-add-frame (scene-graph-remove-frame scene-graph frame-name)
                             new-frame))))


(defun scene-graph-add-tf (scene-graph tf-tag &key
                                                (actual-parent (tf-tag-parent tf-tag))
                                                (tf-absolute-map (make-string-hash-table))
                                                configuration-map
                                                (default-configuration 0d0))
  "Add a fixed frame to the scene."
  (let* ((parent (tf-tag-parent tf-tag))
         (tf-rel (tf-tag-tf tf-tag))
         (name (tf-tag-child tf-tag))
         (tf (if (eq parent actual-parent)
                 tf-rel
                 (tf-mul (scene-graph-tf-relative scene-graph actual-parent parent
                                                  :configuration-map configuration-map
                                                  :default-configuration default-configuration
                                                  :tf-absolute-map tf-absolute-map)
                         tf-rel))))
    (scene-graph-add-frame scene-graph (scene-frame-fixed actual-parent name
                                                          :tf tf))))



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

(defun scene-visual-pov (visual geometry tf)
  (let ((color (scene-visual-color visual))
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
       (pov-sphere (identity-vec3)
                   (scene-sphere-radius geometry)
                   modifiers)))))

(defun scene-graph-pov-frame (scene-graph
                              &key
                                options
                                configuration-map
                                output
                                directory
                                include
                                (default-configuration 0d0))
"Generate the POV-ray scene for the given scene-graph."
  (let ((pov-things)
        (tf-abs (scene-graph-tf-absolute-map scene-graph configuration-map
                                             :default-configuration default-configuration))
        (mesh-set (make-tree-set #'string-compare)))
    (labels ((thing (thing)
               (push thing pov-things))
             (include (file)
               (thing (pov-include file))))
      ;; push frame geometry
      (do-scene-graph-frames (frame scene-graph)
        (let* ((name (scene-frame-name frame))
               (visual (scene-frame-visual frame))
               (geometry (if (get-render-option options :use-collision)
                             (scene-frame-collision frame)
                             (when visual (scene-visual-geometry visual)))))
          ;(print name)
          ;(print visual)
          ;(print geometry)
          (when (and visual geometry)
            (thing (scene-visual-pov visual geometry
                                     (pov-transform* (pov-matrix (gethash name tf-abs)))))
            (thing (pov-line-comment (format nil "FRAME: ~A" name))))
          (when (scene-mesh-p geometry)
            (tree-set-insertf mesh-set
                              (scene-mesh-inc (scene-mesh-file geometry))))))
      (map-tree-set nil #'include mesh-set)
      (map nil #'include (reverse (ensure-list include)))
      ;; version
      (thing (pov-version 3.7)))

    ;; result
    (output (pov-sequence pov-things)
            output
            :directory directory)))
