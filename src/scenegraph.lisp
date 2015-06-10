(in-package :robray)

(defvar *scene-graph*)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; SCENE FRAMES STRUCTURES ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; GEOMETRY ;;;


(defstruct scene-mesh
  "A scene object defined by a mesh."
  name
  source-file
  povray-file)

(defstruct scene-box
  "A box object.
The center of the box is at the origin, and it is aligned with the frame axes."
  dimension)
(defun scene-box (dimension)
  (make-scene-box :dimension dimension))

(defstruct scene-sphere
  "A sphere object.
The center of the sphere is at the origin."
  radius)
(defun scene-sphere (radius)
  (make-scene-sphere :radius radius))


(defstruct (scene-cylinder (:constructor scene-cylinder))
  "A cylinder object.
The cylinder starts at the origin and extends by HEIGHT in the Z direction."
  height
  radius)

(defstruct (scene-cone (:constructor scene-cone))
  "A cone object.
The cone starts at the origin and extends by HEIGHT in the Z direction."
  height
  start-radius
  end-radius)


(defparameter *draw-options*
  '((:no-shadow . nil)
    (:color . (0 0 0))
    (:alpha . 1d0)
    (:visual . t)
    (:collision . t)))

(defun draw-option (options key)
  (alist-get-default options key *draw-options*))

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




(defstruct scene-geometry
  shape
  options
  type
  (collision t)
  (visual t))

(defun scene-geometry (shape options)
  (make-scene-geometry :shape shape
                       :options options
                       :collision (draw-option options :collision)
                       :visual (draw-option options :visual)))

(defun scene-geometry-isa (geometry type)
  (let ((tree (scene-geometry-type geometry)))
    (when tree (tree-set-member-p tree type))))

;;; FRAMES ;;;

(defstruct scene-frame
  "Base struct for frames in the scene."
  (name nil :type frame-name)
  (parent nil :type frame-name)
  tf
  geometry)

(defun scene-frame-geometry-isa (frame type)
  (find-if (lambda (g)
             (scene-geometry-isa g type))
           (scene-frame-geometry frame)))

(defun scene-frame-geometry-collision (frame)
  (loop for g in (scene-frame-geometry frame)
     when (scene-geometry-collision g)
     collect g))

(defstruct (scene-frame-fixed (:include scene-frame))
  "A frame with a fixed transformation")

(defun scene-frame-fixed (parent name &key
                                        geometry
                                        (tf (identity-tf)))
  "Create a new fixed frame."
  (make-scene-frame-fixed :name name
                          :parent parent
                          :tf tf
                          :geometry (etypecase geometry
                                      (list geometry)
                                      (scene-geometry (list geometry)))))


(defstruct (scene-frame-joint (:include scene-frame))
  "BAse struct for varying scene frames."
  axis
  (offset 0d0 :type double-float))

(defstruct (scene-frame-revolute (:include scene-frame-joint))
  "A frame representing a revolute (rotating) joint.")

(defun scene-frame-revolute (parent name &key
                                           axis
                                           (offset 0d0)
                                           (tf (identity-tf)))
  "Create a new revolute frame."
  (assert axis)
  (make-scene-frame-revolute :name name
                             :parent parent
                             :axis axis
                             :offset offset
                             :tf tf))

(defstruct (scene-frame-prismatic (:include scene-frame-joint))
  "A frame representing a prismatic (sliding) joint.")

(defun scene-frame-prismatic (parent name &key
                                            axis
                                            (offset 0d0)
                                            (tf (identity-tf)))
  "Create a new prismatic frame."
  (assert axis)
  (make-scene-frame-prismatic :name name
                              :parent parent
                              :axis axis
                              :offset offset
                              :tf tf))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; SCENE GRAPH STRUCTURE ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; TODO: dot output

(defun scene-frame-compare (frame-a frame-b)
  (labels ((name (frame)
             (etypecase frame
               (string frame)
               (scene-frame (scene-frame-name frame)))))
    (declare (dynamic-extent #'name))
    (frame-name-compare (name frame-a)
                        (name frame-b))))

(defstruct scene-graph
  (frames (make-tree-set #'scene-frame-compare)))

;;; Basic Operations ;;;

(defun scene-graph-lookup (scene-graph frame-name)
  "Find a frame"
  (tree-set-find (scene-graph-frames scene-graph)
                 frame-name))

(defun %scene-graph-add-frame (scene-graph frame)
  "Add a frame to the scene."
  (make-scene-graph :frames (tree-set-replace (scene-graph-frames scene-graph)
                                              frame)))

(defun scene-graph-remove-frame (scene-graph frame-name)
  "Remove a frame from the scene."
  (make-scene-graph :frames (tree-set-remove (scene-graph-frames scene-graph)
                                             frame-name)))

(defun scene-graph-add-geometry (scene-graph frame-name geometry)
  "Bind collision geometry to a frame in the scene."
  (let ((frame (copy-structure (scene-graph-lookup scene-graph frame-name))))
    (push geometry (scene-frame-geometry frame))
    (%scene-graph-add-frame scene-graph frame)))

(defun %scene-graph-merge (scene-graph-1 scene-graph-2)
  "Combine two scene graphs."
  (let* ((set-1 (scene-graph-frames scene-graph-1))
         (set-2 (scene-graph-frames scene-graph-2))
         (intersection (tree-set-intersection set-1 set-2)))
    (assert (zerop (tree-set-count intersection)) ()
            "Duplicate frames in merged trees: ~A"
            (map-tree-set 'list #'scene-frame-name intersection))
    (make-scene-graph :frames (tree-set-union set-1 set-2))))

(defun %scene-graph (things)
  (labels ((rec (scene-graph thing)
             (etypecase thing
               (scene-frame
                (%scene-graph-add-frame scene-graph thing))
               (scene-graph
                (%scene-graph-merge scene-graph thing))
               (list
                (%scene-graph-merge scene-graph
                                    (%scene-graph thing))))))
    (fold #'rec (make-scene-graph) things)))

(defun scene-graph (&rest things)
  (%scene-graph things))

(defmacro scene-graph-f (place &rest things)
  `(setq ,place
         (scene-graph ,place ,@things)))

(defmacro do-scene-graph-frames ((frame-variable scene-graph &optional result) &body body)
  `(do-tree-set (,frame-variable (scene-graph-frames ,scene-graph) ,result)
     ,@body))

(defmacro do-scene-graph-geometry (((frame geometry) scene-graph &optional result)
                                   &body body)
  (with-gensyms (fun)
    `(flet ((,fun (,frame ,geometry)
              ,@body))
       (declare (dynamic-extent #',fun))
       (do-scene-graph-frames (,frame ,scene-graph ,result)
         (loop
            for ,geometry in (scene-frame-geometry ,frame)
            do (,fun ,frame ,geometry))))))

(defmacro do-scene-graph-geometry-types (((frame geometry) (scene-graph type) &optional result)
                                         &body body)
  (with-gensyms (fun typevar)
    `(flet ((,fun (,frame ,geometry)
              ,@body))
       (declare (dynamic-extent #',fun))
       (let ((,typevar ,type))
         (do-scene-graph-geometry ((,frame ,geometry) ,scene-graph ,result)
           (when (scene-geometry-isa ,geometry ,typevar)
             (,fun ,frame ,geometry)))))))


(defun fold-scene-graph-frames (function initial-value scene-graph)
  (fold-tree-set function initial-value (scene-graph-frames scene-graph)))


(defun scene-graph-joints (scene-graph)
  (fold-scene-graph-frames (lambda (list frame)
                             (etypecase frame
                               (scene-frame-fixed list)
                               ((or scene-frame-prismatic scene-frame-revolute)
                                (cons (scene-frame-name frame) list))))
                           nil
                           scene-graph))

(defun scene-mesh-inc (mesh-file)
  "Return the include file for the mesh file"
  (pov-cache-file (rope mesh-file ".inc")))

(defun scene-graph-resolve-mesh (scene-graph &key
                                               reload
                                               (mesh-up-axis "Z")
                                               (mesh-forward-axis "Y")
                                               (directory *robray-tmp-directory*))
  ;(print 'resolve-mesh)
  (let ((mesh-files  ;; filename => (list mesh-nodes)
         (make-hash-table :test #'equal)))
    (labels ((resolve-mesh (mesh)
               (when-let ((source (and (not (scene-mesh-povray-file mesh))
                                       (scene-mesh-source-file  mesh))))
                 (push mesh (gethash source mesh-files))))
             (test-shape (shape)
               (when (and shape (scene-mesh-p shape))
                 (resolve-mesh shape))))
      ;; collect mesh files
      (do-scene-graph-geometry ((frame geometry) scene-graph)
        (declare (ignore frame))
        (test-shape (scene-geometry-shape geometry))))
    (maphash (lambda (mesh-file mesh-nodes)
               ;(format *standard-output* "~&Converting ~A..." mesh-file)
               (multiple-value-bind (geom-name inc-file)
                   (mesh-povray mesh-file :directory directory
                                :reload reload
                                :mesh-up-axis mesh-up-axis
                                :mesh-forward-axis mesh-forward-axis)

               ;; (let* ((inc-file (output-file (scene-mesh-inc mesh-file) directory))
               ;;        (convert (or reload
               ;;                     (not (probe-file inc-file))
               ;;                     (>= (file-write-date mesh-file)
               ;;                         (file-write-date inc-file))))
               ;;        (geom-name (progn (ensure-directories-exist inc-file)
               ;;                          (collada-povray mesh-file
               ;;                                          (when convert inc-file)))))
               ;;   (if convert
               ;;       (format *standard-output* "~&    to  ~A..." inc-file)
               ;;       (format *standard-output* "~&    cached"))

                 (dolist (mesh-node mesh-nodes)
                   (setf (scene-mesh-name mesh-node) geom-name
                         (scene-mesh-povray-file mesh-node) inc-file))))
             mesh-files)
    scene-graph))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Computing Transforms ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun alist-configuration-map (alist &optional default-map)
  (fold (lambda (map item)
          (tree-map-insert map (car item) (cdr item)))
        (or default-map (make-tree-map #'frame-name-compare))
        alist))

(defun pairlist-configuration-map (names values &optional default-map)
  (alist-configuration-map (pairlis names values) default-map))

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



(defun scene-graph-tf-absolute-map (scene-graph
                                    &key
                                      configuration-map
                                      (default-configuration 0d0))
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
      (%scene-graph-add-frame (scene-graph-remove-frame scene-graph frame-name)
                             new-frame))))


(defun scene-graph-add-tf (scene-graph tf-tag &key
                                                geometry
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
    (%scene-graph-add-frame scene-graph (scene-frame-fixed actual-parent name
                                                          :tf tf
                                                          :geometry geometry))))



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
  (declare (type scene-geometry geometry))
  (let* ((options (scene-geometry-options geometry))
         (modifiers (list tf))
         (shape (scene-geometry-shape geometry)))
    (when (and options
               ;; Don't clobber mesh texture
               ;; TODO: there's probably a better way to do this
               (not (scene-mesh-p shape)))
      (push (pov-alist-texture options)
            modifiers))
    (when (find :no-shadow options)
      (push '|no_shadow| modifiers))
    (let ((scale (alist-get-default options :scale nil)))
      (when scale (push (pov-item "scale" scale)
                        modifiers)))
    (etypecase shape
      (scene-mesh (pov-mesh2 :mesh (scene-mesh-name shape)
                             :modifiers modifiers))
      (scene-box
       (pov-box-center (scene-box-dimension shape)
                       :modifiers modifiers))
      (scene-cylinder
       (pov-cylinder-axis (vec 0 0 (scene-cylinder-height shape))
                          (scene-cylinder-radius shape)
                          modifiers))
      (scene-cone
       (pov-cone-axis (vec 0 0 (scene-cone-height shape))
                      (scene-cone-start-radius shape)
                      (scene-cone-end-radius shape)
                      modifiers))
      (scene-sphere
       (pov-sphere (identity-vec3)
                   (scene-sphere-radius shape)
                   modifiers)))))

(defun scene-graph-pov-frame (scene-graph
                              &key
                                render
                                options
                                configuration-map
                                output
                                (directory *robray-tmp-directory*)
                                include
                                (default-configuration 0d0))
"Generate the POV-ray scene for the given scene-graph."
  (let ((pov-things)
        (tf-abs (scene-graph-tf-absolute-map scene-graph
                                             :configuration-map configuration-map
                                             :default-configuration default-configuration))
        (mesh-set (make-tree-set #'string-compare))
        (use-collision (get-render-option options :use-collision)))
    ;(format t "~& col: ~A" use-collision)
    (labels ((thing (thing)
               (push thing pov-things))
             (include (file)
               (thing (pov-include file))))
      ;; push frame geometry
      (do-scene-graph-geometry ((frame geometry) scene-graph)
        ;(format t "~&converting frame: ~A" (scene-frame-name frame))
        (when (if use-collision
                  (scene-geometry-collision geometry)
                  (scene-geometry-visual geometry))
          (let ((shape (scene-geometry-shape geometry))
                (name (scene-frame-name frame)))
            (thing (scene-visual-pov geometry
                                     (pov-transform* (pov-matrix (gethash name tf-abs)))))
            (thing (pov-line-comment (format nil "FRAME: ~A" name)))
            (when (scene-mesh-p shape)
              (setf (tree-set-find mesh-set)
                    (scene-mesh-povray-file shape))))))
      (map-tree-set nil #'include mesh-set)
      (map nil #'include (reverse (ensure-list include)))
      ;; version
      (thing (pov-version "3.7")))

    ;; result
    (let ((result (pov-sequence pov-things)))
      (cond (render
             (pov-render result
                         :file output
                         :directory directory
                                :options options)
             nil)
            (output
             (output result
                     output
                     :directory directory)
             nil)
            (t result)))))
