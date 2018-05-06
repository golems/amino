(in-package :robray)

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
    (when (draw-option options :no-shadow)
      (push '|no_shadow| modifiers))
    (let ((scale (alist-get-default options :scale nil)))
      (when scale (push (pov-item "scale" scale)
                        modifiers)))
    (etypecase shape
      (scene-text (pov-text (scene-text-value shape)
                            :font (scene-text-font shape)
                            :thickness (scene-text-thickness shape)
                            :modifiers modifiers))
      (scene-mesh (pov-mesh2 :mesh (scene-mesh-name shape)
                             :modifiers modifiers))
      (scene-grid (pov-mesh2 :mesh-data (grid-mesh shape)
                             :modifiers modifiers))
      (scene-box
       (pov-box-center (scene-box-dimension shape)
                       :modifiers modifiers))
      (scene-cylinder
       (pov-cylinder-axis (vec 0 0 (scene-cylinder-height shape))
                          (scene-cylinder-radius shape)
                          modifiers))
      (scene-torus
       (pov-torus (scene-torus-angle shape)
                  (scene-torus-major-radius shape)
                  (scene-torus-minor-radius shape)))
      (scene-cone
       (pov-cone-axis (vec 0 0 (scene-cone-height shape))
                      (scene-cone-start-radius shape)
                      (scene-cone-end-radius shape)
                      modifiers))
      (scene-sphere
       (pov-sphere (identity-vec3)
                   (scene-sphere-radius shape)
                   modifiers)))))


(defun grid-mesh (grid)
  (let ((dim (scene-grid-dimension grid))
        (delta (scene-grid-delta grid))
        (thickness (scene-grid-thickness grid))
        (verts (make-array 0 :fill-pointer t :adjustable t))
        (indices (make-array 0 :fill-pointer t :adjustable t)))
    (labels ((helper (x0 x-max y delta vec-fun)
               (loop
                  with i-begin = (length verts)
                  with y-1 = (+ y (/ thickness 1))
                  with y-0 = (- y-1)
                  for x-c = x0 then (+ x-c delta)
                  while (<= (abs x-c) x-max)
                  for i0 from i-begin by 4
                  for i1 from (+ i-begin 1) by 4
                  for i2 from (+ i-begin 2) by 4
                  for i3 from (+ i-begin 3) by 4
                  for x-0 = (- x-c thickness)
                  for x-1 = (+ x-c thickness)
                  do (progn
                       (vector-push-extend (funcall vec-fun x-0 y-0 0) verts)
                       (vector-push-extend (funcall vec-fun x-0 y-1 0) verts)
                       (vector-push-extend (funcall vec-fun x-1 y-0 0) verts)
                       (vector-push-extend (funcall vec-fun x-1 y-1 0) verts)
                       (vector-push-extend (vector i0 i1 i2) indices)
                       (vector-push-extend (vector i3 i1 i2) indices)))))
      (helper 0 (vec-x dim) (vec-y dim) (vec-x delta) #'vec3*)
      (helper (- (vec-x delta)) (vec-x dim) (vec-y dim) (- (vec-x delta)) #'vec3*)
      (helper 0 (vec-y dim) (vec-x dim) (vec-y delta) (lambda (y x z) (vec3* x y z)))
      (helper (- (vec-y delta)) (vec-y dim) (vec-x dim) (- (vec-y delta)) (lambda (y x z) (vec3* x y z)))
      (make-mesh-data :vertex-vectors (vec-flatten verts)
                      :vertex-indices (fnvec-flatten indices)))))


(defun render-scene-graph (scene-graph
                           &key
                             (camera-tf (tf nil))
                             render
                             (options (render-options-default))
                             configuration-map
                             output
                             (directory *robray-tmp-directory*)
                             (include-directory *robray-tmp-directory*)
                             include
                             include-text
                             (default-configuration 0d0))
"Generate the POV-ray scene for the given scene-graph."
  (let ((pov-things)
        (include (or include (merge-pathnames "default.inc"
                                              *robray-share-directory*)))
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
      ;; Camera
      (when camera-tf
        (thing (pov-camera camera-tf
                           :width (get-render-option options :width)
                           :height (get-render-option options :height))))
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
              (let ((pov-file (scene-mesh-povray-file shape)))
                (unless pov-file
                  (error "No POVray mesh file for frame ~A" name))
                (tree-set-insertf mesh-set pov-file))))))
      ;; Include mesh files
      (map-tree-set nil (lambda (mesh-file)
                          (include (merge-pathnames mesh-file include-directory)))
                    mesh-set)
      ;; Include text
      (when include-text (thing include-text))
      ;; Include other files
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
