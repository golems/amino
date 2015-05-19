(in-package :robray)

(defvar *urdf-dom*)
(defvar *urdf-package-alist*)

(defun urdf-resolve-file (filename &optional (package-alist *urdf-package-alist*))
  (let* ((package (ppcre:regex-replace "^package://([^/]*)/.*"
                                       filename
                                       "\\1")))
    (if package
        (let ((package-directory (cdr (assoc package package-alist :test #'string=))))
          (assert package-directory)
          (ppcre:regex-replace "^package://([^/]*)"
                               filename
                               package-directory))
        filename)))


(defstruct urdf-joint
  name
  rpy
  xyz
  type
  axis
  parent
  child)

(defun urdf-load (file)
  "Create the DOM for a Collada file."
  (setq *urdf-dom* (dom-load file)))

(defun urdf-extract-joints (&key (dom *urdf-dom*))
  (loop with robot-node = (dom-select-tag dom "robot" :singleton t)
     for joint-node in (dom-select-tag robot-node "joint" :direct t)
     for name = (dom:get-attribute joint-node "name")
     collect
       (labels ((type (text)
                  (string-case text
                    ("fixed" :fixed)
                    ("prismatic" :prismatic)
                    ("revolute" :revolute)
                    ("continuous" :revolute)
                    (otherwise (error "Unknown joint type: ~A" text))))
                (path (path)
                  (dom-select-path joint-node path
                                   :singleton t :undefined-error nil))
                (maybe-floats (text thing length)
                  (when (> (length text) 0)
                    (let ((seq (parse-float-sequence text)))
                      (assert (= length (length seq)) ()
                              "Invalid length of ~A in joint ~A" thing name)
                      seq))))
         (make-urdf-joint :name (dom:get-attribute joint-node "name")
                          :type (type (dom:get-attribute joint-node "type"))
                          :rpy (maybe-floats (path '("origin" "@rpy")) "origin rpy" 3)
                          :xyz (maybe-floats (path '("origin" "@xyz")) "origin xyz" 3)
                          :axis (maybe-floats (path '("axis" "@xyz"))
                                              "axis xyz" 3)
                          :parent (path '("parent" "@link"))
                          :child (path '("child" "@link"))))))

(defun urdf-create-frames (urdf-joints link-parent-map)
  (loop for j in urdf-joints
     for parent = (gethash (urdf-joint-parent j) link-parent-map)
     for type = (urdf-joint-type j)
     for name = (urdf-joint-name j)
     for rpy = (urdf-joint-rpy j)
     for xyz = (urdf-joint-xyz j)
     for axis = (urdf-joint-axis j)
     for tf = (tf* (amino:euler-rpy rpy) xyz)
     collect
       (ecase type
         (:fixed
          (scene-frame-fixed parent name :tf tf))
         (:revolute
          (scene-frame-revolute parent name
                                :axis axis
                                :tf tf))
         (:prismatic
          (scene-frame-prismatic parent name
                                 :axis axis
                                 :tf tf)))))

(defun urdf-bind-links (dom scene-graph link-parent-map &key
                                                          (default-color '(.5 .5 .5))
                                                          (default-alpha 1d0))
  (labels ((path (node path &optional default)
             (dom-select-path node path :singleton t :undefined-error nil :default default))
           (node-shape (node)
             (let* ((mesh-file (path node '("geometry" "mesh" "@filename")))
                    (box-size (path node '("geometry" "box" "@size")))
                    (sphere-radius (path node '("geometry" "sphere" "@radius")))
                    (cylinder-radius (path node '("geometry" "cylinder" "@radius")))
                    (cylinder-length (path node '("geometry" "cylinder" "@length"))))
               (assert (xor mesh-file box-size sphere-radius (and cylinder-length
                                                                  cylinder-radius)))
               (cond (mesh-file
                      (make-scene-mesh :source-file (urdf-resolve-file mesh-file)))
                     (sphere-radius
                      (make-scene-sphere :radius (parse-float sphere-radius)))
                     (box-size
                      (make-scene-box :dimension (parse-float-sequence box-size)))
                     ((and cylinder-length cylinder-radius)
                      (scene-cylinder :height (parse-float cylinder-length)
                                      :radius (parse-float cylinder-radius))))))
           (node-origin (link-name node suffix geometry)
             (let* ((frame-name (gethash link-name link-parent-map))
                    (rpy (parse-float-sequence (path node '("origin" "@rpy") "0 0 0")))
                    (xyz (parse-float-sequence (path node '("origin" "@xyz") "0 0 0")))
                    (offset (if (scene-cylinder-p geometry) ; URDF cylinders have origin at their center
                                (tf* nil (vec3* 0d0 0d0 (/ (scene-cylinder-height geometry) -2)))
                                (tf* nil nil))))
               (let ((new-frame (scene-frame-fixed frame-name
                                                   (concatenate 'string frame-name "-" suffix)
                                                   :tf (tf-mul (tf* (euler-rpy rpy)
                                                                    (vec3 xyz))
                                                               offset))))
                 (setq scene-graph (scene-graph-add-frame scene-graph new-frame))
                 (scene-frame-name new-frame))))
           (node-options (&key rgba visual collision)
             `(,@(when rgba
                       `((:color . ,(subseq rgba 0 3))
                         (:alpha . ,(elt rgba 3))))
                 (:visual . ,visual)
                 (:collision . ,collision))))

    (loop for link-node in (dom-select-path dom '("robot" "link"))
       for name = (dom:get-attribute link-node "name")
       for visual-node = (path link-node '("visual"))
       for collision-node = (path link-node '("collision"))
       for rgba-text = (when visual-node (path visual-node '("material" "color" "@rgba")))
       for rgba = (if rgba-text (parse-float-sequence rgba-text)
                      (append default-color (list default-alpha)))

       do
         (when visual-node
           (let* ((shape (node-shape visual-node))
                  (frame-name (node-origin name visual-node "visual" shape))
                  (options (node-options :rgba rgba :visual t)))
             ;; bind geometry
             (setq scene-graph
                   (scene-graph-add-geometry scene-graph frame-name
                                             (scene-geometry shape options)))))
         (when collision-node
           (let* ((shape (node-shape collision-node))
                  (frame-name (node-origin name collision-node "collision" shape))
                  (options (node-options :rgba rgba :collision t)))
             (setq scene-graph
                   (scene-graph-add-geometry scene-graph frame-name
                                             (scene-geometry shape options)))))))
  scene-graph)


(defun urdf-parse (urdf
                   &key
                     reload-meshes
                     (mesh-up-axis "Z")
                     (mesh-forward-axis "Y"))

  (let* ((dom (if (stringp urdf)
                  (urdf-load urdf)
                  urdf))
         (urdf-joints (urdf-extract-joints :dom dom))
         ;; link name => link's parent joint name
         (link-parent-map
          (fold (lambda (map j)
                  (setf (gethash (urdf-joint-child j) map)
                        (urdf-joint-name j))
                  map)
                (make-hash-table :test #'equal)
                urdf-joints))
         ;; Create Scene Frames
         (scene-graph (scene-graph (urdf-create-frames urdf-joints link-parent-map))))
    (scene-graph-resolve-mesh (urdf-bind-links dom scene-graph link-parent-map)
                              :mesh-up-axis mesh-up-axis
                              :mesh-forward-axis mesh-forward-axis
                              :reload reload-meshes)))
