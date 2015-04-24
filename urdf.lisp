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
     collect
       (labels ((type (text)
                  (string-case text
                    ("fixed" :fixed)
                    ("revolute" :revolute)
                    (otherwise (error "Unknown joint type: ~A" text))))
                (path (path)
                  (dom-select-path joint-node path
                                   :singleton t :undefined-error nil))
                (maybe-floats (text)
                  (when text
                    (parse-float-sequence text))))
         (make-urdf-joint :name (dom:get-attribute joint-node "name")
                          :type (type (dom:get-attribute joint-node "type"))
                          :rpy (maybe-floats (path '("origin" "@rpy")))
                          :xyz (maybe-floats (path '("origin" "@xyz")))
                          :axis (maybe-floats (path '("axis" "@xyz")))
                          :parent (path '("parent" "@link"))
                          :child (path '("child" "@link"))))))

(defun urdf-create-frames (urdf-joints link-parent-map)
  (labels ((origin-rotated-p (rpy)
             (not (= 0d0 (vecref rpy 0) (vecref rpy 1) (vecref rpy 2))))
           (origin-frame (origin parent rpy xyz)
             (make-scene-frame-fixed :name origin
                                     :parent parent
                                     :tf (tf (euler-rpy rpy) xyz))))
    (loop for j in urdf-joints
       for parent = (gethash (urdf-joint-parent j) link-parent-map)
       for type = (urdf-joint-type j)
       for name = (urdf-joint-name j)
       for rpy = (urdf-joint-rpy j)
       for xyz = (urdf-joint-xyz j)
       for axis = (urdf-joint-axis j)
       for origin = (concatenate 'string name "_origin")
       nconc
         (ecase type
           (:fixed
            (list
             (make-scene-frame-fixed :name name
                                     :parent parent
                                     :tf (amino:tf (amino:euler-rpy rpy) xyz))))
           (:revolute
            (if (origin-rotated-p rpy)
                (list (origin-frame origin parent rpy xyz)
                      (make-scene-frame-revolute :name name
                                                 :parent origin
                                                 :axis axis
                                                 :translation (vec3 nil)))
                (list (make-scene-frame-revolute :name name
                                                 :parent parent
                                                 :axis axis
                                                 :translation (vec3 xyz)))))
           (:prismatic
            (if (origin-rotated-p rpy)
                (list (origin-frame origin parent rpy xyz)
                      (make-scene-frame-prismatic :name name
                                                  :parent origin
                                                  :axis axis
                                                  :rotation (quaternion nil)))
                (list (make-scene-frame-prismatic :name name
                                                  :parent parent
                                                  :axis axis
                                                  :rotation (quaternion rpy)))))))))

(defun urdf-bind-links (dom scene-graph link-parent-map)
  (labels ((path (node path &optional default)
             (dom-select-path node path :singleton t :undefined-error nil :default default)))
    (loop for link-node in (dom-select-path dom '("robot" "link"))
       for name = (dom:get-attribute link-node "name")
       for frame-name = (gethash name link-parent-map)
       for frame = (scene-graph-lookup scene-graph frame-name)
       for visual-node = (path link-node '("visual"))
       do
         (when visual-node
           (let* ((rpy (parse-float-sequence (path visual-node '("origin" "@rpy") "0 0 0")))
                  (xyz (parse-float-sequence (path visual-node '("origin" "@xyz") "0 0 0")))
                  (mesh-file (path visual-node '("geometry" "mesh" "@filename")))
                  (box-size (path visual-node '("geometry" "box" "@size")))
                  (sphere-radius (path visual-node '("geometry" "sphere" "@radius")))
                  (rgba-text (path visual-node '("material" "color" "@rgba")))
                  (rgba (when rgba-text (parse-float-sequence rgba-text))))
             ;; maybe add intermediate frame
             (unless (and (= 0
                             (elt rpy 0) (elt rpy 1) (elt rpy 2)
                             (elt xyz 0) (elt xyz 1) (elt xyz 2)))
               (let ((new-frame (make-scene-frame-fixed :name (concatenate 'string frame-name "-visual")
                                                        :parent frame-name
                                                        :tf (tf (euler-rpy rpy)
                                                                (vec3 xyz)))))
                 (scene-graph-add-frame scene-graph new-frame)
                 (setq frame new-frame
                       frame-name (scene-frame-name new-frame))))
             ;; bind geometry
             (labels ((push-visual (geometry)
                        (push (make-scene-visual :color (when rgba (subseq rgba 0 3))
                                                 :alpha (when rgba (elt rgba 3))
                                                 :geometry geometry)
                              (scene-frame-visual frame))))
               (when mesh-file
                 (push-visual (make-scene-mesh :file (urdf-resolve-file mesh-file))))
               (when sphere-radius
                 (push-visual (make-scene-sphere :radius (parse-float sphere-radius))))
               (when box-size
                 (push-visual (make-scene-box :dimension (parse-float-sequence box-size)))))))))
  scene-graph)


(defun urdf-parse (urdf)
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
    (scene-graph-resolve-mesh (urdf-bind-links dom scene-graph link-parent-map))))
