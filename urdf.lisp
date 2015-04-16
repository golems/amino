(in-package :robray)

(defvar *urdf-dom*)

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

(defun urdf-parse (&key (dom *urdf-dom*))
  (let* ((urdf-joints (urdf-extract-joints :dom dom))
         (link-parent-map ; link name -> link's parent joint
          (fold (lambda (map j)
                  (setf (gethash (urdf-joint-child j) map)
                        (urdf-joint-name j))
                  map)
                (make-hash-table :test #'equal)
                urdf-joints)))
    ;; Create Scene Frames
    (let* ((frames
            (urdf-create-frames urdf-joints link-parent-map))
           (frame-map (fold (lambda (map frame)
                              (setf (gethash (scene-frame-name frame) map) frame)
                              map)
                            (make-hash-table :test #'equal)
                            frames)))
        frames)))
