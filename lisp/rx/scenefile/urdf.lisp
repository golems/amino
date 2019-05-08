;;;; -*- mode: lisp -*-
;;;;
;;;; Copyright (c) 2015, Rice University
;;;; All rights reserved.
;;;;
;;;; Author(s): Neil T. Dantam <ntd@gatech.edu>
;;;;
;;;;   Redistribution and use in source and binary forms, with or
;;;;   without modification, are permitted provided that the following
;;;;   conditions are met:
;;;;
;;;;   * Redistributions of source code must retain the above
;;;;     copyright notice, this list of conditions and the following
;;;;     disclaimer.
;;;;   * Redistributions in binary form must reproduce the above
;;;;     copyright notice, this list of conditions and the following
;;;;     disclaimer in the documentation and/or other materials
;;;;     provided with the distribution.
;;;;   * Neither the name of copyright holder the names of its
;;;;     contributors may be used to endorse or promote products
;;;;     derived from this software without specific prior written
;;;;     permission.
;;;;
;;;;   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
;;;;   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
;;;;   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
;;;;   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
;;;;   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
;;;;   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
;;;;   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
;;;;   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
;;;;   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
;;;;   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
;;;;   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
;;;;   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;;;   POSSIBILITY OF SUCH DAMAGE.

(in-package :robray)

(defvar *urdf-dom*)
(defvar *urdf-package-alist* nil)

;; (defun urdf-package-add (name directory)
;;   (pushnew (cons name directory)
;;            *urdf-package-alist* :test #'equal))

(defun ros-package-path () (uiop/os:getenv "ROS_PACKAGE_PATH"))

(defun (setf ros-package-path) (value)
  (etypecase value
    (string (sb-posix:setenv "ROS_PACKAGE_PATH" value 1))
    (pathname (setf (ros-package-path) (namestring value)))
    (list
     (setf (ros-package-path)
           (format nil "~{~A~^:~}" (loop for part in value
                                      collect (etypecase part
                                                (string part)
                                                (pathname (namestring part)))))))))

(defun ros-find-package (package)
  (labels ((dir-name (dir)
             (car (last (pathname-directory dir))))
           (rec (dirs)
             (some #'visit dirs))
           (visit (dir)
             (cond
               ;; does not exits
               ((not (uiop/filesystem:directory-exists-p dir))
                nil)
               ;; is the package
               ((string= (dir-name dir) package)
                dir)
               ;; is a different package
               ((probe-file (merge-pathnames "package.xml" dir))
                nil)
               ;; Recurse
               (t (rec (uiop/filesystem:subdirectories dir))))))
    (rec (ppcre:split ":" (ros-package-path)))))


(defun urdf-resolve-file (filename) ; &optional (package-alist *urdf-package-alist*))
  (if (ppcre:scan "^package://" filename)
      (let ((suffix (ppcre:regex-replace "^package://+" filename "")))
        (ppcre:register-groups-bind (package-name sep file-part)
            ("^([^/]+)(/|$)(.*)" suffix)
          (declare (ignore sep))
          (if-let ((package (ros-find-package package-name)))
            (merge-pathnames file-part package)
            (error "'~A' not found for ROS_PACKAGE_PATH='~A'"
                   package-name (ros-package-path)))))
      ;; Not a package URL
      filename))


;; (defun urdf-resolve-file (filename) ; &optional (package-alist *urdf-package-alist*))
;;   (if (ppcre:scan "^package://" filename)
;;     (let ((ros-package-path (uiop/os:getenv "ROS_PACKAGE_PATH"))
;;           (suffix (pathname (ppcre:regex-replace "^package://+" filename ""))))
;;       (loop for path in (ppcre:split ":" ros-package-path)
;;          for package-directory = (concatenate 'string path "/")
;;          for candidate = (merge-pathnames suffix package-directory)
;;          do
;;            (when (probe-file candidate)
;;              (return-from urdf-resolve-file (namestring candidate))))
;;       (error "'~A' not found for ROS_PACKAGE_PATH='~A'" filename ros-package-path))
;;     ;; Not a package URL
;;     filename))


(defstruct urdf-joint
  name
  rpy
  xyz
  type
  axis
  parent
  child
  limit-effort
  limit-lower
  limit-upper
  limit-velocity)

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
                (maybe-float (text)
                  (when (> (length text) 0)
                    (parse-float text)))
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
                          :limit-effort (maybe-float (path '("limit" "@effort")))
                          :limit-lower (maybe-float (path '("limit" "@lower")))
                          :limit-upper (maybe-float (path '("limit" "@upper")))
                          :limit-velocity (maybe-float (path '("limit" "@velocity")))
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
     for limits = (joint-limits :effort-limit (urdf-joint-limit-effort j)
                                :velocity-limit (urdf-joint-limit-velocity j)
                                :min-position (urdf-joint-limit-lower j)
                                :max-position (urdf-joint-limit-upper j))
     collect
       (ecase type
         (:fixed
          (scene-frame-fixed parent name :tf tf))
         (:revolute
          (scene-frame-revolute parent name
                                :axis axis
                                :limits limits
                                :tf tf))
         (:prismatic
          (scene-frame-prismatic parent name
                                 :axis axis
                                :limits limits
                                 :tf tf)))))


(defun urdf-material-properties (node materials)
  (let* ((name (dom:get-attribute node "name"))
         (node (if (and name (not (zerop (length name))) materials)
                   (gethash name materials)
                   node))
         (result *draw-options*))
    (when-let ((rgba-text (dom-select-path node '("color" "@rgba")
                                           :singleton t :undefined-error nil)))
      (let ((elts (parse-float-sequence rgba-text)))
        (push `(:color . ,(subseq elts 0 3)) result)
        (push `(:alpha . ,(elt elts 3)) result)))
    ;;(format t "~A: ~A~%" name result)
    result))


(defun urdf-materials (dom)
  (let ((materials (make-hash-table :test #'equal)))
    (dolist (node (dom-select-path dom  '("robot" "material")
                                   :undefined-error nil))
      ;;(urdf-material-properties node nil)
      (let ((name  (dom-select-path node '("@name") :singleton t :undefined-error nil)))
        (when name
          (setf (gethash name materials)
                node))))
    ;; (loop for k being the hash-keys of materials
    ;;    do (format t "~&~A: ~A~%"
    ;;               k
    ;;               (urdf-material-properties (gethash k materials) nil)))
    materials))

(defun urdf-bind-links (dom scene-graph link-parent-map &key
                                                          (default-color '(.5 .5 .5))
                                                          (default-alpha 1d0))
  (labels ((path-1 (node path &optional default)
             (dom-select-path node path :singleton t :undefined-error nil :default default))
           (path-n (node path &optional default)
             (dom-select-path node path :singleton nil :undefined-error nil :default default))
           (node-geometry (name node options)
             (let* ((mesh-file (path-1 node '("geometry" "mesh" "@filename")))
                    (box-size (path-1 node '("geometry" "box" "@size")))
                    (sphere-radius (path-1 node '("geometry" "sphere" "@radius")))
                    (cylinder-radius (path-1 node '("geometry" "cylinder" "@radius")))
                    (cylinder-length (path-1 node '("geometry" "cylinder" "@length"))))
               (cond ((not (or mesh-file box-size sphere-radius cylinder-length cylinder-radius))
                      ;; no shape
                      nil)
                     ((not (xor mesh-file box-size sphere-radius (and cylinder-length
                                                                      cylinder-radius)))
                      ;; multiple shapes
                      (error "bad or multiple shapes in link `~A'" name))
                     (mesh-file
                      (let* ((scale-string (path-1 node '("geometry" "mesh" "@scale") "1 1 1"))
                             (scale (or (parse-float-sequence scale-string)
                                        (list 1d0 1d0 1d0))))
                        ;; TODO: scale x,y,z separatly
                        (scene-geometry-mesh (draw-options-default :options options
                                                                   :scale (car scale))
                                             (urdf-resolve-file mesh-file))))
                     (sphere-radius
                      (scene-geometry-sphere options (parse-float sphere-radius)))
                     (box-size
                      (scene-geometry-box options (parse-float-sequence box-size)))
                     ((and cylinder-length cylinder-radius)
                      (scene-geometry-cylinder options
                                               :height (parse-float cylinder-length)
                                               :radius (parse-float cylinder-radius))))))
           (node-origin (link-name node suffix geometry)
             (check-type geometry (or null scene-geometry))
             (let* ((frame-name (gethash link-name link-parent-map))
                    (rpy (parse-float-sequence (path-1 node '("origin" "@rpy") "0 0 0")))
                    (xyz (parse-float-sequence (path-1 node '("origin" "@xyz") "0 0 0")))
                    ;; URDF cylinders have origin at their center
                    (offset (if (scene-geometry-cylinder-p geometry)
                                (tf* nil (vec3* 0d0 0d0 (/ (scene-geometry-cylinder-height geometry) -2)))
                                (tf* nil nil)))
                    (tf (tf-mul (tf* (euler-rpy rpy)
                                     (vec3 xyz))
                                offset)))
               (if (and (equalp tf +tf-ident+)
                        frame-name)
                   ;; same as parent, reuse frame
                   frame-name
                   ;; offset, add new frame
                   (let ((new-frame (scene-frame-fixed frame-name
                                                       (concatenate 'string link-name "-" suffix)
                                                       :tf (tf-mul (tf* (euler-rpy rpy)
                                                                        (vec3 xyz))
                                                                   offset))))
                     (scene-graph-f scene-graph new-frame)
                     (scene-frame-name new-frame)))))
           (node-options (default &key visual collision)
             (merge-draw-options (draw-options :visual visual
                                               :collision collision)
                                 default)))

    (loop with materials = (urdf-materials dom)
       for link-node in (dom-select-path dom '("robot" "link"))
       for name = (dom:get-attribute link-node "name")
       for visuals = (path-n link-node '("visual"))
       for inertial-node = (path-1 link-node '("inertial"))
       for default-visual-node = (car visuals)
       for material-node = (when default-visual-node
                             (path-1 default-visual-node '("material")))
       for material-props = (when material-node
                              (urdf-material-properties material-node materials))
       do
       ;; Add visuals
         (loop for visual-node in visuals
            for rgba-text = (when visual-node (path-1 visual-node '("material" "color" "@rgba")))
            for rgba = (if rgba-text (parse-float-sequence rgba-text)
                           (append default-color (list default-alpha)))
            for options = (node-options material-props :visual t :collision nil)
            for geometry = (node-geometry name visual-node options)
            when geometry
            do (let ((frame-name (node-origin name visual-node "visual" geometry)))
                 (setq scene-graph
                       (scene-graph-add-geometry scene-graph frame-name geometry))))
       ;; Add collisions
         (loop for collision-node in (path-n link-node '("collision"))
            for options = (node-options material-props :visual nil :collision t)
            for geometry = (node-geometry name collision-node options)
            do
              (let ((frame-name (node-origin name collision-node "collision" geometry)))
                (setq scene-graph
                      (scene-graph-add-geometry scene-graph frame-name geometry))))
       ;; Add inertials
         (when inertial-node
           (let* ((frame-name (node-origin name inertial-node "inertial" nil))
                  (mass (parse-float (path-1 inertial-node '("mass" "@value"))))
                  (ixx (parse-float (path-1 inertial-node '("inertia" "@ixx"))))
                  (ixy (parse-float (path-1 inertial-node '("inertia" "@ixy"))))
                  (ixz (parse-float (path-1 inertial-node '("inertia" "@ixz"))))
                  (iyy (parse-float (path-1 inertial-node '("inertia" "@iyy"))))
                  (iyz (parse-float (path-1 inertial-node '("inertia" "@iyz"))))
                  (izz (parse-float (path-1 inertial-node '("inertia" "@izz"))))
                  (iyx (- ixy))
                  (izy (- iyz))
                  (izx (- ixz)))
             (setq scene-graph
                   (scene-graph-set-inertial scene-graph frame-name
                                             :mass mass
                                             ;; column major
                                             :inertia (vec ixx iyx izx
                                                           ixy iyy izy
                                                           ixz iyz izz)))))))
  scene-graph)


(defun urdf-parse (urdf
                   &key
                     reload-meshes
                     (mesh-up-axis "Z")
                     (mesh-forward-axis "Y"))

  (let* ((dom (if (or (stringp urdf)
                      (pathnamep urdf))
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
         (scene-graph (scene-graph (make-scene-graph :files (make-scene-graph-files urdf))
                                   (urdf-create-frames urdf-joints link-parent-map))))
    (urdf-bind-links dom scene-graph link-parent-map)))
