(in-package :robray)


(defparameter *ros-distribution* "indigo")

(defparameter *pr2-description*
  (format nil "/opt/ros/~A/share/pr2_description/" *ros-distribution*))

(defparameter *pr2-source-directory*
  (concatenate 'string
               (namestring (asdf:system-source-directory :robray))
               "/demo/pr2/"))

(defvar *scene-graph-kitchen*)

(pushnew (cons "pr2_description" *pr2-description*)
         *urdf-package-alist* :test #'equal)

(defvar *scene-graph-pr2*)

(defparameter *scene-graph*
  (scene-graph (load-scene-file "/tmp/pr2.urdf" :mesh-up-axis "Y" :mesh-forward-axis "-Z")
               (load-scene-file (output-file "kitchen.scene" *pr2-source-directory*)
                                :type :moveit)
               (item-frame-marker nil "origin_marker"
                                  :length 1
                                  :width .1)))

(progn
  (uiop/stream:copy-file (output-file "pr2.inc" *pr2-source-directory*)
                         (output-file "pr2.inc" *robray-tmp-directory*))
  (scene-graph-pov-frame  *scene-graph*
                          :configuration-map (alist-tree-map nil #'string-compare)
                          :default-configuration 0d0
                          :include "pr2.inc"
                          :render t
                          :options (render-options-default :use-collision nil
                                                           :options (render-options-full-hd))
                          :output "robray.pov"))
