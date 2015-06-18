(in-package :robray)

(defparameter *ros-distribution* "indigo")

(defparameter *ur-description*
  (format-pathname "/opt/ros/~A/share/ur_description/"
                   *ros-distribution*))

(defparameter *ur-source-directory*
  (format-pathname "~A/~A"
                   (namestring (asdf:system-source-directory :robray))
                   "demo/ur/"))

(urdf-package-add "ur_description" *ur-description*)

;;;;;;;;;;;;;;;;
;; Load robot ;;
;;;;;;;;;;;;;;;;
(defparameter *scene-graph*
  (scene-graph
   ;; robot
   (load-scene-file (format-pathname "~A/~A" *ur-description* "urdf/ur10_robot.urdf")
                    :reload-meshes nil)))


;;;;;;;;;;;;
;; RENDER ;;
;;;;;;;;;;;;
(progn
  (uiop/stream:copy-file (output-file "baxter.inc" *baxter-source-directory*)
                         (output-file "baxter.inc" *robray-tmp-directory*))

  (scene-graph-pov-frame  *scene-graph*
                          :configuration-map
                          (alist-tree-map nil #'string-compare)
                          :default-configuration 0d0
                          :include "baxter.inc"
                          :render t
                          :options (render-options-default :use-collision nil
                                                           :options (render-options-full-hd))
                          :output "robray.pov"))
