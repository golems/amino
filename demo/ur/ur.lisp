(in-package :robray)

(defparameter *ur-source-directory*
  (format-pathname "~A/~A"
                   (namestring (asdf:system-source-directory :robray))
                   "demo/ur/"))
;;;;;;;;;;;;;;;;
;; Load robot ;;
;;;;;;;;;;;;;;;;
(defparameter *scene-graph*
  (scene-graph
   ;; robot
   (load-scene-file (urdf-resolve-file  "package://ur_description/urdf/ur10_robot.urdf")
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
