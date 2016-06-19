(in-package :robray)

(defparameter *ur-source-directory*
  (format-pathname "~A/~A"
                   (namestring (asdf:system-source-directory :amino))
                   "../demo/urdf/ur/"))

;;;;;;;;;;;;;;;;
;; Load robot ;;
;;;;;;;;;;;;;;;;
(defparameter *scene-graph*
  (scene-graph
   ;; robot
   (load-scene-file (rope *ur-source-directory* "/ur10_robot.urdf"))))

;;;;;;;;;;;;
;; Window ;;
;;;;;;;;;;;;

(win-set-scene-graph *scene-graph*)

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
