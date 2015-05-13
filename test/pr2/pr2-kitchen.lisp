(in-package :robray)

(defparameter *pr2-source-directory*
  (concatenate 'string
               (namestring (asdf:system-source-directory :robray))
               "/test/pr2/"))

(defvar *scene-graph-kitchen*)

(setq *scene-graph-kitchen*
      (load-moveit-scene (output-file "kitchen.scene" *pr2-source-directory*)))

(setq *scene-graph* *scene-graph-kitchen*)

(setq *scene-graph*
      (draw-items *scene-graph* nil
                  (item-frame-marker "origin_marker"
                                     :length 1
                                     :width .1)))


(progn
  (uiop/stream:copy-file (output-file "pr2.inc" *pr2-source-directory*)
                         (output-file "pr2.inc" *robray-tmp-directory*))
  (scene-graph-pov-frame  *scene-graph*
                          ;; :configuration-map
                          ;; (alist-tree-map `(("right_s0" . ,(* .25 pi))
                          ;;                   ("right_s1" . ,(* -0.25 pi))
                          ;;                   ("right_e0" . ,(* 0.0 pi))
                          ;;                   ("right_e1" . ,(* 0.25 pi))
                          ;;                   ("right_w0" . ,(* 0.0 pi))
                          ;;                   ("right_w1" . ,(* 0.5 pi))
                          ;;                   ("right_w2" . ,(* 0.0 pi)))
                          ;;                 #'string-compare)
                          :include "pr2.inc"
                          :render t
                          :options (render-options-default :use-collision nil
                                                           :options (render-options-full-hd))
                          :output "robray.pov"))
