(require :amino-rx)

(in-package :robray)

(defparameter *source-directory*
  (concatenate 'string
               (namestring (asdf:system-source-directory :amino))
               "../demo/lisp/"))

(uiop/stream:copy-file (output-file "logo.inc" *source-directory*)
                       (output-file "logo.inc" *robray-tmp-directory*))

(defparameter *length* 1)

(defparameter *scene-graph*
  (scene-graph (draw-e-paper nil "my-paper"
                             :x 3 :y 3
                             :options (draw-options-default :visual t))
               (scene-frame-fixed nil "sphere"
                                  :geometry (scene-geometry-sphere (draw-options-default :alpha .5
                                                                                         :specular '(.25 .25 .25)
                                                                                         :color '(.25 .25 1)
                                                                                         :no-shadow t)
                                                                   *length*))
               (item-frame-marker nil "axes"
                                  :length *length* :width .05
                                  :options (draw-options-default :no-shadow t))))

(win-set-scene-graph *scene-graph*)

(render-scene-graph  *scene-graph*
                                        ;;:include "baxter.inc"
                     :camera-tf (win-tf-camera)
                     :include "logo.inc"
                     :render t
                     :options (render-options-default :use-collision nil
                                                      :options (render-options-medium))
                     :output "robray.pov")
