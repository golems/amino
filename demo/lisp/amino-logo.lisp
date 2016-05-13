(require :amino-rx)

(in-package :robray)

(defparameter *source-directory*
  (concatenate 'string
               (namestring (asdf:system-source-directory :amino))
               "../demo/lisp/"))

(uiop/stream:copy-file (output-file "logo.inc" *source-directory*)
                       (output-file "logo.inc" *robray-tmp-directory*))



(defun anthro-arm (&key
                     (length-0 1d0)
                     (length-1 1d0)
                     (radius .05)
                     parent )
  (let* ((options (draw-options-default :color '(.5 .5 .5)
                                        :visual t
                                        :specular '(.5 .5 .5)))
         (finger-radius (* .33 radius))
         (finger-length (* 3 radius))
         (hand-radius (* 2 radius))
         (finger-base (- hand-radius (* 1.5 finger-radius))))
    (flet ((finger (angle name)
             (scene-graph
              (scene-frame-revolute "hand" (rope "f" name)
                                    :axis '(0 -1 0)
                                    :geometry (list (scene-geometry-cylinder options
                                                                   :height finger-length
                                                                   :radius finger-radius)
                                                    (scene-geometry-sphere options
                                                                           (* 1.5 finger-radius)))

                                    :tf (tf* (z-angle angle)
                                             (vec3* (* finger-base (cos angle))
                                                    (* finger-base (sin angle))
                                                    0)))

              (scene-frame-fixed (rope "f" name) (rope "end" name)
                                 :tf (tf* nil (vec3* 0 0 finger-length))
                                 :geometry (scene-geometry-sphere options
                                                               finger-radius))
              )))
    (scene-graph
     ;; roll-pitch-yaw
     (scene-frame-revolute parent "s0"
                           :axis '(1 0 0)
                           :geometry (scene-geometry-sphere options (* 1.5 radius)))
     (scene-frame-revolute "s0" "s1"
                           :axis '(0 1 0))
     (scene-frame-revolute "s1" "s2"
                           :axis '(1 0 0))

     (scene-frame-revolute "s2" "e"
                           :axis '(0 1 0)
                           :tf (tf* nil (vec3* length-0 0 0))
                           )

     (scene-frame-fixed "e" "e-joint-link"
                           :tf (tf* (x-angle (/ pi 2)) (vec3* 0 radius 0))
                           :geometry (scene-geometry-cylinder  options
                                                               :radius (* 1.5 radius)
                                                               :height (* 2 radius)))

     (scene-frame-revolute "e" "w0"
                           :axis '(1 0 0)
                           :tf (tf* nil (vec3* length-1 0 0)))
     (scene-frame-revolute "w0" "w1"
                           :axis '(0 1 0))
     (scene-frame-revolute "w1" "w2"
                           :axis '(0 0 1)
                           :geometry (scene-geometry-sphere options (* 1.5 radius)))

     (scene-frame-fixed "s2" "upper-link"
                        :geometry (scene-geometry-cylinder options
                                                           :height length-0
                                                           :radius radius)
                        :tf (tf* (y-angle (/ pi 2)) nil))
     (scene-frame-fixed "e" "lower-link"
                        :geometry (scene-geometry-cylinder options
                                                           :height length-1
                                                           :radius radius)
                        :tf (tf* (y-angle (/ pi 2)) nil))



     (scene-frame-fixed "w2" "hand"
                        :geometry (scene-geometry-cone options
                                                           :height (- radius)
                                                           :end-radius radius
                                                           :start-radius hand-radius)
                        :tf (tf* (y-angle (/ pi 2)) (vec3* (* 2 radius) 0 0)))

     (finger (* 1.5 pi) "0")
     (finger (* (+ 1.5 (/ 2 3)) pi) "1")
     (finger (* (- 1.5 (/ 2 3)) pi)  "2")
     )

     )))


(defun frame-bubble (parent &key (length .15) )
  (scene-graph
   (scene-frame-fixed parent (rope parent "frame-sphere")
                      :geometry (scene-geometry-sphere (draw-options-default :alpha .5
                                                                             :specular '(.25 .25 .25)
                                                                             :color '(.25 .25 1)
                                                                             :no-shadow t)
                                                       length))
   (item-frame-marker parent (rope parent "frame-axes")
                      :length length :width (* .15 length)
                      :options (draw-options-default :no-shadow t))))

(defparameter *scene-graph*
  (scene-graph (let ((delta .04))
                 (draw-e-paper nil "my-paper"
                               :x (* delta 5 2 3 ) :y (* delta 5 2 5)
                               :delta delta
                                        ;:grid-color '(1 0 0)
                                        ;:paper-color '(0 0 0)
                               :options (draw-options-default :visual t)))
               (anthro-arm )
               (frame-bubble "s0")
               (frame-bubble "e")
               (frame-bubble "w2")
               (frame-bubble "end0" :length .05)
               (frame-bubble "end1" :length .05)
               (frame-bubble "end2" :length .05)
               ;; (scene-frame-fixed nil "sphere"
               ;;                    :geometry (scene-geometry-sphere (draw-options-default :alpha .5
               ;;                                                                           :specular '(.25 .25 .25)
               ;;                                                                           :color '(.25 .25 1)
               ;;                                                                           :no-shadow t)
               ;;                                                     *length*))
               ;; (item-frame-marker nil "axes"
               ;;                    :length *length* :width .05
               ;;                    :options (draw-options-default :no-shadow t))

               ))


(win-set-scene-graph *scene-graph*)
(win-set-config (alist-configuration-map `(("s0" . -.75)
                                           ("s1" . -2.5)
                                           ("s2" . -1.5)
                                           ("e" . 2)

                                           ("w0" . .3)
                                           ("w1" . -.3)
                                           ("w2" . 0)

                                           ("f0" . .25)
                                           ("f1" . .25)
                                           ("f2" . .25)
                                           )))



(render-scene-graph  *scene-graph*
                                        ;;:include "baxter.inc"
                     :camera-tf (win-tf-camera)
                     :include "logo.inc"
                     :configuration-map (win-config-map)
                     :render t
                     :options (render-options-default :use-collision nil
                                                      :options (render-options-full-hd))
                     :output "robray.pov")
