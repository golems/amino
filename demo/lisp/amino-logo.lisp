(require :amino-rx)

(in-package :robray)

(defparameter *source-directory*
  (concatenate 'string
               (namestring (asdf:system-source-directory :amino))
               "../demo/lisp/"))



(defun anthro-arm (&key
                     (length-0 1d0)
                     (length-1 1d0)
                     (radius .05)
                     parent )
  (let* ((options (draw-options-default :color '(.5 .5 .5)
                                        :visual t
                                        :specular '(3 3 3)))
         (joint-options (merge-draw-options (draw-options :color '(0 0 0) :specular '(.5 .5 .5)) options))
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
                                                    (scene-geometry-sphere joint-options
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
                           :geometry (scene-geometry-sphere joint-options (* 1.5 radius)))
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
                           :geometry (scene-geometry-cylinder  joint-options
                                                               :radius (* 1.25 radius)
                                                               :height (* 2 radius)))

     (scene-frame-revolute "e" "w0"
                           :axis '(1 0 0)
                           :tf (tf* nil (vec3* length-1 0 0)))
     (scene-frame-revolute "w0" "w1"
                           :axis '(0 1 0))
     (scene-frame-revolute "w1" "w2"
                           :axis '(0 0 1)
                           :geometry (scene-geometry-sphere joint-options (* 1.5 radius)))

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

     ;; (scene-frame-fixed nil "label"
     ;;                    :tf (tf* (x-angle (/ pi -2)) (vec3* -.565 -.6 .01))
     ;;                    :geometry (scene-geometry-text (draw-options-default :scale .45 :color '(.1 .1 .1)
     ;;                                                                         :no-shadow t)
     ;;                                                   "AMINO" :thickness .5))


     (finger (* 1.5 pi) "0")
     (finger (* (+ 1.5 (/ 2 3)) pi) "1")
     (finger (* (- 1.5 (/ 2 3)) pi)  "2")
     )

     )))


(defun frame-bubble (parent &key (length .15) )
  (scene-graph
   (scene-frame-fixed parent (rope parent "frame-sphere")
                      :geometry (scene-geometry-sphere (draw-options-default :alpha .33
                                                                             :specular '(.25 .25 .25)
                                                                             :color '(.25 .25 1)
                                                                             :no-shadow t)
                                                       length))
   (item-frame-marker parent (rope parent "frame-axes")
                      :length length :width (* .15 length)
                      :options (draw-options-default :no-shadow t))))

(defparameter *amino-logo*
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
               (frame-bubble "end2" :length .05)))



(progn
  (defparameter *dim* .15)
  (defparameter *tmp-logo*
    (scene-graph
     (anthro-arm )
     (scene-frame-fixed nil "table"
                        :tf (tf* nil '(-.5 -.5 0))
                        :geometry (scene-geometry-box (draw-options-default :color '(.5 .5 .5)
                                                                            :specular '(.2 .2 .2))
                                                      '(1 1 .01)))
     (labels ((box (color alpha)
                (scene-geometry-box (draw-options-default :color color
                                                          :visual t
                                                          :alpha alpha
                                                          :collision t)
                                    (vec *dim* *dim* *dim*)))
              (text (text alpha)
                (scene-geometry-text (draw-options-default :scale .1 :no-shadow t :alpha (max alpha .75))
                                     text :thickness .01))
              (obj-block (parent tag id tf color &optional (alpha 1d0))
                (scene-graph (scene-frame-fixed parent tag
                                                :tf tf
                                                :geometry (box color alpha))
                             (scene-frame-fixed tag (rope tag "-label-0")
                                                :tf (tf* nil (vec3* (- (* .15 *dim*))
                                                                    (- (/ *dim* -2) .001)
                                                                    (- (* .15 *dim*))))
                                                :geometry (text id alpha))
                             (scene-frame-fixed tag (rope tag "-label-1")
                                                :tf (tf* (x-angle (/ pi -2))
                                                         (vec3* (- (* .15 *dim*))
                                                                (- (* .15 *dim*))
                                                                (+ (/ *dim* 2) .001)))
                                                :geometry (text id alpha))
                             (scene-frame-fixed tag (rope tag "-label-2")
                                                :tf (tf* (z-angle (/ pi -2))
                                                         (vec3* (- (/ *dim* -2) .001)
                                                                (+ (* .15 *dim*))
                                                                (- (* .15 *dim*))))
                                                :geometry (text id alpha))


                             )))

       (scene-graph (obj-block "table" "block-a" "A"
                               (tf* nil (vec *dim* (* 1.5 *dim*) (/ *dim* 2)))
                               '(1 0 0))
                    (obj-block "block-a" "block-b" "B"
                                      (tf* nil (vec  (- (* 2 *dim*)) 0 0))
                                      '(0 1 0))
                    (obj-block "block-a" "block-c" "C"
                               (tf* nil (vec 0 0 *dim*))
                               '(0 0 1))
                    (scene-frame-fixed "block-b" "start"
                                       :tf (tf* (g* (quaternion (z-angle (/ pi -2)))
                                                    (quaternion (x-angle (/ pi -2))))
                                                (vec3* -.2 .1 0))
                                       :geometry (scene-geometry-text (draw-options-default :scale .1
                                                                                            :no-shadow t)
                                                                      "start"
                                                                      :thickness .01))

                    (scene-frame-fixed "start" "goal"
                                       :tf (tf* nil
                                                (vec3* .5 0 0))
                                       :geometry (scene-geometry-text (draw-options-default :scale .1
                                                                                            :no-shadow t)
                                                                      "goal"
                                                                      :thickness .01))

                    (obj-block "table" "goal-c" "C"
                               (tf* nil (vec 0 (* -1.5 *dim*) (/ *dim* 2)))
                               '(0 0 1)
                               .25d0)
                    (obj-block "goal-c" "goal-b" "B"
                               (tf* nil (vec 0 0 *dim*))
                               '(0 1 0)
                               .25d0)

                    (obj-block "goal-b" "goal-a" "A"
                               (tf* nil (vec 0 0 *dim*))
                               '(1 0 0)
                               .25d0)


                    (scene-frame-fixed "table" "robo"
                                       :tf (tf* (g* (quaternion (z-angle (/ pi -2)))
                                                    (g*
                                                     (quaternion (x-angle (/ pi -2)))
                                                     (quaternion (x-angle (/ pi 4)))))
                                                (vec3* -.6 .15 0))
                                       :geometry (scene-geometry-text (draw-options-default :scale .3
                                                                                            :no-shadow t)
                                                                      "Robo"
                                                                      :thickness .1))
                    (scene-frame-fixed "table" "synth"
                                       :tf (tf* (g* (quaternion (z-angle (* pi 0)))
                                                    (g*
                                                     (quaternion (x-angle (/ pi -2)))
                                                     (quaternion (x-angle (/ pi 4)))))
                                                (vec3* -.45 -.6 0))
                                       :geometry (scene-geometry-text (draw-options-default :scale .3
                                                                                            :no-shadow t)
                                                                      "Synth"
                                                                      :thickness .1))
                                 ;; (obj-block "table" "B"
                                 ;;            (tf* nil (vec  (- *dim*) *dim* (/ *dim* 2)))
                                 ;;            '(0 1 0))
                                 ;; (obj-block "table" "C"
                                 ;;            (tf* nil (vec *dim* *dim* (* 1.5 *dim*)))
                                 ;;            '(0 0 1))
                                 ;; (obj-block "table" "B"
                                 ;;            (tf* nil (vec  (- *dim*) *dim* (/ *dim* 2)))
                                 ;;            '(0 1 0))
                                 ;; (obj-block "table" "C"
                                 ;;            (tf* nil (vec *dim* *dim* (* 1.5 *dim*)))
                                 ;;            '(0 0 1))
                    )

     )))

  (win-set-scene-graph *tmp-logo*))

;; (win-set-config (alist-configuration-map `(("s0" . -.5)
;;                                            ("s1" . -2.5)
;;                                            ("s2" . -1.0)
;;                                            ("e" . 2.25)

;;                                            ("w0" . .3)
;;                                            ("w1" . -.4)
;;                                            ("w2" . 0)

;;                                            ("f0" . .25)
;;                                            ("f1" . .25)
;;                                            ("f2" . .25)
;;                                            )))

;; (scene-graph-ik *tmp-logo*
;;                 :start (alist-configuration-map `(("s0" . ,(* -.3 pi))
;;                                            ("s1" . ,(* -.75 pi ))
;;                                            ("s2" . ,(* -.5 pi))
;;                                            ("e" . 2.25)

;;                                            ("w0" . 0)
;;                                            ("w1" . 0)
;;                                            ("w2" . 0)

;;                                            ("f0" . .25)
;;                                            ("f1" . .25)
;;                                            ("f2" . .25)
;;                                                   ))
;;                 :frame "w2"
;;                 :tf (tf*
;;                      (quaternion* 0.10066416499691341d0 -0.27216000645069777d0
;;                                   -0.4514820225588964d0 0.8437770085042366d0)
;;                      (VEC3* -0.2629209527967792d0 -0.24463240198299907d0
;;                             0.7840155068842608d0)))





(win-set-config (alist-configuration-map `(("s0" . ,(* -.3 pi))
                                           ("s1" . ,(* -.75 pi ))
                                           ("s2" . ,(* -.5 pi))
                                           ("e" . 2.25)

                                           ("w0" . 0)
                                           ("w1" . .4)
                                           ("w2" . -.95)

                                           ("f0" . .25)
                                           ("f1" . .25)
                                           ("f2" . .25)
                                           )))



(progn
  (uiop/stream:copy-file (output-file "logo.inc" *source-directory*)
                         (output-file "logo.inc" *robray-tmp-directory*))

  (render-scene-graph  (win-scene-graph)
                       ;;:include "baxter.inc"
                       :camera-tf (win-tf-camera)
                       :include "logo.inc"
                       :configuration-map (win-config-map)
                       :render t
                       :options (render-options-default :use-collision nil
                                                        :options (render-options-full-hd))
                       :output "robray.pov"))
