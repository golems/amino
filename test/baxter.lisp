(in-package :robray)

;; Define hosts for rendering
(setq *render-host-alist*
      '(("localhost"  ; 4 core (HT), 3.6GHz
         :jobs 8 :threads 1 :nice 0)
        ("dione"      ; 12 core, 1.4GHz
         :jobs 6 :threads 2 :nice 0)
        ("zeus"       ; 16 core, 2.4GHz
         :jobs 7 :threads 2 :nice 1 :povray "/home/ndantam/local/bin/povray")
        ))
;; Define search path for URDF
(setq *urdf-package-alist*
      `(("baxter_description" . ,(concatenate 'string (namestring (user-homedir-pathname))
                                              "ros_ws/src/baxter_common/baxter_description"))))

;; Load robot scene graph from URDF
(defvar *scene-graph-baxter*)

;(time
(setq *scene-graph-baxter*
      ;(urdf-parse "/home/ntd/ros_ws/src/baxter_common/baxter_description/urdf/baxter.urdf" :reload-meshes t)
      (urdf-parse "/home/ntd/baxter_gripper.urdf" :reload-meshes t)
      )

(setq *scene-graph*
      (scene-graph-merge *scene-graph-baxter*
                         (load-curly-scene "/home/ntd/git/robray/test/scene.robray")
                         ;(urdf-parse "/home/ntd/git/robray/test/scene.urdf")
                         ))


;; (setq *scene-graph*
;;       (scene-graph-

;; Produce a simple animation
;; (time (scene-graph-time-animate
;;  (keyframe-configuration-function (list
;;                                    (joint-keyframe 0d0 nil)
;;                                    (joint-keyframe 2d0 `(("right_s0" ,(* .25 pi))
;;                                                          ("right_s1" ,(* -0.25 pi))
;;                                                          ("right_e0" ,(* 0.0 pi))
;;                                                          ("right_e1" ,(* 0.25 pi))
;;                                                          ("right_w0" ,(* 0.0 pi))
;;                                                          ("right_w1" ,(* 0.5 pi))
;;                                                          ("right_w2" ,(* 0.0 pi))))))
;;  :frames-per-second 15
;;  :time-end 2d0
;;  :encode-video t
;;  :include "baxter.inc" ))





(pov-render (scene-graph-pov-frame  *scene-graph*
                                    :configuration-map
                                    (alist-tree-map `(("right_s0" . ,(* .25 pi))
                                                      ("right_s1" . ,(* -0.25 pi))
                                                      ("right_e0" . ,(* 0.0 pi))
                                                      ("right_e1" . ,(* 0.25 pi))
                                                      ("right_w0" . ,(* 0.0 pi))
                                                      ("right_w1" . ,(* 0.5 pi))
                                                      ("right_w2" . ,(* 0.0 pi)))
                                                    #'string-compare)
                                    :options (render-options-default :use-collision nil :options (render-options-medium))
                                    :include "/tmp/robray/baxter.inc" )
            :options (render-options-default :use-collision nil
                                             :options (render-options-medium))
            :file "robray.pov")
