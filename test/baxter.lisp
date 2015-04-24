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
(setq *scene-graph*
      (urdf-parse "/home/ntd/ros_ws/src/baxter_common/baxter_description/urdf/baxter.urdf"))


;; Produce a simple animation
(time (scene-graph-animate
 (keyframe-configuration-function (list
                                   (joint-keyframe 0d0 nil)
                                   (joint-keyframe 2d0 `(("right_s0" ,(* .25 pi))
                                                         ("right_s1" ,(* -0.25 pi))
                                                         ("right_e0" ,(* 0.0 pi))
                                                         ("right_e1" ,(* 0.25 pi))
                                                         ("right_w0" ,(* 0.0 pi))
                                                         ("right_w1" ,(* 0.5 pi))
                                                         ("right_w2" ,(* 0.0 pi))))))
 :frames-per-second 15
 :time-end 2d0
 :encode-video t
 :include "/tmp/demo.inc" ))





;; (pov-render (scene-graph-pov-frame  *scene-graph*
;;                                     :configuration-map
;;                                     (alist-tree-map `(("right_s0" . ,(* .25 pi))
;;                                                       ("right_s1" . ,(* -0.25 pi))
;;                                                       ("right_e0" . ,(* 0.0 pi))
;;                                                       ("right_e1" . ,(* 0.25 pi))
;;                                                       ("right_w0" . ,(* 0.0 pi))
;;                                                       ("right_w1" . ,(* 0.5 pi))
;;                                                       ("right_w2" . ,(* 0.0 pi)))
;;                                                     #'string-compare)
;;                                     :include "/tmp/demo.inc" )
;;             :quality 0.0
;;             :width 1920
;;             :height 1080
;;             :file "/tmp/robray.pov"
;;             :output "/tmp/robray.png")
