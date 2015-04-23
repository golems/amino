(setq *net-alist*
      '(("localhost" :jobs 8 :threads 1 :nice 0) ;; 4 core (HT), 3.6GHz
        ("dione" :jobs 6 :threads 2 :nice 0)     ;; 12 core, 1.4GHz
        ;("zeus" :jobs 8 :threads 1 :nice 1)
        ))

(setq *urdf-dom* (urdf-load "/home/ntd/ros_ws/src/baxter_common/baxter_description/urdf/baxter.urdf"))

(setq *scene-graph* (urdf-parse))

(scene-graph-resolve-mesh *scene-graph*)

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
                                    :include "/tmp/demo.inc" )
            :quality 0.0
            :width 1920
            :height 1080
            :file "/tmp/robray.pov"
            :output "/tmp/robray.png")


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
