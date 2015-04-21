(setq *urdf-dom* (urdf-load "/home/ntd/ros_ws/src/baxter_common/baxter_description/urdf/baxter.urdf"))

(setq *scene-graph* (urdf-parse))

(scene-graph-resolve-mesh *scene-graph*)

(pov-render (scene-graph-pov-frame  *scene-graph*
                                    (alist-tree-map `(("right_s0" . ,(* .25 pi))
                                                      ("right_s1" . ,(* -0.25 pi))
                                                      ("right_e0" . ,(* 0.0 pi))
                                                      ("right_e1" . ,(* 0.25 pi))
                                                      ("right_w0" . ,(* 0.0 pi))
                                                      ("right_w1" . ,(* 0.5 pi))
                                                      ("right_w2" . ,(* 0.0 pi)))
                                                    #'string-compare)
                                    :include "/tmp/demo.inc" )
            :quality 3
            :width 1920
            :height 1080
            :file "/tmp/robray.pov"
            :output "/tmp/robray.png")


(scene-graph-animate
 (keyframe-configuration-function (list
                                   (joint-keyframe 0d0 nil)
                                   (joint-keyframe 5d0 `(("right_s0" ,(* .25 pi))
                                                         ("right_s1" ,(* -0.25 pi))
                                                         ("right_e0" ,(* 0.0 pi))
                                                         ("right_e1" ,(* 0.25 pi))
                                                         ("right_w0" ,(* 0.0 pi))
                                                         ("right_w1" ,(* 0.5 pi))
                                                         ("right_w2" ,(* 0.0 pi))))))
 :frames-per-second 15
 :time-end 5d0
 :include "/tmp/demo.inc" )
