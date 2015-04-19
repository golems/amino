(setq *urdf-dom* (urdf-load "/home/ntd/ros_ws/src/baxter_common/baxter_description/urdf/baxter.urdf"))

(setq *scene-graph* (urdf-parse))

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
            :file "/tmp/robray.pov"
            :output "/tmp/robray.png")
