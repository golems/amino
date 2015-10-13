(require :amino)

(in-package :robray)

(win-set-scene-graph
 (scene-frame-fixed nil "front_table"
                    :geometry (scene-geometry-box (draw-options-default :color '(.5 .5 .5)
                                                                        :visual t
                                                                        :collision t)
                                                  (vec 1 1 1))))
