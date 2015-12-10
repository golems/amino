(require :amino)

(in-package :robray)

(sb-posix:setenv "ROS_PACKAGE_PATH" "/opt/ros/indigo/share/" 1)

(setq *scene-graph* (load-scene-file "package://baxter_description/urdf/baxter.urdf"))

(win-set-scene-graph *scene-graph*)

(win-set-config `(("right_s0" . ,(* .2 pi))))

(defparameter *ssg* (scene-graph-chain *scene-graph* nil "right_endpoint"))

(defparameter *start*  nil)




;(win-set-config (

;; (win-set-config (sub-scene-graph-config-map *ssg*
;;                                             ;(vec 0.700658 -0.412207 0.188664 1.00646 -0.206635 0.99295 0.10569)
;;                                             (vec 0.700658 -0.412207 0.188664 1.00646 -0.206635 0.99295 0.10569)))





(defparameter *goal*
  `(("right_s0" . ,(* .05 pi))
    ("right_s1" . ,(* -.25 pi))
    ("right_e0" . ,(* 0 pi))
    ("right_e1" . ,(* .25 pi))
    ("right_w0" . ,(* 0 pi))
    ("right_w1" . ,(* .25 pi))
    ("right_w2" . ,(* 0 pi))))

(defparameter *ws-goal*
  (g* (quaternion-translation-2 (quaternion* 0 1 0 0)
                                (vec3* .7 0 .5))
      (quaternion-translation-2 (x-angle (* -.5 pi))
                                nil)))

(win-set-config (scene-graph-ik *scene-graph* :frame "right_endpoint" :tf *ws-goal*))

(defparameter *mp*
  (motion-plan *ssg* *start*
               ;;:jointspace-goal *goal*
               :workspace-goal *ws-goal*))

(win-view-plan *mp*)
