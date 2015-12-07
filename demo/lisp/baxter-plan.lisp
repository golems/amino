(require :amino)

(in-package :robray)

(sb-posix:setenv "ROS_PACKAGE_PATH" "/opt/ros/indigo/share/" 1)

(setq *scene-graph* (load-scene-file "package://baxter_description/urdf/baxter.urdf"))

(win-set-scene-graph *scene-graph*)

(win-set-config `(("right_s0" . ,(/ pi 5))))

(defparameter *ssg* (scene-graph-chain *scene-graph* nil "right_w2"))

(defparameter *start*  nil)

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

(defvar *mp*)

(setq  *mp*
       (motion-plan *ssg* *start*
                    ;:jointspace-goal *goal*
                    :workspace-goal *ws-goal*
                    ))

(win-view-plan *mp*)
