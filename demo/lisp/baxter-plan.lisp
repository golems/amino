(require :amino)

(in-package :robray)

(sb-posix:setenv "ROS_PACKAGE_PATH" "/opt/ros/indigo/share/" 1)

(setq *scene-graph* (load-scene-file "package://baxter_description/urdf/baxter.urdf"))

(win-set-scene-graph *scene-graph*)

(win-set-config `(("left_s0" . ,(/ pi 4))))

(defparameter *ssg* (scene-graph-chain *scene-graph* nil "left_w2"))

(defparameter *start*  nil)

(defparameter *goal*
  `(("left_s0" . ,(* .05 pi))
    ("left_s1" . ,(* -.25 pi))
    ("left_e0" . ,(* 0 pi))
    ("left_e1" . ,(* .25 pi))
    ("left_w0" . ,(* 0 pi))
    ("left_w1" . ,(* .25 pi))
    ("left_w2" . ,(* 0 pi))))


(motion-plan *ssg* *start* *goal*)
