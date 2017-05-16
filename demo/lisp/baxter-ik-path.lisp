(require :amino)

(in-package :robray)

(sb-posix:setenv "ROS_PACKAGE_PATH" "/home/ntd/ros_ws/src/baxter_common/" 1)

(setq *scene-graph* (load-scene-file "package://baxter_description/urdf/baxter.urdf"))

(defparameter *sub-scene-graph* (scene-graph-chain *scene-graph* nil "right_w2"))

(win-set-scene-graph *scene-graph*)

(win-run)


;(defparameter *e0* (tf (vec 0.000000	1.000000	0.000000	0.000000	0.800000	-0.250000	0.305100)))

(defparameter *e1* (tf* (quaternion* 0.000000 0.707107 0.707107 -0.000000)
                        (vec3* 0.600000 0.000000 0.500000)))

(defparameter *q0* (scene-graph-center-map *scene-graph*))

(defparameter *q0*
  (alist-configuration-map
   '(("right_s0" . 0.664700d0) ("right_s1" . -0.666419d0)
     ("right_e0" . 0.249159d0) ("right_e1" . 1.079791d0)
     ("right_w0" . -0.211153d0) ("right_w1" . 1.180571d0) ("right_w2" . 0.116756d0))))


(win-set-scene-graph *scene-graph* :configuration-map *q0*)

(defparameter *ksol-opts*
  (ksol-opt :dt .1d0
            :gain-angle 5d0
            :gain-trans 5d0))

(defparameter *plan*
  (cartesian-path *sub-scene-graph* *q0* *e1*
;                  :ksol-opts *ksol-opts*
                  ))

(win-display-motion-plan *plan*)
