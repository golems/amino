(require :amino)

(in-package :robray)

(sb-posix:setenv "ROS_PACKAGE_PATH" "/opt/ros/indigo/share/" 1)




(setq *scene-graph* (load-scene-file "package://baxter_description/urdf/baxter.urdf"))

(defparameter *config-names*
  '("right_s0" "right_s1"
    "right_e0" "right_e1"
    "right_w0" "right_w1" "right_w2"))

;; Allow Collisions
(defparameter *allowed-collision*
  '(("right_w0_fixed" . "right_wrist-collision")))
(defun joint-config (values)
  (configuration-map-pairs *config-names* values))

(setq *scene-graph* (scene-graph-allow-collisions *scene-graph*
                                                  *allowed-collision*))
(setq *scene-graph*
      (scene-graph-allow-configuration *scene-graph*
                                       (joint-config '(0.375973 -1.44985 0.555649
                                                       2.54396 -0.133194 0.498291 0.260089))))

(setq *scene-graph* (scene-graph-allow-configuration *scene-graph* nil))





(win-set-scene-graph *scene-graph*)

(win-set-config `(("right_s0" . ,(* .2 pi))))

(defparameter *ssg* (scene-graph-chain *scene-graph* nil "right_endpoint"))

(defparameter *start*  nil)




;; some configs
(win-set-config (joint-config '(0.375973 -1.44985 0.555649 2.54396 -0.133194 0.498291 0.260089)))

(defparameter *goal*
  `(("right_s0" . ,(* .05 pi))
    ("right_s1" . ,(* -.25 pi))
    ("right_e0" . ,(* 0 pi))
    ("right_e1" . ,(* .25 pi))
    ("right_w0" . ,(* 0 pi))
    ("right_w1" . ,(* .25 pi))
    ("right_w2" . ,(* 0 pi))))

;; (defparameter *ws-goal*
;;   (g* (quaternion-translation-2 (quaternion* 0 1 0 0)
;;                                 (vec3* .7 0 .5))
;;       (quaternion-translation-2 (x-angle (* -.5 pi))
;;                                 nil)))

(defparameter *ws-goal*
  (tf* (quaternion* 0.0d0 1.0d0 0.0d0 0)
       (vec3* .8 -.25 .3051)))

(defparameter *goal-ik* (scene-graph-ik *scene-graph* :frame "right_endpoint" :tf *ws-goal*))

(win-set-config (scene-graph-ik *scene-graph* :frame "right_endpoint" :tf *ws-goal*))

(defparameter *mp*
  (motion-plan *ssg* *start*
               ;;:jointspace-goal *goal*
               ;;:allowed-list *allowed-collision*
               :workspace-goal *ws-goal*))

(win-view-plan *mp*)
