(require :amino)

(in-package :robray)

(sb-posix:setenv "ROS_PACKAGE_PATH" "/opt/ros/indigo/share/" 1)

(setq *scene-graph* (load-scene-file "package://baxter_description/urdf/baxter.urdf"))

(win-set-scene-graph *scene-graph*)

(win-set-config `(("left_s0" . ,(/ pi 4))))
