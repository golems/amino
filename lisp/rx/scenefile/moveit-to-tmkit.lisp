(load "/home/cannon/tmkit/load.lisp")

(in-package robray)

(require :amino-rx)

(defparameter *robray-tmp-directory* "/tmp/amino-cannon")

(robray::load-moveit-scene "~/moveit_scene.scene")


(with-open-file (stream "~/moveit_scene.robray"
                     :direction :output
                     :if-exists :supersede
                     :if-does-not-exist :create)
  (format stream (sycamore:rope-string (sycamore:rope (robray::load-moveit-scene "~/moveit_scene.scene")))))


(TMSMT:TMP-DRIVER :START-SCENE '("package://ur_description/urdf/ur5_robotiq_robot_limited.urdf" "/home/cannon/baxter-blocks/moveit_scene.robray" "/home/cannon/baxter-blocks/ur5_robotiq85_allowed_collision.robray") :GOAL-SCENE '("/home/cannon/baxter-blocks/moveit_scene_goal.robray") :PDDL '("/home/cannon/baxter-blocks/tm-blocks.pddl") :GUI "1" :SCRIPTS '("/home/cannon/baxter-blocks/tm-blocks.py") :VERBOSE NIL :MAX-STEPS 10 :OUTPUT "ur5-robotiq-sussman.tmp" :WRITE-FACTS NIL :MOTION-TIMEOUT NIL :START-PLAN NIL :START NIL :PREFIX-CACHE T :CONSTRAINTS :STATE)

