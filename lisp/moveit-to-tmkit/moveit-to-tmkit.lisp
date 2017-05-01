(load "/home/cannon/tmkit/load.lisp")

(in-package robray)

(require :amino-rx)

(defparameter *robray-tmp-directory* "/tmp/amino")

(robray::load-moveit-scene "~/moveit_scene.scene")

;; Loading Moveit scene into robray, with reparenting, etc.
(let* ((moveit-graph (robray::load-moveit-scene "~/moveit_scene.scene"))
      (box-frame (scene-graph-find moveit-graph "box1"))) 
  (progn 
    (robray::scene-graph-f moveit-graph (scene-frame-fixed nil "null-frame"))
    (setf moveit-graph (robray::scene-graph-reparent moveit-graph "null-frame" "box1"))
    (scene-graph-remove-frame moveit-graph "box1")
    ;; Annotate frame box1 with type moveable
    (setf (scene-geometry-type (car (scene-frame-geometry box-frame))) (sycamore:tree-set-insert (scene-geometry-type (car (scene-frame-geometry box-frame))) "moveable"))
    (print (draw-option (scene-geometry-options (car (scene-frame-geometry box-frame))) :type))
    (print (scene-graph-find moveit-graph "box1"))
    (with-open-file (stream "~/moveit_scene.robray"
                         :direction :output
                         :if-exists :supersede
                         :if-does-not-exist :create)
      (format stream (sycamore:rope-string (sycamore:rope moveit-graph))))))

;; Testing
(scene-graph-find (load-scene-file "/home/cannon/moveit_scene.robray") "box1")

;; Just writing to file
(with-open-file (stream "~/moveit_scene.robray"
                     :direction :output
                     :if-exists :supersede
                     :if-does-not-exist :create)
  (format stream (sycamore:rope-string (sycamore:rope (robray::load-moveit-scene "~/moveit_scene.scene")))))


;; Full driver run
(TMSMT:TMP-DRIVER :START-SCENE '("package://ur_description/urdf/ur5_robotiq_robot_limited.urdf" "/home/cannon/baxter-blocks/moveit_scene.robray" "/home/cannon/baxter-blocks/ur5_robotiq85_allowed_collision.robray") :GOAL-SCENE '("/home/cannon/moveit_scene_goal.robray") :PDDL '("/home/cannon/baxter-blocks/tm-blocks.pddl") :GUI "1" :SCRIPTS '("/home/cannon/baxter-blocks/tm-blocks.py") :VERBOSE NIL :MAX-STEPS 10 :OUTPUT "ur5-robotiq-sussman.tmp" :WRITE-FACTS NIL :MOTION-TIMEOUT NIL :START-PLAN NIL :START NIL :PREFIX-CACHE T)

