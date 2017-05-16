(require :amino)

(in-package :robray)

(sb-posix:setenv "ROS_PACKAGE_PATH" "/home/ntd/git/universal_robot:/home/ntd/git/robotiq_85_gripper:" 1)

(defparameter *robot* (load-scene-file  "package://ur_description/urdf/ur5_robotiq_robot_limited.urdf"))

(defparameter *scene-graph*
  (scene-graph *robot*
               "/home/ntd/git/bb/ml/setTable-0.robray"
               "/home/ntd/git/bb/ml/ur5_robotiq_allowed_collisions.robray")
               )

(defparameter *sub-scene-graph* (scene-graph-chain *scene-graph* nil "end_effector_grasp"))

(win-set-scene-graph *scene-graph*)

(win-run)


;(defparameter *e0* (tf (vec 0.000000	1.000000	0.000000	0.000000	0.800000	-0.250000	0.305100)))

(defparameter *q0*
  (alist-configuration-map
   '(("elbow_joint" . -2.324322543422016d0)
     ("robotiq_85_left_finger_tip_joint" . 0.0d0)
     ("robotiq_85_left_inner_knuckle_joint" . 0.0d0)
     ("robotiq_85_left_knuckle_joint" . 0.0d0)
     ("robotiq_85_right_finger_tip_joint" . 0.0d0)
     ("robotiq_85_right_inner_knuckle_joint" . 0.0d0)
     ("robotiq_85_right_knuckle_joint" . 0.0d0)
     ("shoulder_lift_joint" . -1.1503917877331418d0)
     ("shoulder_pan_joint" . -0.3381332967673545d0)
     ("wrist_1_joint" . -1.2376611048900046d0)
     ("wrist_2_joint" . 1.5708380473579282d0)
     ("wrist_3_joint" . 1.2318502675979834d0))))

(defparameter *qa*
  (alist-configuration-map
   '(("elbow_joint" . -2.235822574799285d0)
     ("robotiq_85_left_finger_tip_joint" . 0.0d0)
     ("robotiq_85_left_inner_knuckle_joint" . 0.0d0)
     ("robotiq_85_left_knuckle_joint" . 0.0d0)
     ("robotiq_85_right_finger_tip_joint" . 0.0d0)
     ("robotiq_85_right_inner_knuckle_joint" . 0.0d0)
     ("robotiq_85_right_knuckle_joint" . 0.0d0)
     ("shoulder_lift_joint" . -1.1170988115389706d0)
     ("shoulder_pan_joint" . -0.33825301387966017d0)
     ("wrist_1_joint" . -1.3594795494376741d0)
     ("wrist_2_joint" . 1.570712964711854d0)
     ("wrist_3_joint" . 1.2318151300715607d0))))

(defparameter *qb*
  (alist-configuration-map '(("elbow_joint" . -2.4113704814735484d0)
                             ("robotiq_85_left_finger_tip_joint" . 0.0d0)
                             ("robotiq_85_left_inner_knuckle_joint" . 0.0d0)
                             ("robotiq_85_left_knuckle_joint" . 0.0d0)
                             ("robotiq_85_right_finger_tip_joint" . 0.0d0)
                             ("robotiq_85_right_inner_knuckle_joint" . 0.0d0)
                             ("robotiq_85_right_knuckle_joint" . 0.0d0)
                             ("shoulder_lift_joint" . 0.40095058839326425d0)
                             ("shoulder_pan_joint" . 3.141590118408203d0)
                             ("wrist_1_joint" . -0.5758205398509717d0)
                             ("wrist_2_joint" . 3.141590118408203d0)
                             ("wrist_3_joint" . -0.38141502889322676d0))))

(defparameter *q0*
  (pairlist-configuration-map
   '("shoulder_pan_joint" "shoulder_lift_joint" "elbow_joint" "wrist_1_joint" "wrist_2_joint" "wrist_3_joint")
   ' (-0.33821567163893834  -1.150334446023314      -2.324365430817635      -1.23760360011491       1.5707888942908208      1.2317243437774275)))


(defparameter *q1*
  (pairlist-configuration-map
   '("shoulder_pan_joint" "shoulder_lift_joint" "elbow_joint" "wrist_1_joint" "wrist_2_joint" "wrist_3_joint")
   '(-0.33769870409392805  -1.6055076212573962     -1.860371638620203      1.8948923585697421      -1.5706029817777725     -1.9094012087364582)))

(defparameter *e0* (scene-graph-tf-absolute *scene-graph* "end_effector_grasp" :configuration-map *q0*))

(defparameter *e1* (tf-mul (tf* nil '(0 0 .1)) *e0*))

(defparameter *e1* (scene-graph-tf-absolute *scene-graph* "end_effector_grasp" :configuration-map *q1*))


(win-set-scene-graph *robot* :configuration-map *qa*)

(defparameter *ksol-opts*
  (ksol-opt :dt .1d0
            :gain-angle 5d0
            :gain-trans 5d0))

(defparameter *plan*
  (cartesian-path *sub-scene-graph* *q0* *e1*
                  :ksol-opts *ksol-opts*
                  ))

(win-display-motion-plan *plan*)
