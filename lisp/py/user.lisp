(in-package |aminopy|)


;;;;;;;;;;;;;;;;
;; Transforms ;;
;;;;;;;;;;;;;;;;

(defun |tf| (a)
  (tf a))

(defun |tf2| (a b)
  (tf* a b))

(defun |xangle| (a)
  (x-angle a))

(defun |yangle| (a)
  (y-angle a))

(defun |zangle| (a)
  (z-angle a))

(defun |vec| (&rest args)
  (apply #'vec args))

(defun |rotation| (x)
  (rotation x))

(defun |translation| (x)
  (translation x))

(defun |vec3| (x)
  (vec3 x))

(defun |quat| (x)
  (quaternion x))


(defun |mul| (&rest args)
  (reduce #'g* args))

(defun |inverse| (x)
  (inverse x))

;; (defun |vec_y| (x)
;;   (vec-y x))

;; (defun |vec_z| (x)
;;   (vec-z x))

;;;;;;;;;;;;;;;;;
;;; Subscript ;;;
;;;;;;;;;;;;;;;;;

(defmethod clpython:py-subs ((x amino::real-array) (item #+clpython-fixnum-is-a-class fixnum #-clpython-fixnum-is-a-class integer)) ;; tuple
  (aref (amino::real-array-data x) item))

;;;;;;;;;;;;;;;;;;
;; Scene Graphs ;;
;;;;;;;;;;;;;;;;;;

(defun |map_frames| (function scene-graph)
  (robray::map-scene-graph-frames 'list function scene-graph))


(defun |scene_chain| (scene root tip)
  (robray::scene-graph-chain scene root tip))

(defmethod clpython:py-subs ((scene robray::scene-graph) name)
  (robray::scene-graph-find scene name))

(defmacro def-subs-accessors (type &body things)
  (with-gensyms (object name)
    `(defmethod clpython:py-subs ((,object ,type) ,name)
       (cond ,@(loop for (key function) in things
                  collect `((string= ,name ,key)
                            (,function ,object)))))))

(def-subs-accessors robray::scene-frame
  ("name" robray::scene-frame-name)
  ("parent" robray::scene-frame-parent frame)
  ("tf" robray::scene-frame-tf frame)
  ("geometry" robray::scene-frame-geometry frame)
  ("collision" robray::scene-frame-geometry-collision frame))



(defun |frame_isa| (frame type)
  (clpython:py-bool (robray::scene-frame-geometry-isa frame type)))


(defun |scene_frame_tf| (scene name config)
  (declare (ignore config))
  (robray::scene-graph-tf-absolute scene name))

(defun |scene_tf_abs| (scene config name)
  (robray::scene-graph-tf-absolute scene name
                                   :configuration-map config))

(defun |scene_tf_rel| (scene config parent child)
  (robray::scene-graph-tf-relative scene parent child
                                   :configuration-map config))

(defun |frame_fixed_tf| (frame)
  (robray::scene-frame-fixed-tf frame))

(def-subs-accessors robray::scene-geometry
  ("shape" robray::scene-geometry-shape))


(defun |shape_is_box| (shape)
  (robray::scene-box-p shape))

(defun |shape_is_sphere| (shape)
  (robray::scene-sphere-p shape))

(defun |shape_is_cylinder| (shape)
  (robray::scene-cylinder-p shape))

(defun |shape_is_cone| (shape)
  (robray::scene-cone-p shape))

(defun |shape_is_grid| (shape)
  (robray::scene-grid-p shape))

(defun |shape_is_text| (shape)
  (robray::scene-text-p shape))

(defun |shape_is_mesh| (shape)
  (robray::scene-mesh-p shape))

(def-subs-accessors robray::scene-box
  ("dimension" robray::scene-box-dimension))

(def-subs-accessors robray::scene-sphere
  ("radius" robray::scene-sphere-radius))

(def-subs-accessors robray::scene-cylinder
  ("radius" robray::scene-cylinder-radius)
  ("height" robray::scene-cylinder-height))

(def-subs-accessors robray::scene-cone
  ("start_radius" robray::scene-cone-start-radius)
  ("end_radius" robray::scene-cone-start-radius)
  ("height" robray::scene-cone-height))


;;;;;;;;;;;;;;;;;;;;;;;
;;; Motion Planning ;;;
;;;;;;;;;;;;;;;;;;;;;;;

(defun |motion_plan_ws| (sub-scene-graph start-configuration goal
                         &key
                           (|timeout| 1d0)
                           (|simplify| t))
  (robray::motion-plan sub-scene-graph start-configuration
                       :workspace-goal goal
                       :simplify |simplify|
                       :timeout |timeout|))


(defgeneric |motion_plan| (sub-scene-graph start-configuration goal
                              &key
                                |timeout|
                                |simplify|))
(defmethod |motion_plan| (sub-scene-graph start-configuration (goal quaternion-translation)
                         &key
                           (|timeout| 1d0)
                           (|simplify| t))
  (or (robray::motion-plan sub-scene-graph start-configuration
                           :workspace-goal goal
                           :simplify |simplify|
                           :timeout |timeout|)
      (clpython:py-bool nil)))


(defun |motion_plan_endpoint| (motion-plan)
  (robray::motion-plan-endpoint-map motion-plan))
