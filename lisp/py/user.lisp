(in-package |aminopy|)

(defmacro def-attr-accessor (((object type) attr) &body body)
  `(defmethod clpython::attr-hook ((,object ,type) (attr (eql ',(intern (string attr)
                                                                        :clpython.user))))
     ,@body))

(defmacro def-subs-accessors (type &body things)
  (with-gensyms (object name)
    `(progn (defmethod clpython:py-subs ((,object ,type) ,name)
              (cond ,@(loop for (key function) in things
                         collect `((string= ,name ,key)
                                   (,function ,object)))))
            ,@(loop for (key function) in things
                   collect
                   `(def-attr-accessor ((,object ,type) ,key)
                      (,function ,object))))))

;;;;;;;;;;;;;;;;
;; Transforms ;;
;;;;;;;;;;;;;;;;

(defun |tf| (object)
  "Convert OBJECT to the standard transform type."
  (tf object))

(defmethod clpython::attr-hook ((object quaternion-translation)
                                (attr (eql 'clpython.user::|translation|)))
  (amino::quaternion-translation-translation object))

(defmethod clpython::attr-hook ((object quaternion-translation)
                                (attr (eql 'clpython.user::|rotation|)))
  (amino::quaternion-translation-quaternion object))

(defun |tf2| (rotation translation)
  "Create a standard transform type for ROTATION and TRANSLATION.

Examples:

    tf2(1, [1,2,3]) =>
        #S(AMINO:QUATERNION-TRANSLATION
           :QUATERNION #S(AMINO:QUATERNION :DATA #(0.0d0 0.0d0 0.0d0 1.0d0))
           :TRANSLATION #S(AMINO:VEC3 :DATA #(1.0d0 2.0d0 3.0d0)))

    tf2(xangle(math.pi), [1,2,3]) =>
        #S(AMINO:QUATERNION-TRANSLATION
           :QUATERNION #S(AMINO:QUATERNION
                          :DATA #(1.0d0 0.0d0 0.0d0 6.123233995736766d-17))
           :TRANSLATION #S(AMINO:VEC3 :DATA #(1.0d0 2.0d0 3.0d0)))
"
  (tf* rotation translation))

(defun |xangle| (angle)
  "Create a rotation of ANGLE about the X axis.

Examples:

    xangle(math.pi) =>
        #S(AMINO:X-ANGLE :VALUE 3.141592653589793d0)
"
  (x-angle angle))

(defun |yangle| (angle)
  "Create a rotation of ANGLE about the Y axis.

Examples:

    yangle(math.pi) =>
        #S(AMINO:Y-ANGLE :VALUE 3.141592653589793d0)
"
  (y-angle angle))

(defun |zangle| (angle)
  "Create a rotation of ANGLE about the Z axis.

Examples:

    zangle(math.p) =>
        #S(AMINO:Z-ANGLE :VALUE 3.141592653589793d0)
"
  (z-angle angle))

(defun |vec| (&rest elements)
  "Create a numerical vector composted of ELEMENTS.

Examples:

    vec(1,2,3) =>
        [1.0, 2.0, 3.0]
"
  (apply #'vec elements))

(defun |rotation| (transform)
  "Extract the rotation part of TRANSFORM.

Examples:

    rotation(tf2(1, [1,2,3])) =>
        #S(AMINO:QUATERNION :DATA #(0.0d0 0.0d0 0.0d0 1.0d0))

    tf2(1, [1,2,3]).rotation =>
        #S(AMINO:QUATERNION :DATA #(0.0d0 0.0d0 0.0d0 1.0d0))

"
  (rotation transform))

(defun |translation| (transform)
  "Extract the TRANSLATION part of TRANSFORM.

Examples:

    translation(tf2(1, [1,2,3])) =>
        #S(AMINO:VEC3 :DATA #(1.0d0 2.0d0 3.0d0))

    tf2(1, [1,2,3]).translation =>
        #S(AMINO:VEC3 :DATA #(1.0d0 2.0d0 3.0d0))
"
  (translation transform))

(def-attr-accessor ((object quaternion-translation) |rotation|)
  (tf-quaternion object))

(def-attr-accessor ((object quaternion-translation) |translation|)
  (tf-translation object))

(defun |vec3| (object)
  "Convert OBJECT to a VEC3.

Examples:

    vec3([1,2,3]) =>
        #S(AMINO:VEC3 :DATA #(1.0d0 2.0d0 3.0d0))
"
  (vec3 object))

(defmethod clpython::attr-hook ((object vec3)
                                (attr (eql 'clpython.user::|x|)))
  (aref (amino::vec3-data object)
        amino::+x+))

(defmethod clpython::attr-hook ((object vec3)
                                (attr (eql 'clpython.user::|y|)))
  (aref (amino::vec3-data object)
        amino::+y+))

(defmethod clpython::attr-hook ((object vec3)
                                (attr (eql 'clpython.user::|z|)))
  (aref (amino::vec3-data object)
        amino::+z+))


(defun |quat| (object)
  "Convert OBJECT to a quaternion.

Examples:

    quat([1,0,0,0]) =>
        #S(AMINO:QUATERNION :DATA #(1.0d0 0.0d0 0.0d0 0.0d0))

    quat(1) =>
        #S(AMINO:QUATERNION :DATA #(0.0d0 0.0d0 0.0d0 1.0d0))

    quat(xangle(math.pi)) =>
        #S(AMINO:QUATERNION :DATA #(1.0d0 0.0d0 0.0d0 6.123233995736766d-17))
"
  (quaternion object))


(defun |mul| (&rest args)
  "Generically multiply ARGS.

Examples:

    mul(xangle(1),yangle(2)) =>
        #S(AMINO:QUATERNION
           :DATA #(0.2590347239999257d0 0.7384602626041288d0 0.4034226801113349d0
                   0.4741598817790379d0))

    mul(tf2(xangle(1),[1,2,3]), tf2(1,[4,5,6])) =>
        #S(AMINO:QUATERNION-TRANSLATION
           :QUATERNION #S(AMINO:QUATERNION
                          :DATA #(0.479425538604203d0 0.0d0 0.0d0
                                  0.8775825618903728d0))
           :TRANSLATION #S(AMINO:VEC3
                           :DATA #(5.0d0 -0.3473143795066811d0 10.449168759248321d0)))
"
  (reduce #'g* args))

(defun |inverse| (x)
  "Compute the inverse of X.

Examples:

    inverse(xangle(1)) =>
        #S(AMINO:QUATERNION
           :DATA #(-0.479425538604203d0 -0.0d0 -0.0d0 0.8775825618903728d0))

    inverse(tf2(xangle(1),[1,2,3]))
        #S(AMINO:QUATERNION-TRANSLATION
           :QUATERNION #S(AMINO:QUATERNION
                          :DATA #(-0.479425538604203d0 -0.0d0 -0.0d0 0.8775825618903728d0))
           :TRANSLATION #S(AMINO:VEC3
                           :DATA #(-1.0d0 -3.605017566159969d0 0.06203505201137416d0)))
"
  (inverse x))

;;;;;;;;;;;;;;;;;
;;; Subscript ;;;
;;;;;;;;;;;;;;;;;

(defmethod clpython:py-subs ((x amino::real-array) (item #+clpython-fixnum-is-a-class fixnum #-clpython-fixnum-is-a-class integer)) ;; tuple
  (aref (amino::real-array-data x) item))

(defmacro def-quat-attr (name index)
  `(def-attr-accessor ((object amino::real-array) ,name)
     (vecref object ,index)))

(def-quat-attr |x| amino::+x+)
(def-quat-attr |y| amino::+y+)
(def-quat-attr |z| amino::+z+)
(def-quat-attr |w| amino::+w+)

;;;;;;;;;;;;;;;;;;
;; Scene Graphs ;;
;;;;;;;;;;;;;;;;;;

(defun |map_frames| (function scene-graph)
  "Apply FUNCTION to every frame in SCENE-GRAPH."
  (robray::map-scene-graph-frames 'list function scene-graph))


(defun |scene_chain| (scene root tip)
  "Create a kinematic chain from ROOT to TIP."
  (robray::scene-graph-chain scene root tip))

(defmethod clpython:py-subs ((scene robray::scene-graph) name)
  (robray::scene-graph-find scene name))


(def-subs-accessors robray::scene-frame
  ("name" robray::scene-frame-name)
  ("parent" robray::scene-frame-parent frame)
  ("tf" robray::scene-frame-tf frame)
  ("geometry" robray::scene-frame-geometry frame)
  ("collision" robray::scene-frame-geometry-collision frame))

(defun |frame_isa| (frame type)
  "Test if FRAME is of the given TYPE."
  (clpython:py-bool (robray::scene-frame-geometry-isa frame type)))


(defun |scene_frame_tf| (scene name config)
  "Return the absolute transform of the given frame."
  (robray::scene-graph-tf-absolute scene name :configuration-map config))

(defun |scene_tf_abs| (scene config name)
  "Return the absolute transform of the given frame."
  (robray::scene-graph-tf-absolute scene name
                                   :configuration-map config))

(defun |scene_tf_rel| (scene config parent child)
  "Return the relative transform from parent to child."
  (robray::scene-graph-tf-relative scene parent child
                                   :configuration-map config))

(defun |frame_fixed_tf| (frame)
  "Return the fixed portion of the frame's relative transform."
  (robray::scene-frame-fixed-tf frame))

(def-subs-accessors robray::scene-geometry
  ("shape" robray::scene-geometry-shape))


(defun |shape_is_box| (shape)
  "Test if SHAPE is a box."
  (clpython:py-bool (robray::scene-box-p shape)))

(defun |shape_is_sphere| (shape)
  "Test if SHAPE is a sphere."
  (clpython:py-bool (robray::scene-sphere-p shape)))

(defun |shape_is_cylinder| (shape)
  "Test if SHAPE is a cylinder."
  (clpython:py-bool (robray::scene-cylinder-p shape)))

(defun |shape_is_cone| (shape)
  "Test if SHAPE is a cone."
  (clpython:py-bool (robray::scene-cone-p shape)))

(defun |shape_is_grid| (shape)
  "Test if SHAPE is a grid."
  (clpython:py-bool (robray::scene-grid-p shape)))

(defun |shape_is_text| (shape)
  "Test if SHAPE is a text."
  (clpython:py-bool (robray::scene-text-p shape)))

(defun |shape_is_mesh| (shape)
  "Test if SHAPE is a mesh."
  (clpython:py-bool (robray::scene-mesh-p shape)))

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
  "Compute a motion plan to a workspace goal.

If motion planning fails, return False."
  (or (robray::motion-plan sub-scene-graph start-configuration
                           :workspace-goal goal
                           :simplify |simplify|
                           :timeout |timeout|)
      (clpython:py-bool nil)))


(defun |motion_plan_endpoint| (motion-plan)
  "Return the endpoint of a motion plan."
  (robray::motion-plan-endpoint-map motion-plan))
