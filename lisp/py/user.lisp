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

;; (defun |vec_x| (x)
;;   (vec-x x))

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
  ("geometry" robray::scene-frame-geometry frame))



(defun |frame_isa| (frame type)
  (clpython:py-bool (robray::scene-frame-geometry-isa frame type)))


(defun |scene_frame_tf| (scene name config)
  (declare (ignore config))
  (robray::scene-graph-tf-absolute scene name))

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
