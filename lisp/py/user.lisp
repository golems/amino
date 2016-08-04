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

(defun |vec_x| (x)
  (vec-x x))

(defun |vec_y| (x)
  (vec-y x))

(defun |vec_z| (x)
  (vec-z x))

;;;;;;;;;;;;;;;;;;
;; Scene Graphs ;;
;;;;;;;;;;;;;;;;;;

(defun |map_frames| (function scene-graph)
  (robray::map-scene-graph-frames 'list function scene-graph))

(defun |frame_name| (frame)
  (robray::scene-frame-name frame))

(defun |frame_parent| (frame)
  (robray::scene-frame-parent frame))

(defun |frame_isa| (frame type)
  (clpython:py-bool (robray::scene-frame-geometry-isa frame type)))

(defun |find_frame| (scene name)
  (robray::scene-graph-find scene name))

(defun |scene_frame_tf| (scene name config)
  (declare (ignore config))
  (robray::scene-graph-tf-absolute scene name))

(defun |frame_fixed_tf| (frame)
  (robray::scene-frame-fixed-tf frame))
