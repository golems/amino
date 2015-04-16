(in-package :robray)

(defstruct scene-graph
  (frame-map (make-hash-table :test #-string=)))

(defstruct scene-frame
  name
  children)

(defstruct (scene-frame-fixed (:include scene-frame))
  tf)

(defstruct (scene-frame-joint (:include scene-frame))
  axis
  (offset 0 :type double-float))

(defstruct (scene-frame-revolute (:include scene-frame-joint)))

(defstruct (scene-frame-prismatic (:include scene-frame-joint)))

;; Find the relative transform of the scene frame
(defgeneric scene-frame-tf (object))
