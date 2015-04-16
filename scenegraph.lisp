(in-package :robray)

(defstruct scene-graph
  (frame-map (make-hash-table :test #'equal)))

(defstruct scene-frame
  name
  parent
  )

(defstruct (scene-frame-fixed (:include scene-frame))
  tf)

(defstruct (scene-frame-joint (:include scene-frame))
  axis
  (offset 0d0 :type double-float))

(defstruct (scene-frame-revolute (:include scene-frame-joint))
  translation)

(defstruct (scene-frame-prismatic (:include scene-frame-joint))
  rotation)

;; Find the relative transform of the scene frame
(defgeneric scene-frame-tf (object))
