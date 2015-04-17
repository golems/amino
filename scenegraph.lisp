(in-package :robray)

(defstruct scene-graph
  (frame-map (make-hash-table :test #'equal)))

(defstruct scene-mesh
  file)

(defstruct scene-box
  dimension)

(defstruct scene-sphere
  radius)

(defstruct scene-visual
  geometry
  color
  alpha)

(defstruct scene-frame
  name
  parent
  visual
  collision)

(defstruct (scene-frame-fixed (:include scene-frame))
  tf)

(defstruct (scene-frame-joint (:include scene-frame))
  axis
  (offset 0d0 :type double-float))

(defstruct (scene-frame-revolute (:include scene-frame-joint))
  translation)

(defstruct (scene-frame-prismatic (:include scene-frame-joint))
  rotation)

(defun scene-graph-add-frame (scene-graph frame)
  (setf (gethash (scene-frame-name frame) (scene-graph-frame-map scene-graph))
        frame)
  scene-graph)

(defun scene-graph (frames)
  (fold #'scene-graph-add-frame
        (make-scene-graph)
        frames))


(defun scene-graph-lookup (scene-graph frame-name)
  (gethash frame-name (scene-graph-frame-map scene-graph)))

;; Find the relative transform of the scene frame
(defgeneric scene-frame-tf (object))


(defun scene-graph-resolve-mesh (scene-graph)
  (let ((mesh-files  ;; filename => (list mesh-nodes)
         (make-hash-table :test #'equal)))
    (labels ((resolve-mesh (mesh)
               (push mesh
                     (gethash (scene-mesh-file mesh) mesh-files))))
      ;; collect mesh files
      (maphash-values (lambda (frame)
                                        ;(print frame)
                        (loop
                           for v in (scene-frame-visual frame)
                           for g = (scene-visual-geometry v)
                           when (and g (scene-mesh-p g))
                           do (resolve-mesh g)))
                      (scene-graph-frame-map scene-graph)))
    (maphash-keys (lambda (mesh-file)
                    (format *standard-output* "~&Converting ~A..." mesh-file)
                    (let ((dom (dom-load mesh-file)))
                      (collada-povray :dom dom
                                      :file (concatenate 'string
                                                         *robray-cache-directory*
                                                         "povray"
                                                         mesh-file ".inc"))))
                  mesh-files)))
