(load "~/tmkit/load.lisp")

(in-package robray)

(require :amino-rx)

(defparameter *robray-tmp-directory* "/tmp/amino")

;; Loading Moveit scene into robray, with reparenting, etc.
(defun moveit-to-tmkit (moveit-file &optional (out-file "~/out.robray")) 
  (let* ((moveit-graph (robray::load-moveit-scene moveit-file))
        (box-frame (scene-graph-find moveit-graph "box1"))) 
    (progn 
      (robray::scene-graph-f moveit-graph (scene-frame-fixed nil "null-frame"))
      (setf moveit-graph (robray::scene-graph-reparent moveit-graph "null-frame" "box1"))
      (scene-graph-remove-frame moveit-graph "box1")
      ;; Annotate frame box1 with type moveable
      (setf (scene-geometry-type (car (scene-frame-geometry box-frame))) (sycamore:tree-set-insert (scene-geometry-type (car (scene-frame-geometry box-frame))) "moveable"))
      (with-open-file (stream out-file
                           :direction :output
                           :if-exists :supersede
                           :if-does-not-exist :create)
        (format stream (sycamore:rope-string (sycamore:rope moveit-graph)))))))
