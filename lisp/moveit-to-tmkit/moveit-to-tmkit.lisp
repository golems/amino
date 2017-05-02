(load "~/tmkit/load.lisp")

(in-package robray)

(require :amino-rx)

(defparameter *robray-tmp-directory* "/tmp/amino")

(defun reparent (parent-frame frame scene-graph)
  (setf scene-graph (robray::scene-graph-reparent scene-graph parent-frame frame)))

;; Loading Moveit scene into robray, with reparenting, etc.
(defun moveit-to-tmkit (moveit-file &optional (out-file "~/out.robray")) 
  (let* ((moveit-graph (robray::load-moveit-scene moveit-file)))
    (progn 
      (robray::scene-graph-f moveit-graph (scene-frame-fixed nil "null-frame"))
      (robray::map-scene-graph-frames nil #'(lambda (frame) (setf moveit-graph (robray::reparent "null-frame" (robray::scene-frame-name frame) moveit-graph))) moveit-graph) 
      ;; Annotate frame box1 with type moveable
      (with-open-file (stream out-file
                           :direction :output
                           :if-exists :supersede
                           :if-does-not-exist :create)
        (format stream (sycamore:rope-string (sycamore:rope moveit-graph)))))))
