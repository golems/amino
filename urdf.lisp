(in-package :robray)

(defvar *urdf-dom*)

(defstruct urdf-joint
  name
  rpy
  xyz
  type
  axis
  parent
  child)

(defun urdf-load (file)
  "Create the DOM for a Collada file."
  (setq *urdf-dom* (dom-load file)))

(defun urdf-extract-joints (&key (dom *urdf-dom*))
  (loop with robot-node = (dom-select-tag dom "robot" :singleton t)
     for joint-node in (dom-select-tag robot-node "joint" :direct t)
     collect
       (labels ((type (text)
                  (string-case text
                    ("fixed" :fixed)
                    ("revolute" :revolute)
                    (otherwise (error "Unknown joint type: ~A" text))))
                (path (path)
                  (dom-select-path joint-node path
                                   :singleton t :undefined-error nil))
                (maybe-floats (text)
                  (when text
                    (parse-float-sequence text))))
         (make-urdf-joint :name (dom:get-attribute joint-node "name")
                          :type (type (dom:get-attribute joint-node "type"))
                          :rpy (maybe-floats (path '("origin" "@rpy")))
                          :xyz (maybe-floats (path '("origin" "@xyz")))
                          :axis (maybe-floats (path '("axis" "@xyz")))
                          :parent (path '("parent" "@link"))
                          :child (path '("child" "@link"))
                          ))))

         ;; (declare (ignore sub))
         ;;             (cond
         ;;               ((string= "origin" elt)
         ;;                (setf (urdf-joint-rpy joint)
         ;;                      (parse-string-vector (get-attr "rpy" attrs)))
         ;;                (setf (urdf-joint-xyz joint)
         ;;                      (parse-string-vector (get-attr "xyz" attrs))))
         ;;               ((string= "parent" elt)
         ;;                (setf (urdf-joint-parent joint) (get-attr "link" attrs)))
         ;;               ((string= "child" elt)
         ;;                (setf (urdf-joint-child joint) (get-attr "link" attrs)))
         ;;               ((string= "axis" elt)
         ;;                (setf (urdf-joint-axis joint)
         ;;                      (parse-string-vector (get-attr "xyz" attrs)))))))
         ;;         (push joint joints))

(defun urdf-parse (&key (dom *urdf-dom*))
  (let ((robot-node (dom-select-tag dom "robot" :singleton t))
        (joint-parent (make-hash-table :test #'equal)) ;; joint name -> parent link
       ; (child-links (make-hash-table :test #-string=))
        )
    ;; extract link map
    (loop for joint in (dom-select-tag robot-node "joint" :direct t)
       for name = (dom:get-attribute joint "name")
       for parent-node = (dom-select-tag joint "parent" :direct t :singleton t)
       for parent-link = (dom:get-attribute parent-node "link")
       ;for child-node = (dom-select-tag joint "parent" :direct t :singleton t)
       ;for child-link = (dom:get-attribute child-node "link")
       do (setf (gethash name joint-parent)
                parent-link))
    ;;
    (print 1)
    (loop for joint in (dom-select-tag dom "joint")
         for name = (dom:get-attribute joint "name")
         for parent = (gethash name joint-parent)
         do (format t "~&~A => ~A" parent name))

    ))
