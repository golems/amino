(cl:in-package :robray)

;; (defparameter *ros-distribution* "indigo")

;; (labels ((try-add (name)
;;            (let ((dir (format-pathname "/opt/ros/~A/share/~A/" *ros-distribution* name)))
;;              (when (probe-file dir)
;;                (urdf-package-add name dir)))))
;;   (map nil #'try-add '("baxter_description" "pr2_description" "ur_description")))


(setq *render-host-alist*
      '(("localhost" :jobs 1 :threads 1 :nice 1)))
