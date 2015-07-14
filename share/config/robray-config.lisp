(cl:in-package :robray)

(print "Loading robray config.lisp")

(defparameter *ros-distribution* "indigo")

(labels ((try-add (name)
           (let ((dir (format-pathname "/opt/ros/~A/share/~A/" *ros-distribution* name)))
             (when (probe-file dir)
               (pushnew (cons "baxter_description"  dir)
                        *urdf-package-alist* :test #'equal)))))
  (map nil #'try-add '("baxter_description")))


(setq *render-host-alist*
      '(("localhost" :jobs 1 :threads 1 :nice 1)))
