(in-package :robray)


(defparameter *robray-tmp-directory* "/tmp/robray/")
  ;; (concatenate 'string
  ;;              "/tmp/"
  ;;              (sb-posix:getenv "USER")
  ;;              "-cache/robray/"))

(defparameter *width* (/ 1920 2))
(defparameter *height* (/ 1080 2))
(defparameter *quality* 0.5)
(defparameter *frames-per-second* 15)

(defvar *render-host-alist*)
