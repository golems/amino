(in-package :robray)


(defparameter *robray-cache-directory*
  (concatenate 'string
               "/tmp/"
               (sb-posix:getenv "USER")
               "-cache/robray/"))

(defparameter *width* 1280)
(defparameter *height* 720)
(defparameter *quality* 0.5)
(defparameter *frames-per-second* 15)
