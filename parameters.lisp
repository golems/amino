(in-package :robray)


(defparameter *robray-tmp-directory* "/tmp/robray/")
  ;; (concatenate 'string
  ;;              "/tmp/"
  ;;              (sb-posix:getenv "USER")
  ;;              "-cache/robray/"))

(defparameter *width* (/ 1920 2))
(defparameter *height* (/ 1080 2))
(defparameter *quality* 1.0)
(defparameter *frames-per-second* 15)

(defvar *render-host-alist*)

(defparameter *render-options*
  `((:width . ,(/ 1920 2))
    (:height . ,(/ 1080 2))
    (:quality . 1.0)
    (:frames-per-second . 15)
    (:use-collision . nil)
    (:encode-video . t)
    (:antialias . t)
    (:render-frames .t)))

(defun get-render-option (options keyword &optional default)
  (let ((elt (assoc keyword options)))
    (if elt
        (cdr elt)
        default)))

(defun render-options-default (&key
                                 (options *render-options*)
                                 (width (get-render-option options :width))
                                 (height (get-render-option options :height))
                                 (quality (get-render-option options :quality))
                                 (frames-per-second (get-render-option options :frames-per-second))
                                 (use-collision (get-render-option options :use-collision))
                                 (encode-video (get-render-option options :encode-video))
                                 (antialias (get-render-option options :antialias))
                                 (render-frames (get-render-option options :render-frames)))
  (list (cons :width width)
        (cons :height height)
        (cons :quality quality)
        (cons :frames-per-second frames-per-second)
        (cons :use-collision use-collision)
        (cons :encode-video encode-video)
        (cons :antialias antialias)
        (cons :render-frames render-frames)))

(defun render-options (&rest options-plist)
  (plist-alist options-plist))

(defun merge-render-options (new-options &optional (base-options *render-options*))
  (append new-options base-options))

(defun render-options-fast (&optional (base-options *render-options*))
  (merge-render-options (render-options :quality 0.2d0
                                        :use-collision t
                                        :width (/ 1920 4)
                                        :height (/ 1080 4))
                        base-options))

(defun render-options-medium (&optional (base-options *render-options*))
  (merge-render-options (render-options :quality 0.5d0
                                        :use-collision nil
                                        :width (/ 1920 2)
                                        :height (/ 1080 2))
                        base-options))

(defun render-options-full-hd (&optional (base-options *render-options*))
  (merge-render-options (render-options :quality 1d0
                                        :use-collision nil
                                        :width 1920
                                        :height 1080)
                        base-options))
