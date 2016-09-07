;;;; -*- mode: lisp -*-
;;;;
;;;; Copyright (c) 2015, Rice University
;;;; All rights reserved.
;;;;
;;;; Author(s): Neil T. Dantam <ntd@gatech.edu>
;;;;
;;;;   Redistribution and use in source and binary forms, with or
;;;;   without modification, are permitted provided that the following
;;;;   conditions are met:
;;;;
;;;;   * Redistributions of source code must retain the above
;;;;     copyright notice, this list of conditions and the following
;;;;     disclaimer.
;;;;   * Redistributions in binary form must reproduce the above
;;;;     copyright notice, this list of conditions and the following
;;;;     disclaimer in the documentation and/or other materials
;;;;     provided with the distribution.
;;;;   * Neither the name of copyright holder the names of its
;;;;     contributors may be used to endorse or promote products
;;;;     derived from this software without specific prior written
;;;;     permission.
;;;;
;;;;   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
;;;;   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
;;;;   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
;;;;   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
;;;;   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
;;;;   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
;;;;   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
;;;;   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
;;;;   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
;;;;   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
;;;;   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
;;;;   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;;;   POSSIBILITY OF SUCH DAMAGE.

(in-package :robray)

(defparameter *robray-tmp-root* (pathname "/tmp/"))

(defvar *robray-tmp-directory*)

(defun robray-tmpdir ()
  (setq *robray-tmp-directory*
        (if-let ((dir (uiop/os:getenv "AARX_TMPDIR")))
          (pathname (format nil "~A/" dir))
          (subdir *robray-tmp-root*
                  :directory (format nil "amino-~A" (uiop/os:getenv "USER"))))))

(robray-tmpdir)

(defparameter *robray-cache-directory*
  (subdir *robray-tmp-directory* :directory "cache"))

(defun mesh-directory (directory)
  (clean-pathname (concatenate 'string (namestring directory) "/povray/")))

(defparameter *width* (/ 1920 2))
(defparameter *height* (/ 1080 2))
(defparameter *quality* 1.0)
(defparameter *frames-per-second* 15)

(defvar *render-host-alist*)

(defparameter *render-options*
  `((:width . ,(/ 1920 2))
    (:height . ,(/ 1080 2))
    (:quality . 1.0)
    (:codec . :x264)
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
                                 (codec (get-render-option options :codec))
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
        (cons :codec codec)
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
                                        :height 1080
                                        :frames-per-second 30)
                        base-options))

(defun render-options-4k (&optional (base-options *render-options*))
  (merge-render-options (render-options :quality 1d0
                                        :use-collision nil
                                        :width (* 2 1920)
                                        :height (* 2 1080)
                                        :frames-per-second 30)
                        base-options))
