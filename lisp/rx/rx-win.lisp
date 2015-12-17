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

;;;;;;;;;;;;;;;;;;;;;;;;
;;; Foreign Bindings ;;;
;;;;;;;;;;;;;;;;;;;;;;;;

(cffi:defcfun aa-rx-win-default-create rx-win-t
  (title :string)
  (width :int)
  (height :int))

(cffi:defcfun aa-rx-win-set-sg :void
  (window rx-win-t)
  (sg rx-sg-t))

(cffi:defcfun aa-rx-win-set-config :void
  (window rx-win-t)
  (n size-t)
  (q :pointer))

(defun rx-win-set-config (win config)
  (with-foreign-simple-vector (pointer length) config :input
    (aa-rx-win-set-config win length pointer)))

(cffi:defcfun aa-rx-win-start :void
  (window rx-win-t))

(cffi:defcfun aa-rx-win-sg-gl-init :void
  (win rx-win-t)
  (sg rx-sg-t))


(cffi:defcfun aa-rx-win-stop-on-quit :void
  (win rx-win-t)
  (value :boolean))

(cffi:defcfun aa-rx-win-stop :void
  (win rx-win-t))

(cffi:defcfun aa-rx-win-pause :void
  (win rx-win-t)
  (paused :boolean))

(cffi:defcfun aa-rx-win-set-display-plan :void
  (win rx-win-t)
  (n-plan amino-ffi:size-t)
  (plan :pointer)
  (refop ref-op))

(cffi:defcfun aa-rx-win-gl-globals :pointer
  (win rx-win-t))

;;;;;;;;;;;;;;;;;;;
;;; Convenience ;;;
;;;;;;;;;;;;;;;;;;;

(defvar *window* nil)

(defun win-create (&key
                        (title "AminoGL")
                        (width 800)
                        (height 600))
  (unless *window*
    (setq *window*
          (aa-rx-win-default-create title width height))
    (aa-rx-win-stop-on-quit *window* nil)
    (aa-rx-win-start *window*))
  *window*)

(defun win-destroy ()
  (assert *window*)
  (aa-rx-win-destroy (rx-win-pointer *window*))
  (setq *window* nil)
  (values))


(defun win-set-scene-graph (scene-graph)
  (let ((win (win-create))
        (m-sg (mutable-scene-graph scene-graph)))
    (aa-rx-win-sg-gl-init win m-sg)
    (setf (rx-win-mutable-scene-graph win) m-sg
          (rx-win-config-vector win) (make-vec (aa-rx-sg-config-count m-sg)))
    (aa-rx-win-set-sg win m-sg))
  (values))

(defun win-set-config (configs)
  (let* ((win (win-create))
         (sg (rx-win-mutable-scene-graph win))
         (q (rx-win-config-vector win)))
    (mutable-scene-graph-config-vector sg configs q)
    (rx-win-set-config win q)))

(defun win-pause (&optional (paused t))
  (aa-rx-win-pause (win-create) paused))

(defun win-unpause ()
  (aa-rx-win-pause (win-create) nil))

(defmacro with-win-paused (window &body body)
  `(progn (let ((*window* ,window))
            (win-pause)
            (unwind-protect (progn ,@body)
              (win-unpause)))))

(defun win-set-display-plan (plan)
  (when (zerop (length plan))
    (error "Cannot view empty plan"))
  (let ((win (win-create)))
    (with-foreign-simple-vector (pointer length) plan :input
      (aa-rx-win-set-display-plan win length pointer :copy))))

(defmacro with-win-gl-globals ((var &optional (window '*window*)) &body body)
  `(with-win-paused ,window
     (let ((,var (%make-rx-gl-globals (aa-rx-win-gl-globals ,window))))
       ,@body)))


(defun win-view-visual ()
  (with-win-gl-globals (gl-globals)
    (gl-globals-show-visual gl-globals t)
    (gl-globals-show-collision gl-globals nil)))

(defun win-view-collision ()
  (with-win-gl-globals (gl-globals)
    (gl-globals-show-visual gl-globals nil)
    (gl-globals-show-collision gl-globals t)))

(defun win-mask-frames (frames &key
                                 (hide t)
                                 (window *window*))
  (with-win-gl-globals (gl-globals window)
    (let ((m-sg (rx-win-mutable-scene-graph window)))
      (map nil (lambda (frame)
                 (let ((id (mutable-scene-graph-frame-id m-sg frame)))
                   (check-type id frame-id)
                   (aa-gl-globals-mask gl-globals id hide)))
           (ensure-list frames)))))


(defun win-mask-all (&key (window *window*)
                       (hide t))
  (with-win-gl-globals (gl-globals window)
    (if hide
        (let* ((m-sg (rx-win-mutable-scene-graph window))
               (sg (mutable-scene-graph-scene-graph m-sg)))
          (win-mask-frames (scene-graph-frame-names sg)
                           :window window
                           :hide hide))
        (gl-globals-unmask-all gl-globals))))
