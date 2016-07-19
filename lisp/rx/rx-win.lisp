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


(cffi:defcfun aa-rx-win-lock :void
  (window rx-win-t))

(cffi:defcfun aa-rx-win-unlock :void
  (window rx-win-t))

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

(cffi:defcfun aa-rx-win-join :void
  (win rx-win-t))

(cffi:defcfun aa-rx-win-pause :void
  (win rx-win-t)
  (paused :boolean))

(cffi:defcfun aa-rx-win-set-display-plan :void
  (win rx-win-t)
  (sg rx-sg-t)
  (n-plan amino-ffi:size-t)
  (plan :pointer))

(cffi:defcfun aa-rx-win-gl-globals :pointer
  (win rx-win-t))

(cffi:defcfun aa-rx-win-set-display-seq :void
  (win rx-win-t)
  (mp-seq (rx-mp-seq-t)))

(cffi:defcfun aa-rx-win-get-tf-cam :void
  (win rx-win-t)
  (e :pointer))


(cffi:defcfun aa-rx-win-set-tf-cam :void
  (win rx-win-t)
  (e :pointer))

;;;;;;;;;;;;;;;;;;;
;;; Convenience ;;;
;;;;;;;;;;;;;;;;;;;

(defvar *window* nil)

(defmacro with-win-lock (win &body body)
  (with-gensyms (v-win)
    `(let ((,v-win ,win))
       (aa-rx-win-lock ,v-win)
       (unwind-protect (progn ,@body)
         (aa-rx-win-unlock ,v-win)))))

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

(defun win-stop (&optional (window *window*))
  (aa-rx-win-stop window))

(defun win-join (&optional (window *window*))
  (aa-rx-win-join window))

(cffi:defcfun aa-rx-win-destroy :void
  (obj :pointer))

(defun win-destroy ()
  (assert *window*)
  (aa-rx-win-destroy (rx-win-pointer *window*))
  (setq *window* nil)
  (values))


(defun win-set-scene-graph (scene-graph &optional (window (win-create)))
  (let* ((win window)
         (m-sg (mutable-scene-graph scene-graph))
         (q (make-vec (aa-rx-sg-config-count m-sg))))
    (with-win-lock win
      (aa-rx-win-sg-gl-init win m-sg)
      (aa-rx-win-set-sg win m-sg)
      ;; Preserve the window display object.  The active display
      ;; function may be using it.
      (setf (rx-win-mutable-scene-graph win) m-sg
            (rx-win-config-vector win) q)))
  (values))

(defun win-scene-graph (&optional (window *window*))
  (mutable-scene-graph-scene-graph (rx-win-mutable-scene-graph window)))

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

(defun win-display-motion-plan (motion-plan)
  (let ((window (win-create))
        (m-sg (motion-plan-mutable-scene-graph motion-plan))
        (path (motion-plan-path motion-plan))
        (n-path (motion-plan-length motion-plan)))
    (setf (rx-win-display-object window) motion-plan)
    (when (zerop (length path))
      (error "Cannot view empty plan"))
    (with-win-paused window
      (with-foreign-simple-vector (pointer length) path :input
        (declare (ignore length))
        (aa-rx-win-set-display-plan window m-sg n-path pointer)))))

(defun win-display-mp-seq (mp-seq)
  (let ((window (win-create)))
    (setf (rx-win-display-object window) mp-seq)
    (with-win-paused window
      (aa-rx-win-set-display-seq window mp-seq))))

(defun win-display-motion-plan-sequence (motion-plans)
  (let ((window (win-create)))
    (win-display-mp-seq (fold (lambda (mp-seq motion-plan)
                                (aa-rx-win-sg-gl-init window (motion-plan-mutable-scene-graph motion-plan))
                                (mp-seq-append-mp mp-seq motion-plan))
                              (make-mp-seq)
                              motion-plans))))

(defun win-tf-camera (&optional (window *window*))
  (let ((vec (make-vec 7)))
    (declare (dynamic-extent vec))
    (cffi:with-pointer-to-vector-data (vec vec)
      (aa-rx-win-get-tf-cam window vec))
    (tf vec)))

(defun (setf win-tf-camera) (value &optional (win *window*))
  (let ((vec (make-vec 7)))
    (declare (dynamic-extent vec))
    (tf-array (tf value) vec)
    (cffi:with-pointer-to-vector-data (vec vec)
      (aa-rx-win-set-tf-cam win vec))
    (tf vec)))

(defun win-config-map (&optional (window *window*))
  (mutable-scene-graph-config-map (rx-win-mutable-scene-graph window)
                                  (rx-win-config-vector window)))

(defun render-win (&key
                     (window *window*)
                     (camera-tf (win-tf-camera window))
                     (render t)
                     (options (render-options-default))
                     (configuration-map (win-config-map window))
                     output
                     (directory *robray-tmp-directory*)
                     include
                     (default-configuration 0d0))
  "Render the scene graph currently displayed in WINDOW"
  ;; TODO: configurations
  (render-scene-graph (mutable-scene-graph-scene-graph (rx-win-mutable-scene-graph window))
                      :camera-tf camera-tf
                      :render render
                      :options options
                      :configuration-map configuration-map
                      :output output
                      :directory directory
                      :include include
                      :default-configuration default-configuration))
