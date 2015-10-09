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


;;;;;;;;;;;;;;;;;;;;;;;
;;; Foreign Library ;;;
;;;;;;;;;;;;;;;;;;;;;;;

;; TODO: generate from pkg-config and configure

(cffi:define-foreign-library libgl
  (:unix "libGL.so")
  (t (:default "libGL")))

(cffi:define-foreign-library libglu
  (:unix "libGLU.so")
  (t (:default "libGLU")))

(cffi:define-foreign-library libsdl
  (:unix "libSDL2.so")
  (t (:default "libSDL2")))

(cffi:define-foreign-library libamino-gl
  (:unix "libamino_gl.so")
  (t (:default "libamino_gl")))

(cffi:use-foreign-library libgl)
(cffi:use-foreign-library libglu)
(cffi:use-foreign-library libsdl)
(cffi:use-foreign-library libamino-gl)

;;;;;;;;;;;;;;;;;;;;;;;;
;;; Foreign Bindings ;;;
;;;;;;;;;;;;;;;;;;;;;;;;

(cffi:defcfun aa-rx-win-destroy :void
  (obj :pointer))

(cffi:defcfun aa-rx-win-default-create rx-win-t
  (title :string)
  (width :int)
  (height :int))

(cffi:defcfun aa-rx-win-set-sg :void
  (window rx-win-t)
  (sg rx-sg-t))

(cffi:defcfun aa-rx-win-default-start :void
  (window rx-win-t))

(cffi:defcfun aa-rx-win-sg-gl-init :void
  (win rx-win-t)
  (sg rx-sg-t))

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
    (aa-rx-win-default-start *window*))
  (values))

(defun win-destroy ()
  (assert *window*)
  (aa-rx-win-destroy (rx-win-pointer *window*))
  (setq *window* nil)
  (values))

(defun win-set-scene-graph (scene-graph)
  (win-create)
  (let ((win *window*)
        (m-sg (mutable-scene-graph (scene-graph scene-graph))))
    (aa-rx-win-sg-gl-init win m-sg)
    (setf (rx-win-mutable-scene-graph win)
          m-sg)
    (aa-rx-win-set-sg win m-sg))
  (values))
