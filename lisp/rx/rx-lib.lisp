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

(cffi:define-foreign-library libgl
  (:unix (:or "libGL.so"
              (:default "libGL")))
  (t (:default "libGL")))

(cffi:define-foreign-library libglu
  (:unix (:or "libGLU.so"
              (:default "libGLU")))
  (t (:default "libGLU")))

(cffi:define-foreign-library libsdl
  (:unix (:or "libSDL2.so"
              (:default "libSDL2")))
  (t (:default "libSDL2")))

(cffi:define-foreign-library libamino-gl
  (:unix (:or "libamino_gl.so"
              (:default "libamino_gl")))
  (t (:default "libamino_gl")))

(cffi:define-foreign-library libamino-planning
  (:unix (:or "libamino_planning.so"
              (:default "libamino_planning")))
  (t (:default "libamino_planning")))

(cffi:define-foreign-library libamino-cl
  (:unix (:or "libamino_cl.so"
              (:default "libamino_cl")))
  (t (:default "libamino_cl")))


;; TODO: put in separate packages so reloads don't clobber static vars
(cffi:use-foreign-library libgl)
(cffi:use-foreign-library libglu)
(cffi:use-foreign-library libsdl)
(cffi:use-foreign-library libamino-gl)
(cffi:use-foreign-library libamino-planning)
(cffi:use-foreign-library libamino-cl)
