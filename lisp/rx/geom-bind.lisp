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
;;; Geometry Options ;;;
;;;;;;;;;;;;;;;;;;;;;;;;

(cffi:defcfun aa-rx-geom-opt-destroy :void
  (value :pointer))

(cffi:defcfun aa-rx-geom-opt-create rx-geom-opt-t)

(defun make-geom-opt ()
  (amino-ffi::foreign-container-finalizer (aa-rx-geom-opt-create)
                                          #'%make-rx-geom-opt
                                          #'aa-rx-geom-opt-destroy))

(cffi:defcfun aa-rx-geom-opt-set-no-shadow :void
  (opts rx-geom-opt-t)
  (value :int))

(cffi:defcfun aa-rx-geom-opt-set-collision :void
  (opts rx-geom-opt-t)
  (value :int))

(cffi:defcfun aa-rx-geom-opt-set-visual :void
  (opts rx-geom-opt-t)
  (value :int))

(cffi:defcfun aa-rx-geom-opt-set-color3 :void
  (opts rx-geom-opt-t)
  (r :double)
  (g :double)
  (b :double))

(cffi:defcfun aa-rx-geom-opt-set-specular3 :void
  (opts rx-geom-opt-t)
  (r :double)
  (g :double)
  (b :double))

(cffi:defcfun aa-rx-geom-opt-set-alpha :void
  (opts rx-geom-opt-t)
  (a :double))

;;;;;;;;;;;;;;;;
;;; Geometry ;;;
;;;;;;;;;;;;;;;;

(cffi:defcfun aa-rx-geom-destroy :void
  (geom :pointer))

(cffi:defcfun aa-rx-geom-box rx-geom-t
  (opt rx-geom-opt-t)
  (dimension amino::vector-3-t))

(cffi:defcfun aa-rx-geom-sphere rx-geom-t
  (opt rx-geom-opt-t)
  (radius :double))

(cffi:defcfun aa-rx-geom-cylinder rx-geom-t
  (opt rx-geom-opt-t)
  (height :double)
  (radius :double))

(cffi:defcfun aa-rx-geom-cone rx-geom-t
  (opt rx-geom-opt-t)
  (height :double)
  (start-radius :double)
  (end-radius :double))

(cffi:defcfun aa-rx-geom-grid rx-geom-t
  (opt rx-geom-opt-t)
  (dimension amino::vector-2-t)
  (delta amino::vector-2-t)
  (width :double))

;;;;;;;;;;;;
;;; Mesh ;;;
;;;;;;;;;;;;

(cffi:defcfun aa-rx-mesh-create :pointer)
(cffi:defcfun aa-rx-mesh-destroy :void
  (mesh :pointer))

(cffi:defcfun aa-rx-mesh-set-vertices :void
  (mesh rx-mesh-t)
  (n amino-ffi:size-t)
  (vectors :float)
  (copy :int))

(cffi:defcfun aa-rx-mesh-set-normals :void
  (mesh rx-mesh-t)
  (n amino-ffi:size-t)
  (normals :float)
  (copy :int))

(cffi:defcfun aa-rx-mesh-set-indices :void
  (mesh rx-mesh-t)
  (n amino-ffi:size-t)
  (indices :unsigned-int)
  (copy :int))

(cffi:defcfun aa-rx-mesh-set-rgba :void
  (mesh rx-mesh-t)
  (width amino-ffi:size-t)
  (height amino-ffi:size-t)
  (rgba :uint8)
  (copy :int))

(cffi:defcfun aa-rx-mesh-set-texture :void
  (mesh rx-mesh-t)
  (opt rx-geom-opt-t))
