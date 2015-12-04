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
;;; Foreign Binding ;;;
;;;;;;;;;;;;;;;;;;;;;;;

(cffi:defcfun aa-rx-sg-destroy :void
  (value :pointer))

(cffi:defcfun aa-rx-sg-create rx-sg-t)

(cffi:defcfun aa-rx-sg-init :void
  (sg rx-sg-t))

(cffi:defcfun aa-rx-sg-frame-type frame-type
  (sg rx-sg-t)
  (frame rx-frame-id))

(cffi:defcfun aa-rx-sg-frame-name :string
  (sg rx-sg-t)
  (frame rx-frame-id))

(cffi:defcfun aa-rx-sg-config-name :string
  (sg rx-sg-t)
  (config rx-config-id))

(cffi:defcfun aa-rx-sg-frame-parent rx-frame-id
  (sg rx-sg-t)
  (frame rx-frame-id))

(cffi:defcfun aa-rx-sg-frame-count size-t
  (sg rx-sg-t))

(cffi:defcfun aa-rx-sg-config-count size-t
  (sg rx-sg-t))

(cffi:defcfun aa-rx-sg-frame-config rx-config-id
  (sg rx-sg-t)
  (frame rx-frame-id))

(cffi:defcfun aa-rx-sg-frame-id rx-frame-id
  (sg rx-sg-t)
  (name :string))

(cffi:defcfun aa-rx-sg-config-id rx-config-id
  (sg rx-sg-t)
  (name :string))

(cffi:defcfun aa-rx-sg-add-frame-fixed :void
  (sg rx-sg-t)
  (parent :string)
  (name :string)
  (q amino::quaternion-t)
  (v amino::vector-3-t))

(cffi:defcfun aa-rx-sg-add-frame-prismatic :void
  (sg rx-sg-t)
  (parent :string)
  (name :string)
  (q amino::quaternion-t)
  (v amino::vector-3-t)
  (config-name :string)
  (axis amino::vector-3-t)
  (offset :double))

(cffi:defcfun aa-rx-sg-add-frame-revolute :void
  (sg rx-sg-t)
  (parent :string)
  (name :string)
  (q amino::quaternion-t)
  (v amino::vector-3-t)
  (config-name :string)
  (axis amino::vector-3-t)
  (offset :double))

(cffi:defcfun aa-rx-sg-rm-frame :void
  (sg rx-sg-t)
  (name :string))

(cffi:defcfun aa-rx-sg-set-limit-pos :void
  (sg rx-sg-t)
  (config-name :string)
  (min :double)
  (max :double))

(cffi:defcfun aa-rx-sg-set-limit-vel :void
  (sg rx-sg-t)
  (config-name :string)
  (min :double)
  (max :double))

(cffi:defcfun aa-rx-sg-set-limit-acc :void
  (sg rx-sg-t)
  (config-name :string)
  (min :double)
  (max :double))

(cffi:defcfun aa-rx-sg-set-limit-eff :void
  (sg rx-sg-t)
  (config-name :string)
  (min :double)
  (max :double))
