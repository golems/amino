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


(cffi:defcfun aa-rx-globals-destroy :void
  (obj :pointer))

(cffi:defcfun aa-rx-gl-globals-create rx-gl-globals-t)

(cffi:defcfun (gl-globals-show-collision "aa_gl_globals_set_show_collision") :void
  (gl-globals rx-gl-globals-t)
  (show :boolean))

(cffi:defcfun (gl-globals-show-visual "aa_gl_globals_set_show_visual") :void
  (gl-globals rx-gl-globals-t)
  (show :boolean))

(cffi:defcfun (gl-globals-unmask-all "aa_gl_globals_unmask_all") :void
  (gl-globals rx-gl-globals-t))

(cffi:defcfun aa-gl-globals-mask :void
  (gl-globals rx-gl-globals-t)
  (i size-t)
  (value :boolean))

;;;;;;;;;;;;;;;;
;;; Wrappers ;;;
;;;;;;;;;;;;;;;;
