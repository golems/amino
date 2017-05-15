;;;; -*- Lisp -*-
;;;;
;;;; Copyright (c) 2009-2011, Georgia Tech Research Corporation
;;;; All rights reserved.
;;;;
;;;; Author(s): Neil T. Dantam <ntd@gatech.edu>
;;;; Georgia Tech Humanoid Robotics Lab
;;;; Under Direction of Prof. Mike Stilman
;;;;
;;;;
;;;; This file is provided under the following "BSD-style" License:
;;;;
;;;;
;;;;   Redistribution and use in source and binary forms, with or
;;;;   without modification, are permitted provided that the following
;;;;   conditions are met:
;;;;
;;;;   * Redistributions of source code must retain the above copyright
;;;;     notice, this list of conditions and the following disclaimer.
;;;;
;;;;   * Redistributions in binary form must reproduce the above
;;;;     copyright notice, this list of conditions and the following
;;;;     disclaimer in the documentation and/or other materials provided
;;;;     with the distribution.
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

(cl:eval-when (:load-toplevel :execute)
    (asdf:operate 'asdf:load-op 'cffi-grovel))

(asdf:defsystem amino
  :description "Basic utilities for robotics."
  :long-description "Amino provides numerics, geometry, and memory-management for robotics and real-time applications.  This lisp system interfaces with a foreign (C) library."
  :depends-on ("cffi" "sycamore" "cxml" "cl-ppcre")
  :author ("Neil T. Dantam")
  :license :bsd-3
  :homepage "http://amino.golems.org"
  :source-control "https://github.com/golems/amino"
  ;; Keep these components in sync with Makefile.am
  :components ((:file "package")
               (:file "config-macro" :depends-on ("package"))
               (:file "config" :depends-on ("config-macro"))
               (:file "libdir" :depends-on ("config-macro"))
               (:file "util" :depends-on ("package" "libdir"))
               ;; TYPE
               (:file "basic-type" :depends-on ("package"))
               (:file "sparse" :depends-on ("package"))
               ;; FFI
               (cffi-grovel:grovel-file "grovel" :depends-on ("package"))
               (cffi-grovel:grovel-file "tf-grovel" :depends-on ("package"))
               (:file "ffi" :depends-on ("grovel" "basic-type"))
               (:file "foreign" :depends-on ("package"))
               (:file "blas" :depends-on ("foreign" "ffi"))
               (:file "libc" :depends-on ("foreign"))
               ;; Foreign LA
               (:file "amino-la" :depends-on ("foreign"))
               ;; LA
               (:file "basic-ops" :depends-on ("basic-type"))
               (:file "op" :depends-on ("package"))
               (:file "generic" :depends-on ("op" "tf-type"))
               (:file "blas-generic" :depends-on ("generic" "blas"))
               (:file "mem" :depends-on ("foreign"))
               (:file "ct" :depends-on ("mem" "tf-type"))
               (:file "io" :depends-on ("mem"))
               ;; TF
               (:file "tf-type" :depends-on ("foreign" "tf-grovel"))
               (:file "tf" :depends-on ("tf-type"))
               (:file "tf-op" :depends-on ("tf" "generic"))))
