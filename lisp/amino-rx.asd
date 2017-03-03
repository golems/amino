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

(asdf:defsystem amino-rx
  :description "Scene graph data structure and associate operations."
  :long-description "Defines a purely-function interface for scene graphs, i.e., kinematic trees, that shares geometric types with the C implementation.  Binds associated operations such a collision checking, motion planning, and OpenGL rendering."
  :author ("Neil T. Dantam")
  :license :bsd-3
  :homepage "http://amino.golems.org"
  :source-control "https://github.com/golems/amino"
  :depends-on ("amino")
  ;; Keep these components in sync with Makefile.am
  :components ((:file "rx/robray-package")
               ;; bindings
               (cffi-grovel:grovel-file "rx/rx-grovel" :depends-on ("rx/robray-package"))
               (cffi-grovel:grovel-file "rx/sdl-grovel" :depends-on ("rx/robray-package"))
               (:file "rx/rx-lib" :depends-on ("rx/robray-package"))
               (:file "rx/rx-type" :depends-on ("rx/rx-grovel"))
               (:file "rx/geom-bind" :depends-on ("rx/rx-lib" "rx/rx-type"))
               (:file "rx/sg-cfun" :depends-on ("rx/rx-lib" "rx/rx-type" "rx/scenegraph"))
               (:file "rx/rx-sg" :depends-on ("rx/sg-cfun" "rx/rx-lib" "rx/rx-type" "rx/scenegraph"))
               (:file "rx/rx-gl" :depends-on ("rx/rx-lib" "rx/rx-type" "rx/rx-sg"))
               (:file "rx/rx-win" :depends-on ("rx/rx-lib" "rx/rx-type" "rx/rx-sg" "rx/rx-gl" "rx/sdl-grovel"))
               (:file "rx/rx-ik" :depends-on ("rx/rx-lib" "rx/rx-type" "rx/rx-sg"))
               (:file "rx/rx-cl" :depends-on ("rx/rx-lib" "rx/rx-type" "rx/rx-sg"))
               (:file "rx/rx-mp" :depends-on ("rx/rx-lib" "rx/rx-type" "rx/rx-sg"))
               ;; other things
               (:file "rx/util" :depends-on ("rx/robray-package"))
               (:file "rx/mesh" :depends-on ("rx/util"))
               (:file "rx/wavefront" :depends-on ("rx/mesh"))
               (:file "rx/parameters" :depends-on ("rx/robray-package"))
               (:file "rx/povray" :depends-on ("rx/util" "rx/parameters" "rx/robray-package" "rx/mesh"))
               ;;(:file "collada" :depends-on ("util" "povray" "mesh"))
               (:file "rx/scenegraph" :depends-on ("rx/util" "rx/povray" "rx/mesh" "rx/geom-bind"))
               (:file "rx/sg-rope" :depends-on ("rx/scenegraph" "rx/util"))
               (:file "rx/render" :depends-on ("rx/povray" "rx/scenegraph"))
               (:file "rx/render/mp-render" :depends-on ("rx/render"))
               (:file "rx/scenefile/urdf" :depends-on ("rx/util" "rx/povray" "rx/scenegraph" "rx/wavefront"))
               (:file "rx/lexer" :depends-on ("rx/util"))
               (:file "rx/inex" :depends-on ("rx/util"))
               (:file "rx/scenefile/curly" :depends-on ("rx/scenegraph" "rx/mesh" "rx/lexer" "rx/inex"))
               ;(:file "rx/scenefile/moveit" :depends-on ("rx/scenegraph" "rx/mesh"))
               (:file "rx/scenefile/scenefile"
                      :depends-on ("rx/scenefile/urdf" "rx/scenefile/curly"))
               (:file "rx/animate" :depends-on ("rx/scenegraph" "rx/povray"))
               (:file "rx/draw" :depends-on ("rx/scenegraph"))
               (:file "rx/draw-extra" :depends-on ("rx/scenegraph"))
               (:file "rx/trajectory" :depends-on ("rx/scenegraph"))
               (:file "rx/config" :depends-on ("rx/util" "rx/scenefile/urdf" "rx/parameters"))
               (:file "rx/sg-gen" :depends-on ("rx/scenegraph" "rx/mesh"))
               (:file "rx/driver" :depends-on ("rx/sg-gen" "rx/rx-win"))
               ))
