;;;; -*- Lisp -*-
;;;;
;;;; Copyright (c) 2015, Rice University
;;;; All rights reserved.
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

;; (cl:eval-when (:load-toplevel :execute)
;;     (asdf:operate 'asdf:load-op 'cffi-grovel))

(asdf:defsystem robray
  :license :bsd-3
  :description "Robotics Raytracining Frontend"
  :source-control "https://github.com/KavrakiLab/robray"
  ;:homepage "http://ndantam.github.io/cl-ntdoc"
  :author ("Neil T. Dantam" )
  :version "0.1"
  :depends-on ("amino" "sycamore" "cl-ppcre" "cxml")
  :components ((:file "src/robray-package")
               (:file "src/util" :depends-on ("src/robray-package"))
               (:file "src/mesh" :depends-on ("src/util"))
               (:file "src/wavefront" :depends-on ("src/mesh"))
               (:file "src/parameters" :depends-on ("src/robray-package"))
               (:file "src/povray" :depends-on ("src/util" "src/parameters" "src/robray-package" "src/mesh"))
               ;;(:file "collada" :depends-on ("util" "povray" "mesh"))
               (:file "src/scenegraph" :depends-on ("src/util" "src/povray" "src/mesh"))
               (:file "src/scenefile/urdf" :depends-on ("src/util" "src/povray" "src/scenegraph" "src/wavefront"))
               (:file "src/lexer" :depends-on ("src/util"))
               (:file "src/inex" :depends-on ("src/util"))
               (:file "src/scenefile/curly" :depends-on ("src/scenegraph" "src/mesh" "src/lexer" "src/inex"))
               (:file "src/scenefile/moveit" :depends-on ("src/scenegraph" "src/mesh"))
               (:file "src/scenefile/scenefile"
                      :depends-on ("src/scenefile/urdf" "src/scenefile/curly" "src/scenefile/moveit"))
               (:file "src/animate" :depends-on ("src/scenegraph" "src/povray"))
               (:file "src/draw" :depends-on ("src/scenegraph"))
               (:file "src/trajectory" :depends-on ("src/scenegraph"))
               (:file "src/config" :depends-on ("src/util" "src/scenefile/urdf" "src/parameters"))
               )
  :long-description "Robotics Raytracining Frontend"
  )
