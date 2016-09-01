;;;; -*- mode: lisp -*-
;;;;
;;;; Copyright (c) 2016, Rice University
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

(defun aarx-command ()
  (labels ((env (name)
             (uiop/os:getenv name))
           (env-list (name)
             (read-from-string (concatenate 'string "(" (env name) ")"))))
    (let* ((reload (uiop/os:getenv "AARX_RELOAD"))
           (forward-axis (or (uiop/os:getenv "AARX_MESH_FORWARD_AXIS")
                             "Y"))
           (up-axis (or (uiop/os:getenv "AARX_MESH_UP_AXIS")
                        "Z"))
           (gui (uiop/os:getenv "AARX_GUI"))
           (output (env "AARX_OUTPUT"))
           (scene-name (env "AARX_SCENE_NAME"))
           (scene-files (env-list "AARX_SCENE"))
           (scene (fold (lambda (sg name)
                          (scene-graph sg
                                       (load-scene-file name
                                                        :compile gui
                                                        :emit-povray nil
                                                        :bind-c-geom gui
                                                        :reload reload
                                                        :mesh-up-axis up-axis
                                                        :mesh-forward-axis forward-axis)))
                        (scene-graph) scene-files)))
      ;; Compile Scene
      (when output
        (scene-graph-compile scene
                             output
                             :shared-object nil
                             :scene-name scene-name))
      ;; Display in GUI
      (when gui
        (win-create :title "AARXC" :stop-on-quit t)
        (win-set-scene-graph scene)
        (win-run :synchronous t)))))
