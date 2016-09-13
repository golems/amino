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



(defparameter +copying+
  (read-file-into-string (merge-pathnames "COPYING" *robray-root*)))

(defun aarx-version (&optional (stream *standard-output*))
  (format stream
          "aarxc ~A

~A

Written by Neil T. Dantam
"
          amino::+version+
          +copying+
          ))

(defun aarx-version-man (&optional (stream *standard-output*))
  (format stream "~A"
          (ppcre:regex-replace-all (ppcre:create-scanner  "^[ \\*]*" :multi-line-mode t)
                                   (aarx-version nil) "")))

(defun aarx-driver ()
  (robray-tmpdir)
  (labels ((env (name)
             (uiop/os:getenv name))
           (env-integer (name)
             (parse-integer (env name)))
           (env-list (name)
             (read-from-string (concatenate 'string "(" (env name) ")")))
           (env-vec  (name)
             (apply #'vec (env-list name))))
    (let* ((reload (uiop/os:getenv "AARX_RELOAD"))
           (forward-axis (or (uiop/os:getenv "AARX_MESH_FORWARD_AXIS")
                             "Y"))
           (up-axis (or (uiop/os:getenv "AARX_MESH_UP_AXIS")
                        "Z"))
           (gui (uiop/os:getenv "AARX_GUI"))
           (output (env "AARX_OUTPUT"))
           (pov (uiop/os:getenv "AARX_POVRAY"))
           (scene-name (env "AARX_SCENE_NAME"))
           (scene-files (env-list "AARX_SCENE"))
           (config-alist (env-list "AARX_CONFIG"))
           (render (env "AARX_RENDER"))
           (config (alist-configuration-map config-alist))
           (camera-tf (amino::tf-mzlook :eye (env-vec "AARX_CAMERA_EYE")
                                        :target (env-vec "AARX_CAMERA_LOOK")
                                        :up (env-vec "AARX_CAMERA_UP")))
           (scene (fold (lambda (sg name)
                          (scene-graph sg
                                       (load-scene-file name
                                                        :compile gui
                                                        :emit-povray (or pov gui)
                                                        :bind-c-geom gui
                                                        :reload reload
                                                        :mesh-up-axis up-axis
                                                        :mesh-forward-axis forward-axis)))
                        (scene-graph) scene-files)))

      (when (zerop (length scene-files))
        (error "No input scene files specified"))
      ;; Print names
      (when (env "AARX_LIST_FRAMES")
        (format t "~{~&  ~A~%~}" (scene-graph-frame-names scene)))
      (when (env "AARX_LIST_VARS")
        (format t "~{~&  ~A~%~}" (scene-graph-configurations scene)))
      ;; Compile Scene
      (when output
        (scene-graph-compile scene
                             output
                             :shared-object nil
                             :scene-name scene-name))
      ;; POV-Ray
      (when pov
        (render-scene-graph scene
                            :camera-tf camera-tf

                            :configuration-map config
                            :options (render-options-default :width (env-integer "AARX_POV_X")
                                                             :height (env-integer "AARX_POV_Y"))
                            :output pov
                            :include (env-list "AARX_POV_INCLUDE")
                            :directory nil
                            :include-directory *robray-tmp-directory*
                            :render render
                            ))
      ;; Display in GUI
      (when gui
        (setq *win-render* render)
        (win-create :title "AARXC" :stop-on-quit t)
        (win-set-scene-graph scene)
        (setf (win-tf-camera) camera-tf)
        (when config-alist (win-set-config config))
        (win-run :synchronous t)))))

(defun aarx-command ()
  (labels ((env (name)
             (uiop/os:getenv name)))
    (cond ((env "AARX_VERSION")
           (aarx-version))
          ((env "AARX_VERSION_MAN")
           (aarx-version-man))
          (t (aarx-driver)))))
