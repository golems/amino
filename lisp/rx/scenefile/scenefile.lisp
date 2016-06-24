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

(defun scene-file-type (filename)
  (let ((filetype (file-type filename)))
    (if filetype
        (string-case (string-downcase filetype)
          ("urdf" :urdf)
          (("robray" "curly") :curly)
          (otherwise (error "Unrecognized scene file suffix: ~A" filetype)))
        :curly)))


(defun scene-graph-resolve-c! (scene-graph &key
                                             filename
                                             reload
                                             (compile t))
  (when compile
    (assert filename)
    ;; Compile the the thing
    (let ((base (format-pathname "~A/cache/~A" *robray-tmp-directory* filename)))
      (scene-graph-compile scene-graph (format-pathname "~A.c" base)
                           :shared-object (format-pathname "~A.so" base)
                           :reload reload
                           :static-mesh nil
                           :link-meshes t)))
  ;; Bind the C meshes
  (let ((hash (make-hash-table :test #'equalp)))
    ;; load C mesh
    (loop for mesh in (scene-graph-meshes scene-graph)
       do (setf (gethash mesh hash)
                (load-rx-mesh mesh)))
    ;; Bind
    (do-scene-graph-geometry ((frame geometry) scene-graph)
      (declare (ignore frame))
      (let ((shape (scene-geometry-shape geometry)))
        (when (and (scene-mesh-p shape)
                   (null (scene-geometry-c-geom geometry)))
          (setf (scene-geometry-c-geom geometry)
                (aa-rx-geom-mesh (alist-rx-geom-opt (scene-geometry-options geometry))
                                 (gethash shape hash))))))
    ;; Release meshes
    ;; Don't need to do this, the registered destructor will free the mesh
    ;; (maphash (lambda (k rx-mesh)
    ;;            (declare (ignore k))
    ;;            (rx-mesh-destroy (rx-mesh-pointer rx-mesh)))
    ;;          hash)
    ))


(defun scene-graph-resolve! (scene-graph &key
                                           filename
                                           reload
                                           (bind-c-geom t)
                                           (compile t)
                                           (mesh-up-axis "Z")
                                           (mesh-forward-axis "Y"))
  (let ((scene-graph (scene-graph scene-graph)))
    (scene-graph-resolve-povray! scene-graph
                                 :reload reload
                                 :mesh-up-axis mesh-up-axis
                                 :mesh-forward-axis mesh-forward-axis)
    (when bind-c-geom
      (scene-graph-resolve-c! scene-graph
                              :filename filename
                              :reload reload
                              :compile compile))
    scene-graph))

(defun load-scene-file (filename
                        &key
                          type
                          reload
                          (bind-c-geom t)
                          (compile t)
                          (mesh-up-axis "Z")
                          (mesh-forward-axis "Y"))

  (let* ((filename (rope-string (rope filename)))
         (type (or type (scene-file-type filename)))
         (truename (if (eq type :urdf)
                       (urdf-resolve-file filename)
                       filename))
         (scene-graph
          (ecase type
            (:urdf (urdf-parse truename
                               :reload-meshes reload
                               :mesh-up-axis mesh-up-axis
                               :mesh-forward-axis mesh-forward-axis))
            (:curly (load-curly-scene truename :reload-meshes reload))
            ;; (:moveit (load-moveit-scene truename :reload-meshes reload-meshes))
            )))
    (scene-graph-resolve! scene-graph
                          :filename truename
                          :reload reload
                          :bind-c-geom bind-c-geom
                          :compile compile
                          :mesh-up-axis mesh-up-axis
                          :mesh-forward-axis mesh-forward-axis)
    scene-graph))
