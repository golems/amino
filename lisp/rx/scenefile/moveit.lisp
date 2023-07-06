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

(defun export-scene-graph-moveit (scene-graph
                                  &key
                                    (name "noname")
                                    (stream *standard-output*))
  (format stream "~&(~A)" name)
  ;; * NAME
  ;; 1
  ;; type
  ;; size
  ;; translation
  ;; quaternion
  ;; RGBA
  (do-scene-graph-geometry ((frame geometry) scene-graph)
    (let* ((shape (scene-geometry-shape geometry))
           (frame-name (scene-frame-name frame))
           (tf (scene-graph-tf-absolute scene-graph frame-name))
           (v (tf-translation tf))
           (q (tf-quaternion tf))
           (options (scene-geometry-options geometry))
           (color (draw-option options :color))
           (alpha (draw-option options :alpha)))
      ;; shape
      (format stream "~&* ~A" frame-name)
      (format stream "~&1")
      (etypecase shape
        (scene-box
         (with-vec3 (x y z) (scene-box-dimension shape)
           (format stream "~&box~&~F ~F ~F" x y z))))
      ;; translation
      (format stream "~&~F ~F ~F"
              (vecref v 0)
              (vecref v 1)
              (vecref v 2))
      ;; quaternion
      (format stream "~&~F ~F ~F ~F"
              (quaternion-x q)
              (quaternion-y q)
              (quaternion-z q)
              (quaternion-w q))
      ;; quaternion
      (format stream "~&~F ~F ~F ~F"
              (vecref color 0)
              (vecref color 1)
              (vecref color 2)
              alpha))))


(let ((scanner-eof (ppcre:create-scanner "^\\.\\s*$"))
      (scanner-object (ppcre:create-scanner "^\\*\\s+(\\S+)$")))

  (defun load-moveit-scene (pathname
                            &key
                              reload-meshes
                              (directory *robray-tmp-directory*))
    (declare (ignore reload-meshes))
    (with-open-file (stream pathname :direction :input)
      (let ((lineno 0)
            (directory (clean-pathname (concatenate 'string directory "/moveit/")))
            (scene-graph (scene-graph nil)))
        (labels ((line ()
                   (incf lineno)
                   (read-line stream nil nil))
                 (vec-line ()
                     (parse-float-sequence (line)))
                 (int-line ()
                     (parse-integer-list (line)))
                 (next-object ()
                   (let ((line (line)))
                     (if (or (null line)
                             (ppcre:scan scanner-eof line))
                         nil
                         (let ((object-name (ppcre:register-groups-bind (object-name)
                                                (scanner-object line)
                                              object-name)))
                           (unless object-name
                             (error "Malformed object at line ~D: \"~A\"" lineno line))
                           (let ((count (parse-integer (line))))
                             (parse-object object-name count))))))
                 (parse-object (name count)
                   (loop
                      for i below count
                      for object-name = (if (= 1 count)
                                            name
                                            (format nil "~A__~D" name i))
                      for type = (line)
                      do (string-case type
                           ("box" (parse-box object-name))
                           ("mesh" (parse-mesh object-name))
                           (otherwise
                            (error "Unrecognized object type at line ~D: \"~A\"" lineno type))))
                   name)
                 (parse-box (name)
                   (let ((size (vec-line))
                         (translation (vec-line))
                         (quaternion (vec-line))
                         (rgba (vec-line)))
                     (scene-graph-f
                      scene-graph
                      (scene-frame-fixed nil name
                                         :tf (tf* (quaternion quaternion)
                                                  (vec3 translation))
                                         :geometry (scene-geometry-box (draw-options-default :color (subseq rgba 0 3)
                                                                                             :alpha (elt rgba 3))
                                                                        size)))))
                 (parse-mesh (name)
                   (destructuring-bind (vertex-count face-count)
                       (int-line)
                     (let* ((vertices (loop for i below vertex-count
                                         append (vec-line)))
                            (faces (loop for i below face-count
                                      append (int-line)))
                            (translation (vec-line))
                            (quaternion (vec-line))
                            (rgba (vec-line))
                            (color (subseq rgba 0 3))
                            (alpha (elt rgba 3)))
                       (let* ((mesh-data (make-mesh-data :vertex-vectors (list-double-vector vertices)
                                                         :vertex-indices (list-fixnum-vector faces)
                                                         :texture-properties `(((:color . ,color)
                                                                                (:alpha . ,alpha)))))
                              (name (name-mangle name))
                              (inc-file (format nil "~A.inc" name))
                              (povray-file (format nil "moveit/~A" inc-file)))
                         (ensure-directories-exist directory)
                         (output (pov-declare name (pov-mesh2 :mesh-data mesh-data))
                                 inc-file
                                 :directory directory)
                         (scene-graph-f
                          scene-graph
                          (scene-frame-fixed nil name
                                             :tf (tf* (quaternion quaternion)
                                                      (vec3 translation))
                                             :geometry
                                             (scene-geometry-mesh (draw-options-default :color color
                                                                                        :alpha alpha)
                                                                  (make-scene-mesh :name name
                                                                                   :povray-file povray-file)))))))))
          (let ((scene-name (line)))
            (format t "~&Reading scene '~A'...~%" scene-name))
          (loop for x = (next-object)
             while x))
        (format t "~&   done" )
        scene-graph))))
