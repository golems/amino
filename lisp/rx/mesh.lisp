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


(defstruct mesh-data
  name
  (vertex-vectors nil :type (or null (simple-array double-float (*))))
  (vertex-indices nil :type (or null (simple-array fixnum (*))))
  (normal-vectors nil :type (or null (simple-array double-float (*))))
  (normal-indices nil :type (or null (simple-array fixnum (*))))
  (uv-vectors nil :type (or null (simple-array double-float (*))))
  (uv-indices nil :type (or null (simple-array fixnum (*))))
  texture-properties
  (texture-indices nil :type (or null (simple-array fixnum (*)))))

(defun mesh-data-vertex-vectors-count (mesh)
  (/ (length (mesh-data-vertex-vectors mesh))
     3))
(defun mesh-data-vertex-indices-count (mesh)
  (/ (length (mesh-data-vertex-indices mesh))
     3))
(defun mesh-data-normal-vectors-count (mesh)
  (/ (length (mesh-data-normal-vectors mesh))
     3))
(defun mesh-data-normal-indices-count (mesh)
  (/ (length (mesh-data-normal-indices mesh))
     3))

(defmethod print-object ((object mesh-data) stream)
  (print-unreadable-object (object stream :type t :identity nil)
    (format stream "~A" (mesh-data-name object))))

(defun mesh-vector-item (vector i)
  (vec (aref vector (+ 0 (* 3 i)))
       (aref vector (+ 1 (* 3 i)))
       (aref vector (+ 2 (* 3 i)))))

(defun mesh-vertex (mesh i)
  (mesh-vector-item (mesh-data-vertex-vectors mesh)
                    i))

(defun mesh-normal (mesh i)
  (mesh-vector-item (mesh-data-normal-vectors mesh)
                    i))

(defun mesh-deindex-normals (mesh)
"Create one normal vector per vertex in the mesh.

The result is suitable for OpenGL."
  (let* ((v-indices (mesh-data-vertex-indices mesh))
         (n-indices (mesh-data-normal-indices mesh))
         (t-indices (mesh-data-texture-indices mesh))
         (hash (make-hash-table :test #'equalp))
         (count 0)
         (new-indices (make-array (length v-indices)
                                  :element-type 'fixnum)))
    (assert (= (length v-indices)
               (length n-indices)))
    (loop ;; iterate over faces
       for i below (length v-indices)
       for vertex = (mesh-vertex mesh (aref v-indices i))
       for normal = (mesh-normal mesh (aref n-indices i))
       for texture = (when t-indices (aref t-indices (truncate (/ i 3))))
       for v-n = (list vertex normal texture)
       for new-index = (gethash v-n hash) ;; check if already hashed
       do
         (if new-index
             (setf (aref new-indices i)
                   new-index)
             (progn
               (setf (gethash v-n hash) count
                     (aref new-indices i) count)
               (incf count))))
    (assert (= count (hash-table-count hash)))
    (let ((new-vertices (make-vec (* 3 count)))
          (new-normals (make-vec (* 3 count)))
          (new-textures (when t-indices (make-fnvec count))))
      (maphash (lambda (k i)
                 (destructuring-bind (vertex normal texture) k
                   (dotimes (j 3)
                     (let ((offset (+ j (* 3 i))))
                       (setf (aref new-vertices offset) (aref vertex j))
                       (setf (aref new-normals offset) (aref normal j))))
                   (when new-textures
                     ;(print 'a)
                     (setf (aref new-textures i) texture)
                    ; (print 'b)
                     )))
               hash)
      (make-mesh-data :name (mesh-data-name mesh)
                      :vertex-vectors new-vertices
                      :vertex-indices new-indices
                      :normal-vectors new-normals
                      :normal-indices nil
                      :uv-vectors (mesh-data-uv-vectors mesh)
                      :uv-indices (mesh-data-uv-indices mesh)
                      :texture-properties (mesh-data-texture-properties mesh)
                      :texture-indices new-textures))))
