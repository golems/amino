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
  file
  original-file
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

(defun mesh-deindex-normals (mesh)
"Create one normal vector per vertex in the mesh.

The result is suitable for OpenGL."
  (let* ((v-indices (mesh-data-vertex-indices mesh))
         (n-indices (mesh-data-normal-indices mesh))
         (t-indices (mesh-data-texture-indices mesh))
         (v-len (length v-indices))
         (mesh-vertices (mesh-data-vertex-vectors mesh))
         (mesh-normals (mesh-data-normal-vectors mesh))
         (hash (make-hash-table :test #'equal))
         (count 0)
         (new-indices (make-array v-len
                                  :element-type 'fixnum)))
    (declare (type fixnum count))
    (assert (= v-len
               (length n-indices)))
    (loop ;; iterate over faces
       for i from 0 below v-len
       for v-i = (the fixnum (* 3 (aref v-indices i)))
       for n-i = (the fixnum (* 3 (aref n-indices i)))
       for v-n = (list (aref mesh-vertices (+ v-i 0))
                       (aref mesh-vertices (+ v-i 1))
                       (aref mesh-vertices (+ v-i 2))
                       (aref mesh-normals (+ n-i 0))
                       (aref mesh-normals (+ n-i 1))
                       (aref mesh-normals (+ n-i 2))
                       (when t-indices
                         (aref t-indices (truncate (/ i 3)))))
       do
         (if-let ((new-index (gethash v-n hash)))  ;; check if already hashed
           (setf (aref new-indices i)
                 new-index)
           (progn
             (setf (gethash v-n hash) count
                   (aref new-indices i) count)
             (incf count))))

    (assert (= count (hash-table-count hash)))
    (let ((new-vertices (make-vec (* 3 count)))
          (new-normals (make-vec (* 3 count)))
          (new-textures (make-fnvec (if t-indices count 0))))
      (maphash (lambda (v-n i)
                 (declare (type list v-n))
                 (let ((k (* 3 i)))
                   (replace new-vertices v-n :start1 k :start2 0 :end2 3)
                   (replace new-normals v-n :start1 k :start2 3 :end2 6))
                 (when t-indices
                   (setf (aref new-textures i) (elt v-n 6))))
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
