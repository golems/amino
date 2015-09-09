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
         (hash (make-hash-table :test #'equalp))
         (count 0)
         (new-indices (make-array (length v-indices)
                                  :element-type 'fixnum)))
    (assert (= (length v-indices)
               (length n-indices)))
    (loop
       for i below (length v-indices)
       for vertex = (mesh-vertex mesh (aref v-indices i))
       for normal = (mesh-normal mesh (aref n-indices i))
       for v-n = (cons vertex normal)
       do
         (if-let ((new-index (gethash v-n hash)))
           (setf (aref new-indices i)
                 new-index)
           (progn
             (setf (gethash v-n hash) count
                   (aref new-indices i) count)
             (incf count))))
    (let ((new-vertices (make-vec (* 3 count)))
          (new-normals (make-vec (* 3 count))))
      (maphash (lambda (k i)
                 (destructuring-bind (vertex . normal) k
                   (dotimes (j 3)
                     (let ((offset (+ j (* 3 i))))
                       (setf (aref new-vertices offset) (aref vertex j))
                       (setf (aref new-normals offset) (aref normal j))))))
               hash)
      (make-mesh-data :name (mesh-data-name mesh)
                      :vertex-vectors new-vertices
                      :vertex-indices new-indices
                      :normal-vectors new-normals
                      :normal-indices nil
                      :uv-vectors (mesh-data-uv-vectors mesh)
                      :uv-indices (mesh-data-uv-indices mesh)
                      :texture-properties (mesh-data-texture-properties mesh)
                      :texture-indices (mesh-data-texture-indices mesh)))))
