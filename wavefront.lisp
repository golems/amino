(in-package :robray)

(defstruct wavefront-obj-face
  vertex-index
  normal-index
  texture-index
  smooth
  group
  material
  )

(defparameter +wavefront-obj-scanner-v-vn+
  (ppcre:create-scanner "f\\s+(\\d+)//(\\d+) (\\d+)//(\\d+) (\\d+)//(\\d+)"))

(defun parse-wavefront-face (line)
  (labels ((subseq-list (reg-start reg-end items)
             (loop for i in items
                collect (1- (parse-integer line
                                           :start (aref reg-start i)
                                           :end (aref reg-end i))))))
  (multiple-value-bind (start end reg-start reg-end)
      (ppcre:scan +wavefront-obj-scanner-v-vn+ line)
    (declare (ignore end))
    (when start
      (return-from parse-wavefront-face
        (make-wavefront-obj-face :vertex-index (subseq-list reg-start reg-end '(0 2 4))
                                 :normal-index (subseq-list reg-start reg-end '(1 3 5))))))
  (error "Bad or unimplimented face line: ~A" line)))

(defun wavefront-load (input-file)
  (let ((name)
        (vertices        ;(vector (vec x y z)...)
         (make-array 10 :adjustable t :fill-pointer 0))
        (normals         ;(vector (vec x y z)...)
         (make-array 10 :adjustable t :fill-pointer 0))
        (faces  ;(vector (array x y z)...)
         (make-array 10 :adjustable t :fill-pointer 0)))
    (with-open-file (stream input-file :direction :input)
      (loop for line = (read-line stream  nil nil)
         while line
         do
           (case (aref line 0)
             (#\o (assert (null name) () "Multiple objects not implemented")
                  (setq name (subseq line 2)))
             (#\v
              (assert (>= (length line) 2))
              (ecase (aref line 1)
                (#\Space
                 (vector-push-extend (parse-float-sequence line 2) vertices))
                (#\n (vector-push-extend (parse-float-sequence line 3) normals))))
             (#\f (vector-push-extend (parse-wavefront-face line) faces)))))
   (make-mesh-data  :name name
                    :vertices (array-cat 'double-float vertices)
                    :normals (array-cat 'double-float normals)
                    :vertex-indices (array-cat 'fixnum
                                               (loop for f across faces
                                                  collect (wavefront-obj-face-vertex-index f)))
                    :normal-indices (array-cat 'fixnum
                                               (loop for f across faces
                                                  collect (wavefront-obj-face-normal-index f))))))


(defun wavefront-povray (input-file output-file
                         &key
                           (directory *robray-tmp-directory*))
  (let ((mesh-data (wavefront-load input-file)))
    (output (pov-declare (name-mangle (mesh-data-name mesh-data))
                         (pov-mesh2 :mesh-data mesh-data :handedness :right))
            output-file
            :directory directory)))
