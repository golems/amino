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
  (ppcre:create-scanner "f\\s+(\\d+)/(\\d*)/(\\d+) (\\d+)/(\\d*)/(\\d+) (\\d+)/(\\d*)/(\\d+)"))

(defparameter +wavefront-command-scanner+
  (ppcre:create-scanner "\\s*(\\S+)\\s*(.*)"))

(defun wavefront-strip-comment (line)
  (when line
    (let* ((i (position #\# line))
           (stripped (if i
                         (subseq line 0 i)
                         line)))
      (if (ppcre:scan "^\\s*$" stripped)
          nil
          stripped))))

(defun parse-wavefront-face (line material)
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
        (make-wavefront-obj-face :vertex-index (subseq-list reg-start reg-end '(0 3 6))
                                 :normal-index (subseq-list reg-start reg-end '(2 5 8))
                                 :material material))))
  (error "Bad or unimplimented face line: ~A" line)))


;; See: https://corona-renderer.com/wiki/standalone/mtl

(defun wavefront-mtl-load (mtl-file obj-file)
  (let ((mtl-file (output-file mtl-file (pathname-directory obj-file)))
        (material-alist)
        (current-alist)
        (current-mtl)
        (current-specular)
        (current-specular-weight))
    (labels ((add-current ()
               (when current-mtl
                 (prop :specular ;(g* current-specular-weight current-specular))
                       current-specular)
                 (push (cons current-mtl current-alist)
                       material-alist)))
             (newmtl (name)
               (add-current)
               (setq current-mtl name
                     current-alist nil
                     current-specular '(0d0 0d0 0d0)
                     current-specular-weight 1.0))
             (prop (name value)
               (assert current-mtl)
               (push (cons name value) current-alist)))
      (with-open-file (stream mtl-file :direction :input)
        (loop for line = (read-line stream nil nil)
           for lineno from 0
           while line
           do
             (let ((line (wavefront-strip-comment line))
                   (matched nil))
               (ppcre:register-groups-bind (command data)
                   (+wavefront-command-scanner+ line)
                 (setq matched t)
                 (labels ((float-prop (name)
                            (prop name (parse-float data)))
                          (rgb-prop (name)
                            (prop name (parse-float-sequence data))))
                   (string-case command
                     ("newmtl"
                      (newmtl data))
                     ("Ns"
                      (setq current-specular-weight (parse-float data)))
                     ("Ks"
                      (setq current-specular (parse-float-sequence data)))
                     ("Ka" (rgb-prop :ambient))
                     ;("Ka" (prop :ambient '(.1 .1 .1))
                     ("Kd" (rgb-prop :diffuse))
                     ("Ni" (float-prop :index-of-refraction))
                     ("d" (prop :alpha (invert-scale (parse-float data))))
                     ("map_Kd" ;TODO
                      )
                     ("illum" ;TODO
                      )
                     (otherwise (error "Unrecognized MTL command ~A" command)))))
               (assert (or (null line) matched)))))
      (add-current))
    material-alist))



(defun wavefront-obj-load (obj-file)
  (let ((name)
        (vertices        ;(vector (vec x y z)...)
         (make-array 10 :adjustable t :fill-pointer 0))
        (normals         ;(vector (vec x y z)...)
         (make-array 10 :adjustable t :fill-pointer 0))
        (faces  ;(vector (array x y z)...)
         (make-array 10 :adjustable t :fill-pointer 0))
        (materials)
        (current-material))
    (with-open-file (stream obj-file :direction :input)
      (loop for line = (read-line stream  nil nil)
         for lineno from 0
         while line
         do
           (let ((line (wavefront-strip-comment line))
                 (matched nil))
             (ppcre:register-groups-bind (command data)
                 (+wavefront-command-scanner+ line)
               (setq matched t)
               (string-case command
                 ("o"  (assert (null name) () "Multiple objects not implemented")
                       (setq name data))
                 ("v"
                  (vector-push-extend (parse-float-sequence data) vertices))
                 ("vn"
                  (vector-push-extend (parse-float-sequence data) normals))
                 ("f" (vector-push-extend (parse-wavefront-face line current-material) faces))
                 ("mtllib"
                  (setq materials (append materials (wavefront-mtl-load data obj-file))))
                 ("usemtl" (setq current-material data))
                 ("vt" ; TODO
                  )
                 ("g"; TODO
                  )
                 ("s"; TODO
                  )
                 ("l"; TODO
                  )
                 (otherwise (error "Unrecognized command ~A on line ~D" command lineno))))
             (assert (or (null line) matched) ()
                     "Error on line ~D" lineno))))
    (make-mesh-data  :name (name-mangle name)
                     :vertices (array-cat 'double-float vertices)
                     :normals (array-cat 'double-float normals)
                     :texture-properties (map 'list #'cdr materials)
                     :texture-indices (map-into (make-array (length faces) :element-type 'fixnum)
                                                (lambda (f)
                                                  (let ((material (wavefront-obj-face-material f)))
                                                    (position-if (lambda (x) (string= (car x) material))
                                                                 materials)))
                                                faces)
                     :vertex-indices (array-cat 'fixnum
                                                (loop for f across faces
                                                  collect (wavefront-obj-face-vertex-index f)))
                     :normal-indices (array-cat 'fixnum
                                                (loop for f across faces
                                                   collect (wavefront-obj-face-normal-index f))))))


(defun wavefront-povray (obj-file output-file
                         &key (handedness :right))
  (let* ((mesh-data (wavefront-obj-load obj-file))
         (name (mesh-data-name mesh-data)))
    (output (pov-declare (mesh-data-name mesh-data)
                         (pov-mesh2 :mesh-data mesh-data :handedness handedness))
            output-file)
    name))


(defun mesh-povray (mesh-file
                    &key
                      (mesh-up-axis "Z")
                      (mesh-forward-axis "Y")
                      (directory *robray-tmp-directory*))
  ;; Maybe convert
  (labels ((handle-obj (obj-file output-file handedness)
             (let ((name (wavefront-povray obj-file output-file :handedness handedness)))
               (values name output-file)))
           (handle-dae (dae-file output-file)
             (convert dae-file output-file :right))
           (convert (source-file output-file handedness)
             (format t "~&Converting to obj ~A" mesh-file)
             (let* ((obj-file (clean-pathname (format-pathname "~A/povray/~A.obj"
                                                               directory source-file)))
                    (args (list "blender" "-b" "-P"
                                (find-script "meshconv")
                                "--"
                                source-file
                                "-o" obj-file
                                (format nil "--up=~A" mesh-up-axis)
                                (format nil "--forward=~A" mesh-forward-axis))))
               (format t "~&~{~A~^ ~}" args)
               (ensure-directories-exist obj-file)
               (multiple-value-bind (output error-output status)
                   (uiop:run-program args)
                 (declare (ignore output error-output))
                 (assert (zerop status) () "mesh conversion failed"))
               (handle-obj obj-file output-file handedness))))
    (let* ((file-type (string-downcase (file-type mesh-file)))
           (file-basename (file-basename mesh-file))
           (file-dirname (file-dirname mesh-file))
           (output-file (format-pathname "~A/povray/~A/~A.inc"
                                         directory file-dirname file-basename)))
    (string-case file-type
      ("obj" (handle-obj mesh-file output-file :right))
      ("stl"
       (let ((dae1 (format-pathname "~A/~A.dae" file-dirname file-basename))
             (dae2 (format-pathname "~A/~A.DAE" file-dirname file-basename)))
         (cond ((probe-file dae1) (handle-dae dae1 output-file))
               ((probe-file dae2) (handle-dae dae2 output-file))
               (t (convert mesh-file output-file :right)))))
      ("dae"
       (handle-dae mesh-file output-file))
      (otherwise (error "Unknown file type: ~A" file-type))))))
