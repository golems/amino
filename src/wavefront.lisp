(in-package :robray)

;;(declaim (optimize (speed 3) (safety 0)))

(defstruct wavefront-obj-face
  vertex-index
  normal-index
  texture-index
  uv-index
  smooth
  group
  material
  )

(defparameter +wavefront-obj-scanner-v-vn+
  (ppcre:create-scanner "^f\\s+(\\d+)//(\\d+)\\s+(\\d+)//(\\d+)\\s+(\\d+)//(\\d+)\\s*$"))

(defparameter +wavefront-obj-scanner-v-vt-vn+
  (ppcre:create-scanner
   "^f\\s+(\\d+)/(\\d+)/(\\d+)\\s+(\\d+)/(\\d+)/(\\d+)\\s+(\\d+)/(\\d+)/(\\d+)\\s*$"))


(defparameter +wavefront-obj-scanner-v-vt+
  (ppcre:create-scanner
   "^f\\s+(\\d+)/(\\d+)\\s+(\\d+)/(\\d+)\\s+(\\d+)/(\\d+)\\s*$"))

(defparameter +wavefront-command-scanner+
  (ppcre:create-scanner "\\s*(\\S+)\\s*(.*)"))

(defun wavefront-strip-comment (line)
  (declare (type simple-string line))
  (when line
    (let* ((i (position #\# line))
           (stripped (if i
                         (subseq line 0 i)
                         line)))
      (if (ppcre:scan "^\\s*$" stripped)
          nil
          stripped))))

(defun parse-wavefront-face (line material lineno)
  (declare (type simple-string line))
  (labels ((subseq-list (reg-start reg-end items)
             (declare (type simple-vector reg-start reg-end))
             (loop for i in items
                for s = (aref reg-start i)
                for e = (aref reg-end i)
                collect (1- (parse-integer line :start s :end e)))))

    ;; V/Vt/Vn
    (multiple-value-bind (start end reg-start reg-end)
        (ppcre:scan +wavefront-obj-scanner-v-vt-vn+ line)
      (declare (ignore end))
      (when start
        (return-from parse-wavefront-face
          (make-wavefront-obj-face :vertex-index (subseq-list reg-start reg-end '(0 3 6))
                                   :uv-index     (subseq-list reg-start reg-end '(1 4 7))
                                   :normal-index (subseq-list reg-start reg-end '(2 5 8))
                                   :material material))))

    ;; V//Vn
    (multiple-value-bind (start end reg-start reg-end)
        (ppcre:scan +wavefront-obj-scanner-v-vn+ line)
      (declare (ignore end))
      (when start
        (return-from parse-wavefront-face
          (make-wavefront-obj-face :vertex-index (subseq-list reg-start reg-end '(0 2 4))
                                   :normal-index (subseq-list reg-start reg-end '(1 3 5))
                                   :material material))))

    ;; V/Vt
    (multiple-value-bind (start end reg-start reg-end)
        (ppcre:scan +wavefront-obj-scanner-v-vt+ line)
      (declare (ignore end))
      (when start
        (return-from parse-wavefront-face
          (make-wavefront-obj-face :vertex-index (subseq-list reg-start reg-end '(0 2 4))
                                   :uv-index (subseq-list reg-start reg-end '(1 3 5))
                                   :material material))))

  (error "Bad or unimplimented face on line ~D: \"~A\"" lineno line)))


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
                     ("d" (prop :alpha  (parse-float data)))
                     ("map_Kd"
                      ;; TODO: local path names
                      (let* ((dir (file-dirname data))
                             (base (file-basename data))
                             (ext (pov-image-map-type (string-downcase (file-type data))))
                             (cache-file (pov-cache-file (format-pathname "~A/~A.~A" dir base ext))))
                        (uiop/stream:copy-file data cache-file)
                        (prop :image-map cache-file)))
                     ("illum" ;TODO
                      )
                     (otherwise (error "Unrecognized MTL command ~A" command)))))
               (assert (or (null line) matched)))))
      (add-current))
    material-alist))



(defun wavefront-obj-name (obj-file)
  (let ((name))
    (with-open-file (stream obj-file :direction :input)
      (loop for line = (read-line stream nil nil)
         for lineno from 0
         while line
         when (eq #\o (aref line 0))
         do
           (if name
               (error "Duplicate mesh name on line ~D" lineno)
               (ppcre:register-groups-bind (command data)
                   (+wavefront-command-scanner+ line)
                 (unless (string= command "o")
                   (error "Invalid command on line ~D" lineno))
                 (setq name data)))))
    (assert name () "No mesh name in '~A'" obj-file)
    name))

(defun wavefront-obj-load (obj-file)
  (let ((name)
        (vertices        ;(vector (vec x y z)...)
         (make-array 10 :adjustable t :fill-pointer 0))
        (normals         ;(vector (vec x y z)...)
         (make-array 10 :adjustable t :fill-pointer 0))
        (faces  ;(vector (array x y z)...)
         (make-array 10 :adjustable t :fill-pointer 0))
        (uv
         (make-array 10 :adjustable t :fill-pointer 0))
        (materials)
        (current-material))
    (with-open-file (stream obj-file :direction :input)
      (loop for line = (read-line stream nil nil)
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
                 ("f" (vector-push-extend (parse-wavefront-face line current-material lineno)
                                          faces))
                 ("mtllib"
                  (setq materials (append materials (wavefront-mtl-load data obj-file))))
                 ("usemtl" (setq current-material data))
                 ("vt"
                  (vector-push-extend (parse-float-sequence data) uv))
                 ("g"; TODO
                  )
                 ("s"; TODO
                  )
                 ("l"; TODO
                  )
                 (otherwise (error "Unrecognized command ~A on line ~D" command lineno))))
             (assert (or (null line) matched) ()
                     "Error on line ~D" lineno))))
    (let* ((material-index-hash (make-hash-table :test #'equal))
           (texture-properties
            (loop for i from 0 for (material . texture) in materials
               do (setf (gethash material material-index-hash) i)
               collect texture)))
      (make-mesh-data  :name (name-mangle name)
                       :vertex-vectors (array-cat 'double-float vertices)
                       :normal-vectors (array-cat 'double-float normals)
                       :uv-vectors (array-cat 'double-float uv)
                       :texture-properties texture-properties
                       :texture-indices (map-into (make-array (length faces) :element-type 'fixnum)
                                                  (lambda (f) (gethash (wavefront-obj-face-material f)
                                                                       material-index-hash))
                                                  faces)
                       :uv-indices (array-cat 'fixnum
                                              (loop for f across faces
                                                 collect (wavefront-obj-face-uv-index f)))
                       :vertex-indices (array-cat 'fixnum
                                                  (loop for f across faces
                                                     collect (wavefront-obj-face-vertex-index f)))
                       :normal-indices (array-cat 'fixnum
                                                  (loop for f across faces
                                                     collect (wavefront-obj-face-normal-index f)))))))


(defun wavefront-povray (obj-file output-file
                         &key
                           reload
                           (handedness :right)
                           (directory *robray-tmp-directory*))
  (let ((output-file (output-file output-file directory)))
    (if (or reload
            (not (probe-file output-file))
            (< (file-write-date output-file)
               (file-write-date obj-file)))
        ;; Convert
        (let* ((mesh-data (wavefront-obj-load obj-file))
               (name (mesh-data-name mesh-data)))
          (format t "~&povenc ~A" obj-file)
          (output (pov-declare (mesh-data-name mesh-data)
                               (pov-mesh2 :mesh-data mesh-data :handedness handedness))
                  output-file)
          name)
        ;; Cached, extract name only
        (progn
          ;(format t "~&pov cached ~A" obj-file)
          (wavefront-obj-name obj-file)))))

(defun wavefront-convert (source-file output-file
                          &key
                            reload
                            (mesh-up-axis "Z")
                            (mesh-forward-axis "Y"))
  (if (or reload
          (not (probe-file output-file))
          (< (file-write-date output-file)
             (file-write-date source-file)))
      ;; load
      (let* ((args (list "blender" "-b" "-P"
                         (find-script "meshconv")
                         "--"
                         source-file
                         "-o" output-file
                         (format nil "--up=~A" mesh-up-axis)
                         (format nil "--forward=~A" mesh-forward-axis))))
        (format t "~&~{~A~^ ~}" args)
        (ensure-directories-exist output-file)
        (multiple-value-bind (output error-output status)
            (uiop:run-program args)
          (declare (ignore output error-output))
          (assert (zerop status) () "mesh conversion failed")))
      ;; cache
      (progn
        ;(format t "~&obj cached ~A" output-file)
        ))
  (values))

(defun mesh-povray (mesh-file
                    &key
                      reload
                      (mesh-up-axis "Z")
                      (mesh-forward-axis "Y")
                      (directory *robray-tmp-directory*))
  ;; Maybe convert
  (labels ((handle-obj (obj-file output-file)
             (let ((name (wavefront-povray obj-file output-file
                                           :reload reload
                                           :directory directory)))
               (values (name-mangle name) output-file)))
           (handle-dae (dae-file output-file)
             (convert dae-file output-file))
           (convert (source-file output-file)
             (let ((obj-file (pov-cache-file (rope source-file ".obj")
                                             directory)))
               (wavefront-convert source-file obj-file
                                  :reload  reload
                                  :mesh-up-axis mesh-up-axis
                                  :mesh-forward-axis mesh-forward-axis)
               (handle-obj obj-file output-file ))))
    (let* ((file-type (string-downcase (file-type mesh-file)))
           (file-basename (file-basename mesh-file))
           (file-dirname (file-dirname mesh-file))
           (output-file (format-pathname "povray/~A/~A.inc"
                                         file-dirname file-basename)))
    (string-case file-type
      ("obj" (handle-obj mesh-file output-file))
      ("stl"
       (let ((dae1 (format-pathname "~A/~A.dae" file-dirname file-basename))
             (dae2 (format-pathname "~A/~A.DAE" file-dirname file-basename)))
         (cond ((probe-file dae1) (handle-dae dae1 output-file))
               ((probe-file dae2) (handle-dae dae2 output-file))
               (t (convert mesh-file output-file)))))
      ("dae"
       (handle-dae mesh-file output-file))
      (otherwise (error "Unknown file type: ~A" file-type))))))
