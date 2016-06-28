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

;;(declaim (optimize (speed 3) (safety 0)))

;;(cffi:defcfun aa-rx-wf-obj-create rx-wf-obj-t)

(cffi:defcfun aa-rx-wf-parse rx-wf-obj-t
  (filename :string))

(defmacro def-wf-obj-array (accessor)
  `(cffi:defcfun ,accessor :void
     (obj rx-wf-obj-t)
     (ptr :pointer)
     (n :pointer)))

(def-wf-obj-array aa-rx-wf-obj-get-vertices)
(def-wf-obj-array aa-rx-wf-obj-get-normals)
(def-wf-obj-array aa-rx-wf-obj-get-vertex-indices)
(def-wf-obj-array aa-rx-wf-obj-get-normal-indices)
(def-wf-obj-array aa-rx-wf-obj-get-uv-indices)
(def-wf-obj-array aa-rx-wf-obj-get-texture-indices)


(cffi:defcfun aa-rx-wf-obj-mtl-count size-t
     (obj rx-wf-obj-t))

(cffi:defcfun aa-rx-wf-obj-get-mtl :string
  (obj rx-wf-obj-t)
  (i size-t))

(cffi:defcfun aa-rx-wf-obj-material-count size-t
     (obj rx-wf-obj-t))

(cffi:defcfun aa-rx-wf-obj-get-material-name :string
  (obj rx-wf-obj-t)
  (i size-t))

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
  (let ((mtl-file (file-resolve mtl-file (file-dirname obj-file)))
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
             (let ((line (strip-hash-comment line))
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
                     ("Ke" (rgb-prop :emission))
                     ;("Ka" (prop :ambient '(.1 .1 .1))
                     ("Kd" (rgb-prop :diffuse))
                     ("Ni" (float-prop :index-of-refraction))
                     ("d" (prop :alpha  (parse-float data)))
                     ("map_Kd"
                      ;; TODO: local path names
                      (let* ((source-file (file-resolve data (file-dirname mtl-file)))
                             (dir (file-dirname source-file))
                             (base (file-basename source-file))
                             (ext (pov-image-map-type (string-downcase (file-type source-file))))
                             (cache-file (pov-cache-file (format-pathname "~A/~A.~A" dir base ext))))
                        (ensure-directories-exist cache-file)
                        (uiop/stream:copy-file source-file cache-file)
                        (prop :image-map cache-file)))
                     ("illum" ;TODO
                      )
                     (otherwise (error "Unrecognized MTL command ~A" command)))))
               (assert (or (null line) matched)))))
      (add-current))
    material-alist))



(defun wavefront-obj-name (obj-file)
  (let ((name)
        (warned))
    (with-open-file (stream obj-file :direction :input)
      (loop for line = (read-line stream nil nil)
         for lineno from 0
         while line
         when (eq #\o (aref line 0))
         do
           (if name
               (progn (unless warned
                        (warn "Duplicate mesh name `~A:~D'" obj-file lineno)
                        (setq warned t))
                      (setq name (file-basename obj-file)))
               (ppcre:register-groups-bind (command data)
                   (+wavefront-command-scanner+ line)
                 (unless (string= command "o")
                   (error "Invalid command on line ~D" lineno))
                 (setq name data)))))
    (or name (file-basename obj-file))))

(let ((scanner (ppcre:create-scanner "^(\\S+)\\s+(\\S+)\\s+(\\S+)\\s*$" )))
  (defun parse-vec3 (string &key (start 0))
    (multiple-value-bind (start end s e) (ppcre:scan scanner string :start start)
      (declare (ignore start end))
      (loop for i below 3
         collect (parse-float (subseq string (aref s i) (aref e i)))))))

(defun wavefront-obj-load (obj-file)
  (format t "~&  LOAD ~A~%" obj-file)
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
      (loop with warned = nil
         for line = (read-line stream nil nil)
         for lineno from 0
         while line
         do
           (let ((line (strip-hash-comment line))
                 (matched nil))
             (ppcre:register-groups-bind (command data)
                 (+wavefront-command-scanner+ line)
               (setq matched t)
               (string-case command
                 ("o"  (if (null name)
                           (setq name data)
                           (progn (unless warned
                                    (warn "`~A': Multiple objects not implemented"
                                          obj-file)
                                    (setq warned t))
                                  (setq name (file-basename obj-file)))))
                 ("v"
                  (vector-push-extend (parse-vec3 data)
                                      vertices))
                 ("vn"
                  (vector-push-extend (parse-vec3 data) normals))
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
      (make-mesh-data  :name (name-mangle (or name (file-basename obj-file)))
                       :file obj-file
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
(defun wavefront-obj-load2 (filename )
  ;(declare (optimize (speed 3) (safety 0)))
  (let* ((obj (aa-rx-wf-parse (rope-string filename)))
         ;; Materials
         (materials-alist (loop for i below (aa-rx-wf-obj-mtl-count obj)
                             for mtl-file = (aa-rx-wf-obj-get-mtl obj i)
                             append (wavefront-mtl-load mtl-file filename)))
         (materials-array (make-array (aa-rx-wf-obj-mtl-count obj))))
    (dotimes (i (length materials-array))
      ;; TODO: error check
      (setf (aref materials-array i)
            (assoc (aa-rx-wf-obj-get-material-name obj i) materials-alist :test #'string=)))
    (print materials-array)
    ;; Extract data arrays
    (labels ((get-array (accessor foreign-type lisp-type)
               (cffi:with-foreign-objects ((n 'size-t)
                                           (v :pointer))
                 (funcall accessor obj v n)
                 (amino-ffi::foreign-vector-dup (cffi:mem-ref v :pointer)
                                                (cffi:mem-ref n 'size-t)
                                                foreign-type lisp-type)))
             (get-fix-array (accessor)
               ;; TODO: We could elide a copy here by directly converting
               ;; foreign memory to fixnums
               (let* ((v32 (get-array accessor :int32 '(signed-byte 32)))
                      (n (length v32))
                      (vf (make-fnvec (length v32))))
                 (dotimes (i n)
                   (setf (aref vf i) (aref v32 i)))
                 vf)))
      (let ((vertices (get-array #'aa-rx-wf-obj-get-vertices :double 'double-float))
            (normals (get-array #'aa-rx-wf-obj-get-normals :double 'double-float))
            (vertex-indices  (get-fix-array #'aa-rx-wf-obj-get-vertex-indices))
            (normal-indices  (get-fix-array #'aa-rx-wf-obj-get-normal-indices))
            (uv-indices  (get-fix-array #'aa-rx-wf-obj-get-uv-indices))
            (texture-indices  (get-fix-array #'aa-rx-wf-obj-get-texture-indices)))
        (values)
       ;; vertex-indices
        (let* (;(material-index-hash (make-hash-table :test #'equal))
               (texture-properties
                (loop for (material . texture) across materials-array
                   collect texture)))
          (make-mesh-data  :name (file-basename filename)
                           :file filename
                           :vertex-vectors vertices
                           :normal-vectors normals
                           ;; TODO: uv vectors
                           ;; :uv-vectors (array-cat 'double-float uv)
                           :texture-properties texture-properties
                           :texture-indices texture-indices
                           :uv-indices uv-indices
                           :vertex-indices vertex-indices
                           :normal-indices normal-indices))))))

(defun mesh-regen-p (reload source-file output-file)
  (or reload
      (not (probe-file output-file))
      (< (file-write-date output-file)
         (file-write-date source-file))))

(defun wavefront-convert (source-file output-file
                          &key
                            reload
                            (mesh-up-axis "Z")
                            (mesh-forward-axis "Y"))
  "Convert something to a wavefront OBJ file."
  (when (mesh-regen-p reload source-file output-file)
    ;; load
    (let* ((args (list "blender" "-b" "-P"
                       (find-script "meshconv")
                       "--"
                       source-file
                       "-o" output-file
                       (format nil "--up=~A" mesh-up-axis)
                       (format nil "--forward=~A" mesh-forward-axis))))
      (format t "~&  ~{~A~^ ~}~%" args)
      (ensure-directories-exist output-file)
      ;; check blender return value
      (multiple-value-bind (output status)
          (capture-program-output args)
        (unless (and (zerop status)
                     (probe-file output-file))
          (error "Blender could not convert `~A' to Wavefront OBJ.~%Does your Blender have support for ~A files?~%OUTPUT:~%~A"
                 source-file (file-type source-file)
                 output)))))
  (values)
)

(defun mesh-obj-tmp (source-file directory)
  (pov-cache-file (rope source-file ".obj")
                  directory))

(defun load-mesh (mesh-file
                  &key
                    reload
                    (directory *robray-tmp-directory*)
                    (mesh-up-axis "Z")
                    (mesh-forward-axis "Y"))
  (labels ((handle-obj (obj-file)
             (wavefront-obj-load obj-file))
           (convert (source-file)
             (let ((obj-file (mesh-obj-tmp source-file directory)))
               (wavefront-convert source-file obj-file
                                  :reload  reload
                                  :mesh-up-axis mesh-up-axis
                                  :mesh-forward-axis mesh-forward-axis)
               (handle-obj obj-file))))
    (let ((file-type (string-downcase (file-type mesh-file))))
      (string-case file-type
        ("obj" (handle-obj mesh-file))
        ("stl" (convert mesh-file))
        ("dae" (convert mesh-file))
        (otherwise (error "Unknown file type: ~A" file-type))))))



(defun mesh-povray (mesh-file
                    &key
                      reload
                      (mesh-up-axis "Z")
                      (mesh-forward-axis "Y")
                      (handedness :right)
                      (directory *robray-tmp-directory*))
  (unless (probe-file mesh-file)
    (error "Mesh file not found"))
  (let* ((file-type (string-downcase (file-type mesh-file)))
         (file-basename (file-basename mesh-file))
         (file-dirname (file-dirname mesh-file))
         (src-obj-p (string= (string-downcase file-type) "obj"))
         (obj-file (if src-obj-p
                       mesh-file
                       (mesh-obj-tmp mesh-file directory)))
         (rel-output-file (format-pathname "povray/~A/~A.inc"
                                       file-dirname file-basename))
         (abs-output-file (output-file rel-output-file directory)))
    (if (or (mesh-regen-p reload mesh-file abs-output-file)
            (not (probe-file obj-file)))
        ;; Regenerate
        (let* ((mesh-data (load-mesh mesh-file
                                     :reload  reload
                                     :directory directory
                                     :mesh-up-axis mesh-up-axis
                                     :mesh-forward-axis mesh-forward-axis))
               (name (mesh-data-name mesh-data)))
          (format t "~&  POVENC ~A~%" obj-file)
          (output (pov-declare (mesh-data-name mesh-data)
                               (pov-mesh2 :mesh-data mesh-data :handedness handedness))
                  abs-output-file)
          (values (name-mangle name) rel-output-file))
        ;; Don't regenerate
        (values (name-mangle (wavefront-obj-name obj-file))
                rel-output-file))))

 ;; (labels ((handle-obj (obj-file output-file)
 ;;             (let ((name (wavefront-povray obj-file output-file
 ;;                                           :reload reload
 ;;                                           :directory directory)))
 ;;               (values (name-mangle name) output-file)))
 ;;           (handle-dae (dae-file output-file)
 ;;             (convert dae-file output-file))
 ;;           (convert (source-file output-file)
 ;;             (let ((obj-file (pov-cache-file (rope source-file ".obj")
 ;;                                             directory)))
 ;;               (wavefront-convert source-file obj-file
 ;;                                  :reload  reload
 ;;                                  :mesh-up-axis mesh-up-axis
 ;;                                  :mesh-forward-axis mesh-forward-axis)
 ;;               (handle-obj obj-file output-file ))))

;; (defun wavefront-povray (obj-file output-file
;;                          &key
;;                            reload
;;                            (handedness :right)
;;                            (directory *robray-tmp-directory*))
;;   (let ((output-file (output-file output-file directory)))
;;     (if (or reload
;;             (not (probe-file output-file))
;;             (< (file-write-date output-file)
;;                (file-write-date obj-file)))
;;         ;; Convert
;;         (let* ((mesh-data (wavefront-obj-load obj-file))
;;                (name (mesh-data-name mesh-data)))
;;           (format t "~&povenc ~A" obj-file)
;;           (output (pov-declare (mesh-data-name mesh-data)
;;                                (pov-mesh2 :mesh-data mesh-data :handedness handedness))
;;                   output-file)
;;           name)
;;         ;; Cached, extract name only
;;         (progn
;;           ;(format t "~&pov cached ~A" obj-file)
;;           (wavefront-obj-name obj-file)))))
