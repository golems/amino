(in-package :robray)

(defun scene-file-type (filename)
  (let ((filetype (file-type filename)))
    (if filetype
        (string-case (string-downcase filetype)
          ("urdf" :urdf)
          (("robray" "curly") :curly)
          (otherwise (error "Unrecognized scene file suffix: ~A" filetype)))
        :curly)))

(defun load-scene-file (filename
                        &key
                          type
                          reload-meshes
                          (mesh-up-axis "Z")
                          (mesh-forward-axis "Y"))

  (let* ((filename (rope-string (rope filename)))
         (type (or type (scene-file-type filename))))
    (ecase type
      (:urdf (urdf-parse (urdf-resolve-file filename)
                         :reload-meshes reload-meshes
                         :mesh-up-axis mesh-up-axis
                         :mesh-forward-axis mesh-forward-axis))
      (:curly (load-curly-scene filename :reload-meshes reload-meshes))
      (:moveit (load-moveit-scene filename
                                  :reload-meshes reload-meshes)))))
