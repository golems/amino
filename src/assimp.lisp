(in-package :robray)

;; Assimp does support the normal_indices feature of POV-ray,
;; so the meshes it imports are much larger than necessary.
(defun assimp-pov (input-file output-file)
  (labels ((vector-append (vector-list)
             (apply #'concatenate 'list vector-list))
           (avg-rgb (item)
             (pov-float (/ (+ (vecref item 0)
                              (vecref item 1)
                              (vecref item 2))
                           3d0))))
  (let* ((root (ai:import-into-lisp input-file
                                    :processing-flags '(:ai-process-gen-normals
                                                        :ai-process-optimize-meshes
                                                        :ai-process-join-identical-vertices)))
         (scene (ai:root-node root))
         (nodes (ai:children scene))
         (name (loop for node across nodes
                  while (zerop (length (ai:meshes node)))
                  finally (return (ai:name node))))
         (meshes (ai:meshes root))
         (materials (ai:materials root))
         (vertices (vector-append (loop for mesh across meshes
                                     collect
                                       (map 'vector #'pov-float-vector-right
                                            (ai:vertices mesh)))))
         (faces (vector-append (loop for mesh across meshes
                                  for material-idx = (ai:material-index mesh)
                                  collect (loop for face across (ai:faces mesh)
                                             for face-node = (pov-integer-vector face)
                                             append (if (zerop (length materials))
                                                        (list face-node)
                                                        (list face-node material-idx))))))
         (normals (vector-append (loop for mesh across meshes
                                   collect
                                     (map 'vector (lambda (thing)
                                                    (let ((x (vec-x thing))
                                                          (y (vec-y thing))
                                                          (z (vec-z thing)))
                                                      (assert (not (zerop (sqrt (+ (* x x)
                                                                                   (* y y)
                                                                                   1
                                                                                   (* z z)))))))
                                                    (pov-float-vector-right thing))
                                          (ai:normals mesh)))))
         (textures (loop for material across materials
                      for ambient = (gethash "$clr.ambient" material)
                      for diffuse = (gethash "$clr.diffuse" material)
                      ;for emmisive = (gethash "$clr.emmisive" material)
                      for specular = (gethash "$clr.specular" material)
                      ;for opacity = (gethash "$clr.opacity" material)
                      for shininess = (gethash "$mat.shininess" material)
                      for finishes = (append  (when ambient (list (pov-item "ambient"
                                                                            (pov-rgb ambient))))
                                              (when diffuse (list (pov-item "diffuse"
                                                                            (avg-rgb diffuse))))
                                              (when specular (list (pov-item "specular"
                                                                             (avg-rgb specular)))))
                      for pigments = (append (when diffuse (list (pov-item "color"
                                                                           (pov-rgb diffuse)))))
                      collect (pov-texture (append
                                            (when pigments (list (pov-pigment pigments))
                                            (when finishes (list (pov-finish finishes))))))))
         )
    (output (pov-declare (name-mangle name)
                         (pov-mesh2 :vertex-vectors vertices
                                    :face-indices faces
                                    :texture-list textures
                                    ;:normal-vectors normals
                                        ;:normal-indices (loop for i below (length faces)
                                        ;collect (pov-integer-vector (list i i i)))
                                    ))
            output-file)
    ;nodes
    ;; ;(map 'list #'ai:name nodes)
    ;; ;nodes
    ;; (gethash "$clr.ambient" (elt materials 0))
    name
    )))
