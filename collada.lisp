(in-package :robray)

(defvar *collada-dom*)

;;; Collada is right-handed

;;; TODO:
;;; - asset/unit
;;; - asset/up_axis

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; General Collada Parsing ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun collada-load (file)
  "Create the DOM for a Collada file."
  (setq *collada-dom* (dom-load file)))

(defun collada-node-text (node)
  "Return NODE's text."
  (let ((children (dom:child-nodes node)))
    (assert (= 1 (length children)))
    (dom:data (elt children 0))))

(defun collada-text-split (node)
  "Split NODE's text by spaces."
  (ppcre:split " " (collada-node-text node)))

(defun collada-node-integers (node)
  "Parse NODE's text into a list of integers."
  (map 'list #'parse-integer (collada-text-split node)))

(defun collada-node-floats (node)
  (map 'list #'parse-float (collada-text-split node)))


(defun collada-xml-id-map (dom)
  (let ((hash (make-hash-table :test #'equal)))
    (labels ((visit (node)
               (when (and (dom:element-p node)
                          (dom:has-attribute node "id"))
                 (setf (gethash (dom:get-attribute node "id") hash)
                       node))
               (map nil #'visit (dom:child-nodes node))))
      (visit dom))
    hash))

(defun collada-lookup (dom id &optional (id-map (collada-xml-id-map dom)))
  (let ((id (if (eql #\# (aref id 0)) ;; eat a leading hash
                (subseq id 1)
                id)))
    (gethash id id-map)))

(defun collada-group-list (list stride)
  "Group flat list LIST into a list of lists of length STRIDE."
  (loop for x = list then (nthcdr stride x)
     while x
     collect (subseq x 0 stride)))

(defun collada-parse-source (dom node)
  (declare (ignore dom))
  (assert (string= (dom:tag-name node) "source"))
  (let* ((array-node (dom-select-tag node "float_array" :direct t :singleton t))
         (array-values (collada-node-floats array-node))
         (technique-node (dom-select-tag node "technique_common" :direct t :singleton t))
         (accessor-node (dom-select-tag technique-node "accessor" :direct t :singleton t))
         (stride (if (dom:has-attribute accessor-node "stride")
                     (parse-integer (dom:get-attribute accessor-node "stride"))
                     1)))
    (assert (= 3 stride))
    (collada-group-list array-values stride)))

(defun collada-parse-vertices (dom node)
  (assert (string= (dom:tag-name node) "vertices"))
  (let ((input (dom-select-tag node "input" :direct t :singleton t)))
    (assert (and (dom:has-attribute input "semantic")
                 (string= (dom:get-attribute input "semantic")
                          "POSITION")))
    (collada-attr-id-parse dom input "source")))

(defun collada-parse-float (dom node)
  (declare (ignore dom))
  (assert (string= (dom:tag-name node) "float"))
  (parse-float (collada-node-text node)))


(defun collada-parse-color (dom node)
  (declare (ignore dom))
  (assert (string= (dom:tag-name node) "color"))
  (collada-node-floats node))

(defun collada-parse-input (dom node)
  (assert (string= (dom:tag-name node) "input"))
  (collada-attr-id-parse dom node "source"))

(defstruct collada-effect
  emission
  ambient
  diffuse
  specular
  shininess
  index-of-refraction)

(defun collada-parse-effect (dom node)
  (assert (string= (dom:tag-name node) "effect"))
  (let* ((profile (dom-select-tag node "profile_COMMON" :direct t :singleton t))
         (technique (dom-select-tag profile "technique" :direct t :singleton t))
         (phong (dom-select-tag technique "phong" :direct t :singleton t))
         (result (make-collada-effect)))
    (labels ((color-value (node)
               (collada-parse-color dom (dom-select-tag node "color" :direct t :singleton t)))
             (float-value (node)
               (collada-parse-float dom (dom-select-tag node "float" :direct t :singleton t))))
      (loop for child across (dom:child-nodes phong)
         when (dom:element-p child)
         do
           (string-case (dom:tag-name child)
              ("emission"
               (setf (collada-effect-emission result)
                     (color-value child)))
              ("ambient"
               (setf (collada-effect-ambient result)
                     (color-value child)))
              ("diffuse"
               (setf (collada-effect-diffuse result)
                     (color-value child)))
              ("specular"
               (setf (collada-effect-specular result)
                     (color-value child)))
              ("shininess"
               (setf (collada-effect-shininess result)
                     (float-value child)))
              ("index_of_refraction"
               (setf (collada-effect-index-of-refraction result)
                     (float-value child)))
              (otherwise
               (format *error-output* "Unknown effect tag: ~A" child)))))
    result))


(defun collada-parse-material (dom node)
  (assert (string= (dom:tag-name node) "material"))
  (let ((instance-effect (dom-select-tag node "instance_effect" :direct t :singleton t)))
    (collada-attr-id-parse dom instance-effect "url")))


(defun collada-parse (dom node)
  (let ((tag (dom:tag-name node)))
    (string-case (dom:tag-name node)
      ("vertices"
       (collada-parse-vertices dom node))
      ("source"
       (collada-parse-source dom node))
      ("input"
       (collada-parse-input dom node))
      ("effect"
       (collada-parse-effect dom node))
      ("color"
       (collada-parse-color dom node))
      ("float"
       (collada-parse-float dom node))
      ("material"
       (collada-parse-material dom node))
      (otherwise
       (error "Unknown tag type: ~A" tag)))))

(defun collada-id-parse (dom id)
  (collada-parse dom (collada-lookup dom id)))

(defun collada-attr-id-parse (dom node attribute)
  (assert (dom:has-attribute node attribute))
  (collada-parse dom (collada-lookup dom (dom:get-attribute node attribute))))

(defun collada-geometry-name (dom)
  "Find the name of the single geometry defined in this collada file"
  (name-mangle (dom-select-path dom '("COLLADA" "library_geometries" "geometry" "@name")
                                   :singleton t)))

;;;;;;;;;;;;;;;;;;;;;;;;;
;;; POVRAY CONVERSION ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;



;; TODO: Put text list before face_indices
;; TODO: Add texture index to each face
;; TODO: Collect vertices from multiple poly counts
(defun collada-povray-geometry (dom geometry-node)
  (let* ((mesh  (dom-select-tag geometry-node "mesh" :singleton t))
         (polylists  (dom-select-tag mesh "polylist" :direct t))
         (inputs (dom-select-path mesh '("polylist" "input")))
         (input-vertex (fold (lambda (set input)
                               (if (and (dom:has-attribute input "semantic")
                                        (string-equal (dom:get-attribute input "semantic")
                                                      "VERTEX")
                                        (dom:has-attribute input "source"))
                                   (tree-set-insert set (dom:get-attribute input "source"))
                                   set))
                             (make-tree-set #'string-compare)
                             inputs))
         (vertex-source (if (not (= 1 (tree-set-count input-vertex)))
                            (error "Cannot convert ~D source arrays to povray format" (tree-set-count input-vertex))
                            (car (tree-set-list input-vertex))))
         (vertices (collada-id-parse dom vertex-source))
         (input-normal (fold (lambda (set input)
                               (if (and (dom:has-attribute input "semantic")
                                        (string-equal (dom:get-attribute input "semantic")
                                                      "NORMAL")
                                        (dom:has-attribute input "source"))
                                   (tree-set-insert set (dom:get-attribute input "source"))
                                   set))
                             (make-tree-set #'string-compare)
                             inputs))
         (normal-source (if (not (= 1 (tree-set-count input-normal)))
                            (error "Cannot convert ~D source arrays to povray format" (tree-set-count input-normal))
                            (car (tree-set-list input-normal))))
         (normals (collada-id-parse dom normal-source))
         (textures (loop for p in polylists
                      for material-id = (dom:get-attribute p "material")
                      when (not (zerop (length material-id)))
                      collect
                        (collada-povray-material dom (collada-lookup dom material-id))))
         (polylist-prims (loop for p in polylists
                            for prim = (collada-node-integers (dom-select-tag p "p" :direct t :singleton t))
                            for vcount = (collada-node-integers (dom-select-tag p "vcount" :direct t :singleton t))
                            collect (progn
                                      (dolist (c vcount)
                                        (assert (= 3 c) () "Cannot handle non-triangular polygons."))
                                      prim)))
         (faces (loop for p in polylists
                   for i from 0
                   for prim in polylist-prims
                   for vertex-indices = (collada-group-list (loop for rest = prim then (cddr rest)
                                                               while rest
                                                               collect (car rest))
                                                            3)
                   nconc (progn
                           (loop for x in vertex-indices
                              nconc (if textures
                                        (list (pov-integer-vector x)
                                              (pov-value i))
                                        (list (pov-integer-vector x)))))))
         (normal-indices (loop for p in polylists
                            for prim  in polylist-prims
                            for normal-indices = (collada-group-list (loop for rest = (cdr prim) then (cddr rest)
                                                                        while rest
                                                                        collect (car rest))
                                                                     3)
                            nconc
                            normal-indices))
         )
    (assert (or (zerop (length textures))
                (= (length textures) (length polylists))))
    ;(print normal-indices)
    (pov-declare (name-mangle (dom:get-attribute geometry-node "name"))
                 (pov-mesh2 :vertex-vectors (map 'list #'pov-float-vector-right vertices)
                            :face-indices faces
                            :texture-list textures
                            :normal-vectors (map 'list #'pov-float-vector-right normals)
                            :normal-indices (map 'list #'pov-integer-vector normal-indices)
                            ))))

    ;;      (print vertex-source)
    ;;      (print faces)
    ;;      textures))

    ;;      (prim  (collada-node-integers (dom-select-tag polylist "p" :direct t :singleton t)))
    ;;      (vcount  (collada-node-integers (dom-select-tag polylist "vcount" :direct t :singleton t)))
    ;;      (inputs (dom:get-elements-by-tag-name polylist "input"))
    ;;      (vertices)
    ;;      (normals))
    ;; ;; check vcount
    ;; (dolist (c vcount)
    ;;   (assert (= 3 c) () "Cannot handle non-triangular polygons."))
    ;; ;; Parse inputs
    ;; (loop for input across inputs
    ;;    for semantic = (dom:get-attribute input "semantic")
    ;;    for value = (collada-parse dom input)
    ;;    do
    ;;      (cond ((string= semantic "VERTEX")
    ;;             (setq vertices value))
    ;;            ((string= semantic "NORMAL")
    ;;             (setq normals value))
    ;;            (t (error "Unknown input 'semantic' attribute: ~A" semantic))))
    ;; ;; Parse Prim
    ;; (let ((vertex-indices (collada-group-list (loop for rest = prim then (cddr rest)
    ;;                                              while rest
    ;;                                              collect (car rest))
    ;;                                           3))
    ;;       (normal-indices (collada-group-list (loop for rest = (cdr prim) then (cddr rest)
    ;;                                              while rest
    ;;                                              collect (car rest))
    ;;                                           3)))
    ;;   (assert (dom:has-attribute geometry-node "name"))
    ;;   (pov-declare (name-mangle (dom:get-attribute geometry-node "name"))
    ;;                (pov-mesh2 :vertex-vectors (map 'list #'pov-float-vector-right vertices)
    ;;                           :normal-vectors (map 'list #'pov-float-vector-right normals)
    ;;                           :face-indices (map 'list #'pov-integer-vector vertex-indices)
    ;;                           :normal-indices (map 'list #'pov-integer-vector normal-indices)
    ;;                          :modifiers modifiers
    ;;                          )))))

               ;(map 'list (lambda (x) (apply #'pov-vector x)) normals)
    ;(print (list vertices normals))))


;; #S(COLLADA-EFFECT
;;    :EMISSION (0.0d0 0.0d0 0.0d0 1.0d0)
;;    :AMBIENT (0.0d0 0.0d0 0.0d0 1.0d0)
;;    :DIFFUSE (0.64d0 0.64d0 0.64d0 1.0d0)
;;    :SPECULAR (0.5d0 0.5d0 0.5d0 1.0d0)
;;    :SHININESS 50.0d0
;;    :INDEX-OF-REFRACTION 1.0d0)

(defun collada-povray-material (dom node)
  (let ((material (collada-parse dom node))
        (finishes)
        (pigments))
    (labels ((avg-rgb (item)
               (pov-float (/ (+ (first item)
                                (second item)
                                (third item))
                             3d0)))
             (add-finish (name finish)
               (push (pov-item name finish)
                     finishes)))
    (when (collada-effect-ambient material)
      (add-finish "ambient" (pov-rgb (collada-effect-ambient material))))
    (when (collada-effect-diffuse material)
      (let ((diffuse (collada-effect-diffuse material)))
        (add-finish "diffuse" (avg-rgb diffuse))
        (push (pov-item "color" (pov-rgb diffuse))
              pigments)))
    (when (collada-effect-specular material)
      (add-finish "specular" (avg-rgb (collada-effect-specular material))))

      ;; TODO: emission, index-of-refraction, shininess
    ;; (when (collada-effect-shininess material)
    ;;   (add-finish "reflection" (pov-float (/ (collada-effect-shininess material)
    ;;                                          100d0))))
    (let ((textures))
      (when pigments (push (pov-pigment pigments)
                           textures))
      (when finishes (push (pov-finish finishes)
                           textures))
      (pov-texture textures)))))

(defun collada-povray (&key
                         (dom *collada-dom*)
                         (directory *robray-tmp-directory*)
                         file)
  (let* ((geometry-node (dom-select-path dom '("COLLADA" "library_geometries" "geometry")  :singleton t))
         (result (collada-povray-geometry dom geometry-node)))
    (if file
        (output result file :directory directory)
        result)))




;; (defun collada-povray-visual-scene (dom node)
;;   (assert (string= "library_visual_scenes" (dom:tag-name node)))
;;   (loop
;;      for child-node across (dom:get-elements-by-tag-name node "node")
;;      for instance-geometry = (dom:get-elements-by-tag-name child-node "instance_geometry")
;;      unless (zerop (length instance-geometry))
;;      collect
;;        (progn
;;          (assert (= 1 (length instance-geometry)))
;;          (let ((instance-geometry (aref instance-geometry 0)))
;;            (assert (dom:has-attribute instance-geometry "url"))
;;            (let* ((geometry-node (collada-lookup dom (dom:get-attribute instance-geometry "url")))
;;                   (instance-material (dom:get-elements-by-tag-name instance-geometry "instance_material"))
;;                   (modifiers))
;;              ;; maybe set material
;;              (unless (zerop (length instance-material))
;;                (let ((instance-material (aref instance-material 0)))
;;                  (when (dom:has-attribute instance-material "target")
;;                    (push (collada-povray-material dom
;;                                                   (collada-lookup dom (dom:get-attribute instance-material
;;                                                                                          "target")))
;;                          modifiers))))
;;              (collada-povray-geometry dom geometry-node modifiers))))))
