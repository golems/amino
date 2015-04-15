(in-package :robray)

(defvar *collada-dom*)

;;; Collada is right-handed

;;; TODO:
;;; - asset/unit
;;; - asset/up_axis

(defun collada-load (file)
  (cxml:parse-file file (cxml-dom:make-dom-builder)))

(defun collada-node-singleton (node tag-name)
  (let ((children (dom:get-elements-by-tag-name node tag-name)))
    (assert (= 1 (length children)))
    (elt children 0)))


(defun collada-text-split (node)
  (let ((children (dom:child-nodes node)))
    (assert (= 1 (length children)))
    (let ((text (dom:data (elt children 0))))
      (ppcre:split " " text))))

(defun collada-node-integers (node)
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
  (assert (eql #\# (aref id 0)))
  (gethash (subseq id 1)
           id-map))

(defun collada-group-list (list stride)
  (loop for x = list then (nthcdr stride x)
     while x
     collect (subseq x 0 stride)))

(defun collada-parse-source (dom node)
  (declare (ignore dom))
  (assert (string= (dom:tag-name node) "source"))
  (let* ((array-node (collada-node-singleton node "float_array"))
         (array-values (collada-node-floats array-node))
         (technique-node (collada-node-singleton node "technique_common"))
         (accessor-node (collada-node-singleton technique-node "accessor"))
         (stride (if (dom:has-attribute accessor-node "stride")
                     (parse-integer (dom:get-attribute accessor-node "stride"))
                     1)))
    (assert (= 3 stride))
    (collada-group-list array-values stride)))

(defun collada-parse-vertices (dom node)
  (assert (string= (dom:tag-name node) "vertices"))
  (let ((input (collada-node-singleton node "input")))
    (assert (and (dom:has-attribute input "semantic")
                 (string= (dom:get-attribute input "semantic")
                          "POSITION")))
    (collada-source-parse dom input)))


(defun collada-parse-input (dom node)
  (assert (string= (dom:tag-name node) "input"))
  (collada-source-parse dom node))

(defun collada-parse (dom node)
  (let ((tag (dom:tag-name node)))
    (cond
      ((string= tag "vertices")
       (collada-parse-vertices dom node))
      ((string= tag "source")
       (collada-parse-source dom node))
      ((string= tag "input")
       (collada-parse-input dom node))
      (t (error "Unknown tag type: ~A" tag)))))


(defun collada-id-parse (dom id)
  (collada-parse dom (collada-lookup dom id)))

(defun collada-source-parse (dom node)
  (assert (dom:has-attribute node "source"))
  (collada-id-parse dom (dom:get-attribute node "source")))

(defun collada-povray-mangle (collada-identifier)
  (substitute #\_ #\- collada-identifier))

(defun collada-povray-geometry (dom geometry-node)
  (let* ((mesh  (collada-node-singleton geometry-node "mesh"))
         (polylist  (collada-node-singleton mesh "polylist"))
         (prim  (collada-node-integers (collada-node-singleton polylist "p")))
         (vcount  (collada-node-integers (collada-node-singleton polylist "vcount")))
         (inputs (dom:get-elements-by-tag-name polylist "input"))
         (vertices)
         (normals))
    ;; check vcount
    (dolist (c vcount)
      (assert (= 3 c) () "Cannot handle non-triangular polygons."))
    ;; Parse inputs
    (loop for input across inputs
       for semantic = (dom:get-attribute input "semantic")
       for value = (collada-parse dom input)
       do
         (cond ((string= semantic "VERTEX")
                (setq vertices value))
               ((string= semantic "NORMAL")
                (setq normals value))
               (t (error "Unknown input 'semantic' attribute: ~A" semantic))))
    ;; Parse Prim
    (let ((vertex-indices (collada-group-list (loop for rest = prim then (cddr rest)
                                                 while rest
                                                 collect (car rest))
                                              3))
          (normal-indices (collada-group-list (loop for rest = (cdr prim) then (cddr rest)
                                                 while rest
                                                 collect (car rest))
                                              3)))
      (assert (dom:has-attribute geometry-node "name"))
      (pov-define (collada-povray-mangle (dom:get-attribute geometry-node "name"))
                  (pov-mesh2 :vertex-vectors (map 'list #'pov-float-vector vertices)
                             :normal-vectors (map 'list #'pov-float-vector normals)
                             :face-indices (map 'list #'pov-integer-vector vertex-indices)
                             :normal-indices (map 'list #'pov-integer-vector normal-indices)
                             )))))

               ;(map 'list (lambda (x) (apply #'pov-vector x)) normals)
    ;(print (list vertices normals))))


(defun collada-povray (&key (dom *collada-dom*))
  ;; Meshes
  (labels ((nameref (dom name)
             (dom:get-elements-by-tag-name dom name))
           (nameref-singleton (dom name)
             (let ((tags (dom:get-elements-by-tag-name dom name)))
               (assert (= 1 (length tags)))
               (elt tags 0))))
    (let* ((library-geometry (nameref-singleton dom "library_geometries"))
           (geometries (nameref library-geometry "geometry"))
           )
      (pov-sequence
       (loop for g across geometries
          collect (collada-povray-geometry dom g))))))


         ;; for mesh = (nameref-singleton g "mesh")
         ;; for polylist = (nameref-singleton mesh "polylist")
         ;; for prim = (collada-node-floats (nameref-singleton polylist "p"))
         ;; ;; TODO: check vcount is always 3 (triangles)
         ;; for vcount = (collada-node-integers (nameref-singleton polylist "vcount"))
         ;; collect prim))))
