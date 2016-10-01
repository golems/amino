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

(defstruct curly-block
  type
  name
  line
  statements)


(defstruct curly-def
  name
  value)

(defstruct curly-allow-collision
  frame-0
  frame-1)

;;; Scanner element:
;;;    (list regex (or type (lambda (string start end))))
;;;    If the second element is a function, its result is
;;;    (values (token-value token-type token-end))

(defparameter +integer-regex+
  '(:regex "-?\\d+"))

(defparameter +float-regex+
  (let ((exponent '(:regex "[eEdDsS]-?\\d+")))
    `(:sequence (:regex "-?")
                (:alternation (:sequence (:alternation (:regex "\\d+\\.\\d*")
                                                       (:regex "\\.\\d+"))
                                         (:greedy-repetition 0 1 ,exponent))
                              (:sequence (:regex "\\d+")
                                         ,exponent)))))

(defparameter +number-regex+
  `(:alternation ,+integer-regex+
                 ,+float-regex+))

(defparameter +curly-token-regex+
  `(
    ;; Punctuation
    ("{" #\{)
    ("}" #\})
    ("\\(" #\()
    ("\\)" #\))
    ("\\[" #\[)
    ("\\]" #\])
    ("," #\,)
    (";" #\;)
    ;; Keywords
    ("frame(?!\\w)" :frame)
    ("geometry(?!\\w)" :geometry)
    ("class(?!\\w)" :class)
    ("object(?!\\w)" :object)
    ("include(?!\\w)" :include)
    ("allow_collision(?!\\w)" :allow-collision)
    ("def(?!\\w)" :def)
    ("π|pi(?!\\w)"
     ,(lambda (string start end)
              (declare (ignore start string))
              (values pi :float end)))
    ("deg(?!\\w)"
     ,(lambda (string start end)
              (declare (ignore start string))
              (values (/ pi 180d0) :float end)))
    ;("isa(?!\\w)" :isa)
    ;; identifiers
    ("[a-zA-Z][a-zA-Z0-9_\\-]*"
     ,(lambda (string start end)
              (values (subseq string start end) :identifier end)))

    ;; Non floating point integer
    ;; uses positive lookahead to find a non-floating-point suffix
    ("-?\\d+(?=[^\\d.eEdD]|$)"
     ,(lambda (string start end)
              (values (parse-integer string :start start :end end)
                      :integer end)))

    ;; Floating point number
    (;"-?((\\d+\\.\\d*)|(\\.\\d+))"
     ,+float-regex+
     ,(lambda (string start end)
              (values (parse-float (subseq string start end))
                      :float
                      end)))
    ("\\*" :binop *)
    ("\\+" :binop +)
    ("/" :binop /)
    ("\\-" -)
    ;; string
    ("\\\"[^\\\"]*\\\""
     ,(lambda (string start end)
              (values (subseq string (1+ start) (1- end))
                      :string
                      end)))))


(defparameter +curly-comment-regex+
  `(:alternation :whitespace-char-class
                 ;; line comment
                 (:regex "(#|(//)).*?(\\n|$)")
                 ;; block comment
                 (:sequence "/*"
                            (:greedy-repetition
                             0 nil
                             (:alternation (:regex "[^*]")
                                           (:regex "\\*(?!/)")))
                                        ;(:regex "(.|\\n)*")
                            "*/")))

(defparameter +curly-lexer+ (make-lexer +curly-token-regex+
                                        +curly-comment-regex+))

(defun curly-next-token (string start line)
  "Returns (VALUES TOKEN-VALUE TOKEN-TYPE STRING-START STRING-END)"
  ;; check end
  (if (>= start (length string))
     (values nil nil nil nil nil)
     ;; find token
     (lexer-lex +curly-lexer+ string start line)))

(defun curly-tokenize (string)
  (loop with start = 0
     for line from 1
     for (value type t-start end) = (multiple-value-list (curly-next-token string start line))
     while value
     do (setq start end)
     collect (print (list value type (subseq string t-start end)))))

(defun curly-tokenize-file (file)
  (curly-tokenize (read-file-into-string file)))

(defun curly-parse-string (string &optional pathname)
  (let ((start 0)
        (line 1))
    (labels ((parse-error (found wanted)
               (error "Parse error ~A:~D, found ~A, wanted ~A"
                      (or pathname "line") line found wanted))
             (next (&optional valid-types)
               (multiple-value-bind (value type t-start end t-line)
                   (curly-next-token string start line)
                 (declare (ignore t-start))
                 ;; (when value
                 ;;   (format t "~&token: ~A `~A'" value (subseq string t-start end)))
                 (setq start end
                       line t-line)
                 (when valid-types
                   (unless (find type (ensure-list valid-types))
                     (parse-error type valid-types)))
                 (values type value)))
             (next-token (&optional valid-types)
               (multiple-value-bind (type token) (next valid-types)
                 (declare (ignore type))
                 token))
             (start ()
               ;(print 'start)
               (multiple-value-bind (type token) (next)
                 (when type
                   (case type
                     ((:frame :object :class)
                      (cons (named-block token)
                            (start)))
                     (:allow-collision
                      (cons (allow-collision)
                            (start)))
                     (:include
                      (append (include)
                              (start)))
                     (:def (cons (def)
                                 (start)))
                     (otherwise (parse-error type "frame, object, or EOF"))))))
             (allow-collision ()
               (let* ((frame-0 (next-token '(:string :identifier)))
                      (frame-1 (next-token '(:string :identifier))))
                 (next-token #\;)
                 (make-curly-allow-collision :frame-0 frame-0
                                             :frame-1 frame-1)))
             (def ()
               (multiple-value-bind (type token) (next)
                 (case type
                   (:identifier (def-value token))
                   (otherwise (parse-error type :identifier)))))
             (def-value (name)
               (multiple-value-bind (e type token) (expr)
                 (declare (ignore token))
                 (case type
                   (#\; (make-curly-def :name name :value e))
                   (otherwise (parse-error type #\;)))))
             (include ()
               (multiple-value-bind (type token) (next)
                 (case type
                   (:string
                    (let ((include-file (merge-pathnames (pathname token) pathname)))
                      (curly-parse-file include-file)))
                   (otherwise (parse-error type :string)))))
             (body ()
               ;(print 'body)
               (let ((type (next)))
                 (unless (eq #\{ type)
                   (parse-error type "{")))
               (body-first-item ))
             (named-block (block-type)
               (multiple-value-bind (type token) (next)
                 (let ((name (case type
                               (:identifier token)
                               (:string token)
                               (otherwise
                                (parse-error type "identifier or string")))))
                   (make-curly-block :type block-type
                                     :line line
                                     :name name
                                     :statements (body)))))
             (geometry ()
               (make-curly-block :type :geometry
                                 :line line
                                 :name nil
                                 :statements (body)))
             (body-first-item ()
               ;(print 'body-first)
               (multiple-value-bind (type token) (next)
                 (unless (eq type #\})
                   (cons (case type
                           (:geometry
                            (geometry))
                           ((:frame :object)
                            (named-block token))
                           (:identifier ; beginning of block statement
                            (body-statement (list token)))
                           (t (parse-error type "identifier, frame, object, geometry, or }")))
                     (body-first-item)))))
             (body-statement (items)
               (declare (type list items))
               ;(format t "~&body statement: ~A" (reverse items))
               (multiple-value-bind (type token) (next)
                 (case type
                   (#\; (reverse items))
                   ;; (#\{
                   ;;  (assert (= 1 (length items)))
                   ;;  (make-curly-block :name (car items)
                   ;;                    :type (car items)
                   ;;                    :line line
                   ;;                    :statements (body-first-item)))

                   (#\[
                    (body-statement (cons (array nil) items)))
                   ((:identifier :integer :float :string)
                    (body-statement (cons token items)))
                   (otherwise (parse-error type "a value")))))

             (expr ()
               (expr-val nil))
             (expr-result (e)
               (if (null (cdr e))
                   (car e)
                   (inexp-parse (reverse e))))
             (expr-next (e)
               (multiple-value-bind (type token) (next)
                 (case type
                   ((:binop -) (expr-val (cons token e)))
                   (otherwise (values (expr-result e)
                                      type token)))))
             (expr-val (e)
               (multiple-value-bind (type token) (next)
                 (case type
                   ((:float :integer :identifier) (expr-next (cons token e)))
                   (- (expr-val (list* '_  e)))
                   (#\( (expr-next (cons (expr-paren) e)))
                   (otherwise (values (expr-result e)
                                      type token)))))
             (expr-paren ()
               (multiple-value-bind (r type token) (expr)
                 (declare (ignore token))
                 (case type
                   (#\) r)
                   (otherwise (parse-error type ")")))))
             (array (elements)
               (multiple-value-bind (e type token) (expr)
                 (declare (ignore token))
                 ;(print e)
                 (let ((elements (if e
                                     (cons e elements)
                                     elements)))
                   (case type
                     (#\] (apply #'vector (reverse elements)))
                     (#\, (array elements))
                     (otherwise (parse-error type "] or ,"))))))
             ;; (array-delim (elements)
             ;;   (multiple-value-bind (type token) (next)
             ;;     (declare (ignore token))
             ;;     (case type
             ;;       (#\] (apply #'vec (reverse elements)))
             ;;       (#\, (array elements))
             ;;       (otherwise (parse-error type "} or ,")))))
             )

      (start))))

(defun curly-parse-file (pathname)
  (let ((pathname (rope-pathname pathname)))
    (curly-parse-string (read-file-into-string pathname)
                        pathname)))


(defun curly-eval (defs e)
  (labels ((recurse (args)
             (map 'list (lambda (e)
                          (curly-eval defs e))
                  args)))
    (etypecase e
      (number e)
      (string (if-let ((a (assoc e defs :test #'equal)))
                (cdr a)
                e))
      (list
       (destructuring-case e
         ((+ &rest args)
          (apply #'+ (recurse args)))
         ((- &rest args)
          (apply #'- (recurse args)))
         ((/ &rest args)
          (apply #'/ (recurse args)))
         ((* &rest args)
          (apply #'* (recurse args)))))
      (vector
       (apply #'vec (recurse e))))))


(defun load-curly-scene (pathname)
  (let ((curly (curly-parse-file pathname))
        (file-directory (file-dirname pathname))
        (frames)
        (geoms)
        (allowed-collisions)
        (defs)
        (classes (make-tree-map #'string-compare)))
    ;; TODO: namespaces
    (labels ((add-def (def)
               (push (cons (curly-def-name def)
                           (curly-eval defs (curly-def-value def)))
                     defs))
             (add-prop (properties stmt)
               (assert (= 2 (length stmt)))
               (acons (first stmt) (curly-eval defs (second stmt))
                      properties))
             (add-prop-bool (properties stmt)
               (assert (= 2 (length stmt)))
               (acons (first stmt) (not (zerop (curly-eval defs (second stmt))))
                      properties))
             (get-prop (properties key &optional default)
               (let ((assoc (assoc key properties :test #'equal)))
                 (if assoc (cdr assoc) default)))
             (get-orientation (properties)
               (let ((q (get-prop properties "quaternion"))
                     (rpy (get-prop properties "rpy")))
                 (cond
                   (q q)
                   (rpy (aa:euler-rpy rpy))
                   (t aa::+TF-QUAT-IDENT+))))
             (make-prop () nil)
             (get-class (stmt)
               (assert (= 2 (length stmt)))
               (multiple-value-bind (class present)
                   (tree-map-find classes (second stmt))
                 (unless present (error "Class ~A not found" (second stmt)))
                 class))
             (class-properties (properties class-properties)
               (apply #'append
                      properties
                      class-properties))
             (add-thing (thing)
               (etypecase thing
                 (curly-block (add-block thing nil))
                 (curly-allow-collision (push thing allowed-collisions))
                 (curly-def (add-def thing))))
             (add-block (cb parent)
               ;(print 'add-block)
               (case (curly-block-type cb)
                 (:frame (add-frame cb parent))
                 (:geometry (add-geom cb parent))
                 (:class
                  (unless (null parent)
                    (error "Cannot nest class in ~A" parent))
                  (add-class cb))
                 (otherwise (error "Parse error ~A:~D, unrecognized block type ~A"
                                   pathname
                                   (curly-block-line cb)
                                   (curly-block-type cb)))))
             (add-class (cb)
               ;(print 'adding-class)
               (let ((name (curly-block-name cb))
                     (properties (make-prop))
                     (parent-classes))
                 (when (tree-map-contains classes name)
                   (error "Duplicate class ~A" name))
                 (dolist (stmt (curly-block-statements cb))
                   (typecase stmt
                     (null)
                     (cons (string-case (car stmt)
                             (("quaternion" "rpy"
                               "translation"
                               "type" "axis" "offset" "parent"
                               "shape" "mesh" "dimension" "delta" "thickness" "color" "alpha" "specular")
                              (setq properties (add-prop properties stmt)))
                             (("visual" "collision")
                              (setq properties (add-prop-bool properties stmt)))
                             ("isa"
                              (setq properties (add-prop properties stmt))
                              (push (get-class stmt) parent-classes))
                             (otherwise (error "Parse error, unrecognized property ~A in class ~A"
                                               (car stmt) name))))
                     (t (error "Parse error, unrecognized statement in class ~A"
                               name))))
                 (setf (tree-map-find classes name)
                       (class-properties properties parent-classes))))
             (add-frame (cb parent)
               (let ((name (curly-block-name cb))
                     (properties (make-prop))
                     (parent-classes))
                 (dolist (stmt (curly-block-statements cb))
                   (etypecase stmt
                     (null)
                     (cons (string-case (car stmt)
                             (("axis" "quaternion" "rpy" "translation" "type" "offset" "variable")
                              (setq properties
                                    (add-prop properties stmt)))
                             ("parent"
                              (unless (null parent)
                                (error "Cannot reparent frame ~A" name))
                              (setq properties (add-prop properties stmt)))
                             ("isa"
                              (push (get-class stmt) parent-classes))
                             (otherwise (error "Unknown property '~A' in frame ~A"
                                               (car stmt) name))))
                     (curly-block
                      (add-block stmt name))))
                 (insert-frame name (class-properties properties (reverse parent-classes)) parent)))
             (insert-frame (name properties parent)
               (let* ((frame-type (get-prop properties "type" "fixed"))
                      (tf (tf* (get-orientation properties)
                               (get-prop properties "translation")))
                      (parent (get-prop properties "parent" parent))
                      (frame (if (string= frame-type "fixed")
                                 (scene-frame-fixed parent name :tf tf)
                                 (let ((axis (get-prop properties "axis"))
                                       (offset (get-prop properties "offset" 0d0)))
                                   (funcall (string-case frame-type
                                              ("prismatic" #'scene-frame-prismatic)
                                              ("revolute" #'scene-frame-revolute)
                                              (otherwise (error "Unhandled frame type ~A in frame ~A"
                                                                frame-type name)))
                                            parent name
                                            :configuration-name (or (get-prop properties "variable")
                                                                    name)
                                            :axis axis
                                            :configuration-offset offset
                                            :tf tf)))))
                 (push frame frames)))
             (add-geom (cb parent)
               (let ((properties (make-prop))
                     (classes))
                 (dolist (stmt (curly-block-statements cb))
                   (etypecase stmt
                     (null)
                     (cons (string-case (car stmt)
                             (("shape" "color" "alpha" "specular" "mesh"
                                       "dimension" "delta" "thickness" "radius" "height" "start-radius" "end-radius" "scale")
                              (setq properties (add-prop properties stmt)))
                             (("visual" "collision")
                              (setq properties (add-prop-bool properties stmt)))
                             ("isa"
                              (setq properties (add-prop properties stmt))
                              (push (get-class stmt) classes))
                             (otherwise (error "Unknown property '~A' in geometry at line ~D"
                                               (car stmt)
                                               (curly-block-line cb)))))))
                 (insert-geom (class-properties properties classes)
                              parent)))
             (property-classes (properties)
               (let ((list (loop for (key . value) in properties
                              when (string= key "isa")
                              collect value)))
                 (when list
                   (apply #'tree-set #'string-compare list))))
             (property-options (properties)
               (list*
                (cons :visual (get-prop properties "visual" t))
                (cons :collision (get-prop properties "collision" t))
                (loop
                   for name in '("color" "alpha" "scale" "specular")
                   for kw in   '(:color  :alpha :scale :specular)
                   for value = (get-prop properties name)
                   when value collect (cons kw value))))
             (insert-geom (properties parent)
               (let* ((shape-prop (get-prop properties "shape"))
                      (mesh-prop (get-prop properties "mesh"))
                      (options (property-options properties))
                      (geometry
                       (cond
                         (shape-prop (string-case shape-prop
                                       ("box"
                                        (scene-geometry-box options (get-prop properties "dimension")))
                                       ("sphere"
                                        (scene-geometry-sphere options (get-prop properties "radius")))
                                       ("grid"
                                        (scene-geometry-grid options
                                                             :dimension (get-prop properties "dimension")
                                                             :delta (get-prop properties "delta")
                                                             :width (get-prop properties "thickness")))
                                       ("cylinder"
                                        (scene-geometry-cylinder options :height (get-prop properties "height")
                                                                 :radius (get-prop properties "radius")))
                                       (otherwise (error "Unknown shape: ~A" (get-prop properties "shape")))))
                         (mesh-prop
                          (scene-geometry-mesh options
                                               (make-scene-mesh :source-file
                                                                (file-resolve mesh-prop file-directory))))
                         (t (error "No shape or mesh in ~A" parent))))
                      ;(geometry (scene-geometry shape (property-options properties)))
                      (parent (get-prop properties "parent" parent)))
                 (setf (scene-geometry-type geometry) (property-classes properties))
                 (push (cons parent geometry) geoms))))
      ;; Evaluate Statements
      (dolist (c curly)
        (add-thing c))
      ;; Create Scene Graph
      (let ((scene-graph (scene-graph (make-scene-graph :files (make-scene-graph-files (namestring pathname)))
                                      frames)))
        ;; Add Geometry
        (setq scene-graph (fold (lambda (sg g)
                                  (scene-graph-add-geometry sg (car g) (cdr g)))
                                scene-graph geoms))
        ;; Allow Collisions
        (setq scene-graph (fold (lambda (sg ac)
                                  (scene-graph-allow-collision sg
                                                               (curly-allow-collision-frame-0 ac)
                                                               (curly-allow-collision-frame-1 ac)))
                                scene-graph allowed-collisions))
        scene-graph))))
