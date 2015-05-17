(in-package :robray)

(defstruct curly-block
  type
  name
  line
  statements)

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
    ;("isa(?!\\w)" :isa)
    ;; identifiers
    ("[a-zA-Z][a-zA-Z0-9_]*"
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
    ;; string
    ("\\\"[^\\\"]*\\\""
     ,(lambda (string start end)
              (values (subseq string (1+ start) (1- end))
                      :string
                      end)))))


(defparameter +curly-comment-regex+
  `(:greedy-repetition
    0 nil
    (:alternation :whitespace-char-class
                  ;; line comment
                  (:regex "(#|(//)).*?(\\n|$)")
                  ;; block comment
                  (:sequence "/*" (:regex "(.*?)") "*/"))))

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

(defun curly-parse-string (string &optional name)
  (let ((start 0)
        (line 1))
    (labels ((parse-error (found wanted)
               (error "Parse error ~A:~D, found ~A, wanted ~A"
                      (or name "line") line found wanted))
             (next ()
               (multiple-value-bind (value type t-start end t-line)
                   (curly-next-token string start line)
                 (declare (ignore t-start))
                 ;; (when value
                 ;;   (format t "~&token: ~A `~A'" value (subseq string t-start end)))
                 (setq start end
                       line t-line)
                 (values type value)))
             (start ()
               ;(print 'start)
               (multiple-value-bind (type token) (next)
                 (when type
                   (case type
                     ((:frame :object :class)
                      (cons (named-block token)
                            (start)))
                     (otherwise (parse-error type "frame, object, or EOF"))))))
             (body ()
               ;(print 'body)
               (let ((type (next)))
                 (unless (eq #\{ type)
                   (parse-error type "{")))
               (body-first-item ))
             (named-block (block-type)
               (multiple-value-bind (type token) (next)
                 (unless (eq :identifier type)
                   (parse-error type "identifier"))
                 (make-curly-block :type block-type
                                   :line line
                                   :name token
                                   :statements (body))))
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
             (array (elements)
               (multiple-value-bind (type token) (next)
                 (case type
                   (#\] (apply #'vec (reverse elements)))
                   (otherwise (array-delim (cons token elements))))))
             (array-delim (elements)
               (multiple-value-bind (type token) (next)
                 (declare (ignore token))
                 (case type
                   (#\] (apply #'vec (reverse elements)))
                   (#\, (array elements))
                   (otherwise (parse-error type "} or ,"))))))

      (start))))

(defun curly-parse-file (pathname)
  (curly-parse-string (read-file-into-string pathname)
                      pathname))


(defun load-curly-scene (pathname)
  (let ((curly (curly-parse-file pathname))
        (frames)
        (geoms)
        (classes (make-tree-map #'string-compare)))
    ;; TODO: classes
    ;; TODO: include files
    ;; TODO: namespaces
    ;; TODO: affordances
    (labels ((add-prop (properties stmt)
               (assert (= 2 (length stmt)))
               (acons (first stmt) (second stmt)
                      properties))
             (get-prop (properties key &optional default)
               (let ((assoc (assoc key properties :test #'equal)))
                 (if assoc (cdr assoc) default)))
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
                     (properties (make-prop)))
                 (when (tree-map-contains classes name)
                   (error "Duplicate class ~A" name))
                 (dolist (stmt (curly-block-statements cb))
                   (typecase stmt
                     (null)
                     (cons (string-case (car stmt)
                             (("quaternion" "translation" "type" "axis" "offset" "parent"
                                            "shape" "dimension" "color" "alpha")
                              (setq properties (add-prop properties stmt)))
                             ("isa"
                              (push (get-class stmt) classes))
                             (otherwise (error "Parse error, unrecognized property ~A in class ~A"
                                               (car stmt) name))))
                     (t (error "Parse error, unrecognized statement in class ~A"
                               name))))
                 (setq classes
                       (tree-map-insert classes name properties))))
             (add-frame (cb parent)
               (let ((name (curly-block-name cb))
                     (properties (make-prop))
                     (classes))
                 (dolist (stmt (curly-block-statements cb))
                   (etypecase stmt
                     (null)
                     (cons (string-case (car stmt)
                             (("axis" "quaternion" "translation" "type" "offset")
                              (setq properties
                                    (add-prop properties stmt)))
                             ("parent"
                              (unless (null parent)
                                (error "Cannot reparent frame ~A" name))
                              (setq properties (add-prop properties stmt)))
                             ("isa"
                              (push (get-class stmt) classes))
                             (otherwise (error "Unknown property in frame ~A" name))))
                     (curly-block
                      (add-block stmt name))))
                 (insert-frame name (class-properties properties (reverse classes)) parent)))
             (insert-frame (name properties parent)
               (let* ((frame-type (get-prop properties "type" "fixed"))
                      (frame (string-case frame-type
                               ("fixed" (scene-frame-fixed (get-prop properties "parent" parent)
                                                           name
                                                           :tf (tf* (get-prop properties "quaternion")
                                                                    (get-prop properties "translation"))))
                               (otherwise (error "Unhandled frame type ~A in frame ~A" frame-type name)))))
                 (push frame frames)))
             (add-geom (cb parent)
               (let ((properties (make-prop))
                     (classes))
                 (dolist (stmt (curly-block-statements cb))
                   (etypecase stmt
                     (null)
                     (cons (string-case (car stmt)
                             (("shape" "dimension" "color" "alpha")
                              (setq properties (add-prop properties stmt)))
                             ("isa"
                              (push (get-class stmt) classes))
                             (otherwise (error "Unknown property in geometry at line ~D"
                                               (curly-block-line cb)))))))
                 (insert-geom (class-properties properties (reverse classes))
                              parent)))
             (insert-geom (properties parent)
               (let ((v ;; TODO: collision
                      (make-scene-visual :color (get-prop properties "color")
                                         :alpha (get-prop properties "alpha" 1d0)
                                         :geometry
                                         (string-case (get-prop properties "shape")
                                           ("box"
                                            (scene-box (get-prop properties "dimension")))
                                           (t (error "Unknown shape: ~A" (get-prop properties "shape")))))))
                 (push (cons (get-prop properties "parent" parent)
                             v)
                       geoms)))
             )
      (dolist (c curly)
        (add-block c nil)))
    (let ((sg (scene-graph frames)))
      (fold (lambda (sg g)
              (scene-graph-add-visual sg (car g) (cdr g)))
            sg
            geoms))))
