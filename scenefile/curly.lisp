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
                  (:regex "(#|//)[\\n$]*(\\n|$)")
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
                 (cond
                   ((eq token :frame)
                    (cons (frame)
                          (start)))
                   ((null type)
                    nil)
                   (t (parse-error type "frame or EOF")))))
             (body ()
               ;(print 'body)
               (let ((type (next)))
                 (assert (eq #\{ type)))
               (body-first-item ))
             (frame ()
               (multiple-value-bind (type token) (next)
                 (assert (eq :identifier type))
                 (make-curly-block :type :frame
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
                 (cond
                   ((eq type #\})
                    nil) ; empty block
                   ((eq token :geometry)
                    (cons (geometry)
                          (body-first-item)))
                   ((eq type :identifier) ; beginning of block statement
                    (cons (body-statement (list token))
                          (body-first-item)))
                   (t (error "Unknown statement type: ~A" type)))))
             (body-statement (items)
               (declare (type list items))
               ;(format t "~&body statement: ~A" (reverse items))
               (multiple-value-bind (type token) (next)
                 (declare (ignore type))
                 (case token
                   (#\; (reverse items))
                   (#\{
                    (assert (= 1 (length items)))
                    (make-curly-block :name (car items)
                                      :type (car items)
                                      :line line
                                      :statements (body-first-item)))

                   (#\[
                    (body-statement (cons (array nil) items)))
                   (otherwise (body-statement (cons token items))))))
             (array (elements)
               (multiple-value-bind (type token) (next)
                 (case type
                   (#\] (apply #'vec (reverse elements)))
                   (otherwise (array-delim (cons token elements))))))
             (array-delim (elements)
               (multiple-value-bind (type token) (next)
                 (declare (ignore token))
                 (ecase type
                   (#\] (apply #'vec (reverse elements)))
                   (#\, (array elements))))))

      (start))))

(defun curly-parse-file (pathname)
  (curly-parse-string (read-file-into-string pathname)
                      pathname))


(defun load-curly-scene (pathname)
  (let ((curly (curly-parse-file pathname))
        (frames)
        (geoms))

    (labels ((add-prop (hash stmt)
               (assert (= 2 (length stmt)))
               (setf (gethash (first stmt) hash)
                     (second stmt)))
             (add-block (cb parent)
               (case (curly-block-type cb)
                 (:frame (add-frame cb parent))
                 (:geometry (add-object cb parent))
                 (otherwise (error "Parse error ~A:~D, unrecognized block type ~A"
                                   pathname
                                   (curly-block-line cb)
                                   (curly-block-type cb)))))

             (add-frame (cb parent)
               (let ((name (curly-block-name cb))
                     (properties (make-hash-table :test #'equal)))
                 (dolist (stmt (curly-block-statements cb))
                   (etypecase stmt
                     (null)
                     (cons (string-case (car stmt)
                             ("axis" (add-prop properties stmt))
                             ("quaternion"  (add-prop properties stmt))
                             ("translation"  (add-prop properties stmt))
                             ("parent"  (add-prop properties stmt))
                             ;("color"  (add-prop properties stmt))
                             ;("alpha"  (add-prop properties stmt))
                             ("type"  (add-prop properties stmt))
                             ("offset"  (add-prop properties stmt))))
                     (curly-block
                      (add-block stmt name))))
                 (let ((frame (string-case (gethash "type" properties "fixed")
                                ("fixed" (scene-frame-fixed (gethash "parent" properties parent)
                                                            name
                                                            :tf (tf* (gethash "quaternion" properties)
                                                                     (gethash "translation" properties)))))))
                   (push frame frames))))
             (add-object (cb parent)
               (let ((properties (make-hash-table :test #'equal)))
                 (dolist (stmt (curly-block-statements cb))
                   (etypecase stmt
                     (null)
                     (cons (string-case (car stmt)
                             ("shape" (add-prop properties stmt))
                             ("dimension"  (add-prop properties stmt))
                             ("color"  (add-prop properties stmt))
                             ("alpha"  (add-prop properties stmt))))))
                 (let ((v
                        ;; TODO: collision
                        (make-scene-visual :color (gethash "color" properties)
                                           :alpha (gethash "alpha" properties 1d0)
                                           :geometry
                                           (string-case (gethash "shape" properties)
                                             ("box"
                                              (scene-box (gethash "dimension" properties)))
                                             (t (error "Unknown shape: ~A" (gethash "shape" properties)))))))
                   (push (cons (gethash "parent" properties parent)
                               v)
                         geoms)))))
      (dolist (c curly)
        (add-block c nil)))
    (let ((sg (scene-graph frames)))
      (fold (lambda (sg g)
              (scene-graph-add-visual sg (car g) (cdr g)))
            sg
            geoms))))
