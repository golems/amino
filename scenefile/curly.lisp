(in-package :robray)

(defstruct curly-block
  type
  name
  statements)

(defparameter +curly-scanners+
  (map 'list (lambda (thing)
               (destructuring-bind (regex &rest rest) thing
                 (cons (ppcre:create-scanner (format nil "^(~A)" regex)) rest)))
       (list (list "{" #\{)
             (list "}" #\})
             (list "\\(" #\()
             (list "\\)" #\))
             (list "\\[" #\[)
             (list "\\]" #\])
             (list "," #\,)
             (list ";" #\;)
             (list "\\\"[^\\\"]*\\\"" :string
                   (lambda (string start end)
                     (subseq string (1+ start) (1- end))))
             (list "[a-zA-Z][a-zA-Z0-9_]*" :identifer
                   (lambda (string start end)
                     (let ((token (subseq string start end)))
                       (cond ((string= token "frame")
                              :frame)
                             ((string= token "geometry")
                              :geometry)
                             ((string= token "class")
                              :class)
                             (t token)))))
             (list "-?(([0-9]+\\.[0-9]*)|(\\.[0-9]+))" :float
                   (lambda (string start end)
                     (parse-float (subseq string start end))))
             (list "-?[0-9]+" :integer
                   (lambda (string start end)
                     (parse-integer string :start start :end end))))))

(defparameter +curly-comment-regex+
  (let ((whitespace "\\s")
        (line-comment "#|//).*(\\n|$")
        (block-comment "/\\*[^(/\\*)]*\\*/"))
    (ppcre:create-scanner (format nil "^((~A)|(~A)|(~A))*"
                                  whitespace
                                  line-comment
                                  block-comment))))

(defun curly-next-token (string start)
  "Returns (VALUES TOKEN-VALUE TOKEN-TYPE STRING-START STRING-END)"
  (multiple-value-bind (comment-start comment-end)
      (ppcre:scan +curly-comment-regex+ string :start start)
    (assert (= start comment-start))
    (let ((start comment-end))
      (loop for (scanner type value-function) in +curly-scanners+
         do (multiple-value-bind (start end)
                (ppcre:scan scanner string :start start)
              (when start
                (return (values type
                                (if value-function
                                    (funcall value-function string start end)
                                    type)
                                start
                                end))))))))


(defun curly-parse-string (string)
  (let ((start 0))
    (labels ((next ()
               (multiple-value-bind (type token t-start end)
                   (curly-next-token string start)
                 ;; (when type
                 ;;   (format t "~&token: ~A `~A'" token (subseq string t-start end)))
                 (setq start end)
                 (values type token)))
             (start ()
               ;(print 'start)
               (multiple-value-bind (type token) (next)
                 (cond
                   ((eq token :frame)
                    (cons (frame)
                          (start)))
                   ((null type)
                    nil)
                   (t (error "Unknown type: ~A" type)))))
             (body ()
               ;(print 'body)
               (let ((type (next)))
                 (assert (eq #\{ type)))
               (body-first-item ))
             (frame ()
               (multiple-value-bind (type token) (next)
                 (assert (eq :identifer type))
                 (make-curly-block :type :frame
                                   :name token
                                   :statements (body))))
             (geometry ()
               (make-curly-block :type :geometry
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
                   ((eq type :identifer) ; beginning of block statement
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
  (let ((string
         (with-open-file (stream pathname)
           (with-output-to-string (string)
             (loop for char = (read-char stream nil nil)
                while char
                do (write-char char string))))))
    (curly-parse-string string)))


(defun load-curly-scene (pathname)
  (let ((curly (curly-parse-file pathname))
        (frames)
        (geoms))

    (labels ((add-prop (hash stmt)
               (assert (= 2 (length stmt)))
               (setf (gethash (first stmt) hash)
                     (second stmt)))
             (add-block (cb parent)
               (ecase (curly-block-type cb)
                 (:frame (add-frame cb parent))
                 (:geometry (add-object cb parent))))
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
