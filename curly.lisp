(in-package :robray)

(defun curly-eat-blank (string start)
  (loop for i from start below (length string)
     while (amino::char-isblank (aref string i))
     finally (return i)))

(defparameter +curly-regex-tokens+
  (list (list "{" #\{)
        (list "}" #\})
        (list "\\(" #\()
        (list "\\)" #\))
        (list "\\\"[^\\\"]*\\\"" :string
              (lambda (string start end)
                (subseq string (1+ start) (1- end))))
        (list "[a-zA-Z][a-zA-Z0-9_]*" :identifer
              (lambda (string start end)
                (let ((token (subseq string start end)))
                  (cond ((string= token "frame")
                         :frame)
                        ((string= token "class")
                         :class)
                        (t token)))))
        (list "[0-9]+\\.[0-9]*" :float
              (lambda (string start end)
                (parse-float (subseq string start end))))
        (list "[0-9]+" :integer
              (lambda (string start end)
                (parse-integer string :start start :end end)))))


(defparameter +curly-comment-regex+
  (let ((whitespace "\\s")
        (line-comment "#|//).*(\\n|$")
        (block-comment "/\\*[^(/\\*)]*\\*/"))
    (ppcre:create-scanner (format nil "^((~A)|(~A)|(~A))*"
                                  whitespace
                                  line-comment
                                  block-comment))))

(defparameter +curly-scanners+
  (loop for thing in +curly-regex-tokens+
     for regex = (car thing)
     for rest = (cdr thing)
     collect (cons (ppcre:create-scanner (concatenate 'string "^" regex)) rest)))

(defun curly-next-token (string start)
  "Returns (VALUES TOKEN-VALUE TOKEN-TYPE END)"
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
                                end))))))))


(defun curly-parse-string (string)
  (let ((start 0))
    (labels ((next ()
               (multiple-value-bind (type token end)
                   (curly-next-token string start)
                 (setq start end)
                 (values type token)))
             (start ()
               (multiple-value-bind (type token) (next)
                 (when type
                   (cons token (body)))))
             (body ()
               (let ((type (next)))
                 (assert (eq #\{ type)))
               (body-first-item))
             (body-first-item ()
               (multiple-value-bind (type token) (next)
                 (if (eq type #\})
                     nil
                     (body-next-item token))))
             (body-next-item (head-token)
               (multiple-value-bind (type token) (next)
                 (case type
                   (#\} (list head-token))
                   (#\{ (cons (cons head-token (body-first-item))
                              (body-first-item)))
                   (otherwise (cons head-token (body-next-item token)))))))

      (start))))

(defun curly-parse-file (pathname)
  (let ((string
         (with-open-file (stream pathname)
           (with-output-to-string (string)
             (loop for char = (read-char stream nil nil)
                while char
                do (write-char char string))))))
    (curly-parse-string string)))
