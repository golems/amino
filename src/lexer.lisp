(in-package :robray)

(defun ensure-regex-tree (regex)
  (etypecase regex
    (string (ppcre:parse-string regex))
    (cons regex)))

(defun strip-registers (tree)
  (etypecase tree
    (atom tree)
    (cons
     (destructuring-bind (head &rest body) tree
       (if (eq head :register)
           (progn (assert (= 1 (length body)))
                  (strip-registers (car body)))
           (cons head (map 'list #'strip-registers body)))))))

(defstruct (lexer (:constructor %make-lexer))
  token-scanner
  token-regex
  skip-scanner
  skip-regex
  handlers)

(defun make-lexer (token-regexes &optional skip-regex)
  (let ((big-regex `(:sequence
                     :start-anchor
                     (:alternation ,@(loop for (regex thing) in token-regexes
                                        collect (list :register
                                                      (strip-registers (ensure-regex-tree regex)))))))
        (handlers (loop for (regex type-or-handler) in token-regexes
                     collect
                       (let ((type type-or-handler))
                         (if (functionp type-or-handler)
                             type-or-handler
                             (lambda (string start end)
                               (declare (ignore string start))
                               (values type type end)))))))
    (%make-lexer :token-scanner (ppcre:create-scanner big-regex)
                 :token-regex big-regex
                 :skip-regex skip-regex
                 :skip-scanner (when skip-regex (ppcre:create-scanner skip-regex))
                 :handlers (make-array (length token-regexes)
                                        :initial-contents handlers))))

(defun lexer-lex (lexer string start line
                  &key
                    (match :first))
  (unless (>= start (length string))
    ;; Skip
    (let ((token-start (if-let ((skip-scanner (lexer-skip-scanner lexer)))
                         (multiple-value-bind (skip-start skip-end)
                             (ppcre:scan skip-scanner string :start start)
                           (if skip-start
                               (progn (assert (= start skip-start))
                                      skip-end)
                               start))
                         start)))
      ;; Match Token Rgex
      (unless (>= token-start (length string))
        (multiple-value-bind (r-start end reg-start reg-end)
            (ppcre:scan (lexer-token-scanner lexer) string :start token-start)
          (declare (ignore end))
          (assert (= r-start token-start))
          ;; Extract type and value
          (let ((i (ecase match
                     (:first (loop for k across reg-start
                                for i from 0
                                until k
                                finally (return i))))))
            (assert (and i (aref reg-start i)))
            (multiple-value-bind (value type end)
                (funcall (aref (lexer-handlers lexer) i)
                         string (aref reg-start i) (aref reg-end i))
              (values value type token-start end
                      (+ line (count #\Newline string :start start :end end))))))))))
