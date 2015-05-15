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
  scanner
  regex
  handlers)

(defun make-lexer (token-regexes)
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
                               (declare (ignore string))
                               (values type type start end)))))))
    (%make-lexer :scanner (ppcre:create-scanner big-regex)
                 :regex big-regex
                 :handlers (make-array (length token-regexes)
                                        :initial-contents handlers))))

(defun lexer-lex (lexer string start
                  &key
                    (match :first))
  (unless (>= start (length string))
    (multiple-value-bind (r-start end reg-start reg-end)
        (ppcre:scan (lexer-scanner lexer) string :start start)
      (declare (ignore end))
      (assert (= r-start start))
      (let ((i (ecase match
                 (:first (loop for k across reg-start
                            for i from 0
                            until k
                            finally (return i))))))
        (assert (and i (aref reg-start i)))
        (funcall (aref (lexer-handlers lexer) i)
                 string (aref reg-start i) (aref reg-end i))))))
