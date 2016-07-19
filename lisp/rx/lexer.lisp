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
        (skip-regex (when skip-regex `(:sequence :start-anchor
                                                 (:greedy-repetition 1 nil
                                                                     ,(ensure-regex-tree skip-regex)))))
        (handlers (loop for (regex type-or-handler token) in token-regexes
                     collect
                       (if (functionp type-or-handler)
                           type-or-handler
                           (let* ((type type-or-handler)
                                  (token (or token type)))
                             (lambda (string start end)
                               (declare (ignore string start))
                               (values (or token type) type end)))))))
    (%make-lexer :token-scanner (ppcre:create-scanner big-regex :multi-line-mode t)
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
