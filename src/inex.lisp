;; Copyright (c) 2011, Georgia Tech Research Corporation
;; Copyright (c) 2015, Rice University
;; All rights reserved.
;;
;; Redistribution and use in source and binary forms, with or without
;; modification, are permitted provided that the following conditions
;; are met:
;;
;; * Redistributions of source code must retain the above copyright
;;   notice, this list of conditions and the following disclaimer.
;;
;; * Redistributions in binary form must reproduce the above copyright
;;   notice, this list of conditions and the following disclaimer in
;;   the documentation and/or other materials provided with the
;;   distribution.
;;
;; * Neither the name of the copyright holder(s) nor the names of its
;;   contributors may be used to endorse or promote products derived
;;   from this software without specific prior written permission.
;;
;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
;; "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
;; LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
;; FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
;; COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
;; INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
;; (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
;; SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
;; HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
;; STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
;; OF THE POSSIBILITY OF SUCH DAMAGE.


(in-package :robray)


(defun inexp-precedence (op)
  (case op
      ((+ -) 1)
      ((* /) 2)
      ((^ **) 3)))

(defun inexp-left (op)
  (ecase op
    ((+ - * /) t)
    ((^ **) nil)))

(defun inexp-opp (k)
  (find k '(+ - * / ^ **)))

(defun inexp-rpn (list)
  (let (stack)
    (dolist (e list)
      (if (inexp-opp e)
          (let ((a (pop stack))
                (b (pop stack)))
            (cond
              ((find e '(+  *))
               (push (list e a b) stack))
              ((find e '(-  /))
               (push (list e b a) stack))
              ((find e '(^  **))
               (push (list 'expt b a) stack))
              (t (error "unknown op: ~A" e))))
          (push e stack)))
    (assert (and stack (null (cdr stack)))
            () "bad rpn stack: ~A" stack)
    (car stack)))

(defun inexp-shunt (list)
  (cond
    ((atom list) (list list))
    ((inexp-opp (cadr list))
      (let (out stack)
        (dolist (k list)
          (cond
            ((listp k) (push (inexp-parse k) out))
            ((inexp-opp k)
             (loop
                while (and stack (if (inexp-left k)
                                     (<= (inexp-precedence k)
                                         (inexp-precedence (car stack)))
                                     (< (inexp-precedence k)
                                        (inexp-precedence (car stack)))))
                do (push (pop stack) out))
             (push k stack))
            (t (push k out))))
        (dolist (k stack) (push k out))
        (reverse out)))
    (t (list (cons (car list) (mapcar #'inexp-parse (cdr list)))))))

(defun inexp-parse (list)
  (if (atom list)
      list
      (inexp-rpn (inexp-shunt list))))

(defmacro inexp (&rest list)
  (inexp-parse list))

;; (defun test-inexp ()
;;   (assert (= 2 (inexp 1 + 1)))
;;   (assert (= 7 (inexp 1 + 2 * 3)))
;;   (assert (= 19 (inexp 1 + 2 * 3 * 3)))
;;   (assert (= 257 (inexp 1 + 2 ^ 2 ^ 3)))
;;   (assert (= 6 (inexp 1 + (+ 2 3))))
;;   (assert (= 6 (inexp 1 + (+ 2 3))))
;;   (assert (= 9 (inexp 1 + (+ 2 (3 + 3)))))
;;   (assert (= 19 (inexp + 2 (5 + (+ 6 2)) 4)))
;;   )
