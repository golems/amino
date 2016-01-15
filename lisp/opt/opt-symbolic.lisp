;;;; -*- mode: lisp -*-
;;;;
;;;; Copyright (c) 2016, Rice University
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


(in-package :amino)


(defmacro with-constraint ((type terms value) constraint &body body)
  (with-gensyms (op fun)
    `(flet ((,fun (,type ,terms ,value)
              ,@body))
       (destructuring-bind (,type (,op &rest ,terms) ,value) ,constraint
         (assert (eq ,op '+))
         (,fun ,type ,terms (coerce ,value 'double-float))))))

(defmacro do-constraints ((type terms value &optional result) constraints &body body)
  (with-gensyms (c)
    `(dolist (,c ,constraints ,result)
       (with-constraint (,type ,terms ,value) ,c
         ,@body))))


(defmacro with-term ((factor variable) term &body body)
  (with-gensyms (op fun term-var)
    `(let ((,term-var ,term))
       (flet ((,fun (,factor ,variable)
                ,@body))
         (if (and (listp ,term-var)
                  (eq '* (car ,term-var)))
             (destructuring-bind (,op ,factor ,variable) ,term
               (declare (ignore ,op))
               (,fun (coerce ,factor 'double-float) ,variable))
             (,fun 1d0 ,term-var))))))


(defun opt-term (factor variable)
  `(* ,factor ,variable))

(defun opt-constraint (type terms value)
  `(,type (+ ,@terms) ,value))

(defstruct normalized-constraint
  (lower nil :type (or double-float null))
  (terms nil :type list)
  (upper nil :type (or double-float null)))

(defun normalized-constraint-bounds-p (normalized-constraint)
  (let ((terms (normalized-constraint-terms normalized-constraint)))
    (and terms (null (cdr terms)))))

(defun normalize-terms (terms)
  (let ((m (sycamore:make-tree-map #'sycamore-util:gsymbol-compare)))
    (dolist (term terms)
      (with-term (f v) term
        (let ((f1 (multiple-value-bind (f0 key present) (sycamore:tree-map-find m v)
                    (declare (ignore key))
                    (if present
                        (+ f f0)
                        f))))
        (sycamore:tree-map-insertf m v f1))))
    (sycamore:map-tree-map :inorder 'list #'cons m)))

(defun normalize-constraints (constraints)
  (let ((hash (make-hash-table :test #'equal)))
    (dolist (c constraints)
      (with-constraint (type terms value) c
        (let* ((terms (normalize-terms terms))
               (nc (or (gethash terms hash)
                       (setf (gethash terms hash)
                             (make-normalized-constraint :terms terms)))))
          (ecase type
                  (<= (setf (normalized-constraint-upper nc)
                            (if-let ((current (normalized-constraint-upper nc)))
                              (min current value)
                              value)))
                  (>= (setf (normalized-constraint-lower nc)
                            (if-let ((current (normalized-constraint-lower nc)))
                              (max current value)
                              value)))
                  (= (if-let ((current (normalized-constraint-lower nc)))
                       (unless (= current value)
                         (error "Unable to satisfy constraint ~A" c))
                       (setf (normalized-constraint-lower nc) value))
                     (if-let ((current (normalized-constraint-upper nc)))
                       (unless (= current value)
                         (error "Unable to satisfy constraint ~A" c))
                       (setf (normalized-constraint-upper nc) value)))))))
    (hash-table-values hash)))

(defun opt-variable-position (variables var)
  (position var variables :test #'equal))

(defun opt-variable-count (variables)
  (length variables))

(defun opt-constraint-bounds-p (constraint)
  (with-constraint (type terms value) constraint
    (declare (ignore type value))
    (and terms (null (cdr terms)))))

(defun opt-type-flip (type)
  (ecase type
    (= '=)
    (<= '>=)
    (>= '<=)))

(defun opt-constraints-equality-p (constraints)
  (dolist (c constraints)
    (unless (opt-constraint-bounds-p c)
      (with-constraint (type terms value) c
        (declare (ignore terms value))
        (unless (eq type '=)
          (return-from opt-constraints-equality-p nil)))))
  t)

(defun opt-variables (constraints objectives)
  (let ((hash-vars (make-hash-table :test #'equal)))
    (labels ((add-term (term)
               (with-term (factor variable) term
                 (declare (ignore factor))
                 (setf (gethash variable hash-vars) t))))
      (do-constraints (type terms value) constraints
        (declare (ignore value))
        (assert (find type '(= <= >=)))
        (map nil #'add-term terms))
      (map nil #'add-term objectives))
    (hash-table-keys hash-vars)))

(defun opt-bounds (variables constraints
                   &key
                     (default-lower 0d0)
                     (default-upper most-positive-double-float))
  (let* ((n (opt-variable-count variables))
         (has-lower (make-array n :element-type 'bit  :initial-element 0))
         (has-upper (make-array n :element-type 'bit :initial-element 0))
         (lower (make-vec n :initial-element most-negative-double-float))
         (upper (make-vec n :initial-element most-positive-double-float)))
    ;(declare (dynamic-extent has-lower has-upper))
    (dolist (c constraints)
      (when (opt-constraint-bounds-p c)
        (with-constraint (type terms bound) c
          (assert (= 1 (length terms)))
          (with-term (f v) (car terms)
            (multiple-value-bind (type bound)
                (if (< f 0)
                    (values (opt-type-flip type) (/ bound f))
                    (values type (/ bound f)))
              (let ((i (opt-variable-position variables v)))
                (labels ((setit (v h b)
                           (setf (aref v i) b
                                 (aref h i) 1))
                         (upper ()
                           (setit upper has-upper (min (aref upper i) bound)))
                         (lower ()
                           (setit lower has-lower (max (aref lower i) bound))))
                  (ecase type
                    (<= (upper))
                    (>= (lower))
                    (= (lower) (upper))))))))))
    ;; Set Defaults
    (dotimes (i n)
      (when (zerop (aref has-lower i))
        (setf (aref lower i) default-lower))
      (when (zerop (aref has-upper i))
        (setf (aref upper i) default-upper)))
    ;; Result
    (values lower upper)))

(defun opt-terms-vec (variables terms &optional vec)
  (let ((vec (or vec (make-vec (opt-variable-count variables)))))
    ;; get initial row
    (dolist (term terms)
      (with-term (factor variable) term
        (setf (vecref vec (opt-variable-position variables variable))
              factor)))
    vec))

(defun opt-constraint-row (variables constraint
                            &key
                              vec
                              (default-lower most-negative-double-float)
                              (default-upper most-positive-double-float))
  (with-constraint (c-type terms value) constraint
    (let ((v (opt-terms-vec variables terms vec)))
      (ecase c-type
        (<= (values default-lower v value))
        (>= (values value v default-upper))
        (=  (values value v value))))))

(defun opt-constraint-matrices (variables constraints
                                &key
                                  (default-lower most-negative-double-float)
                                  (default-upper most-positive-double-float))
  (let* ((constraints (loop for c in constraints
                         unless (opt-constraint-bounds-p c)
                         collect c))
         (m (length constraints))
         (n (length variables))
         (lower (make-vec m))
         (upper (make-vec m))
         (A (make-matrix m n)))
    (loop
       for c in constraints
       for i below m
       do (let ((row (matrix-block A i 0 1 n)))
            (multiple-value-bind (l r u)
                (opt-constraint-row variables c
                                    :vec row
                                    :default-lower default-lower
                                    :default-upper default-upper)
              (declare (ignore r))
              (setf (vecref lower i) l
                    (vecref upper i) u))))
    (values lower A upper)))

(defun opt-linear-objective (variables objectives)
  (opt-terms-vec variables objectives))


;; (opt-constraint-matrices '(x y)
;;                          '((:<= (+ (* 120 x) (* 210 y)) 1500)
;;                            (:<= (+ (* 110 x) (* 30 y)) 4000)
;;                            (:<= (+ (* 1 x) (* 1 y)) 75)))
