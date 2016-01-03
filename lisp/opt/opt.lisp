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

(def-la-cfun ("aa_opt_qp_solve_cgal" aa-opt-qp-solve-cgal
                                     :pointer-type :pointer) :int
    (m size-t)
    (n size-t)
    (A :matrix)
    (b :pointer)
    (c :pointer)
    (c0 :double)
    (D :matrix)
    (l :pointer)
    (u :pointer)
    (x :pointer))


(defstruct qp
  A
  B
  C
  (c0 0d0 :type double-float)
  D
  L
  U)

(defun qp-var-count (qp)
  (matrix-cols (qp-a qp)))

(defun qp-solve (qp &optional (x (make-vec (qp-var-count qp))))
  (let ((r))
    (with-foreign-matrix (a ld-a rows-a cols-a) (qp-a qp) :input
      (with-foreign-simple-vector (b n-b) (qp-b qp) :input
        (with-foreign-simple-vector (c n-c) (qp-c qp) :input
          (with-foreign-matrix (d ld-d rows-d cols-d) (qp-d qp) :input
            (with-foreign-simple-vector (l n-l) (qp-l qp) :input
              (with-foreign-simple-vector (u n-u) (qp-u qp) :input
                (with-foreign-simple-vector (x n-x) x :output
                  (assert (= n-x cols-a rows-d cols-d n-c n-l n-u))
                  (assert (= rows-a n-b))
                  ;(break)
                  (setq r
                        (aa-opt-qp-solve-cgal rows-a cols-a
                                              a ld-a
                                              b c (qp-c0 qp)
                                              d ld-d
                                              l u
                                              x)))))))))
    (assert (zerop r))
    x))


(defun opt-term (factor variable)
  `(* ,factor ,variable))

(defun opt-constraint (type terms value)
  `(,type (+ ,@terms) ,value))

(defmacro with-constraint ((type terms value) constraint &body body)
  (with-gensyms (op fun)
    `(flet ((,fun (,type ,terms ,value)
              ,@body))
       (destructuring-bind (,type (,op &rest ,terms) ,value) ,constraint
         (assert (eq ,op '+))
         (,fun ,type ,terms (coerce ,value 'double-float))))))

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

(defun opt-bounds (variables constraints
                   &key
                     (default-lower 0d0)
                     (default-upper most-positive-double-float))
  (let ((lower (make-array (opt-variable-count variables)
                           :element-type 'double-float
                           :initial-element default-lower))

        (upper (make-array (opt-variable-count variables)
                           :element-type 'double-float
                           :initial-element default-upper)))
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
                (labels ((upper ()
                           (assert (>= bound (aref lower i)))
                           (setf (aref upper i)
                                 (min (aref upper i) bound)))
                         (lower ()
                           (assert (<= bound (aref upper i)))
                           (setf (aref lower i)
                                 (max (aref lower i) bound))))

                  (ecase type
                    (<= (upper))
                    (>= (lower))
                    (= (lower) (upper))))))))))
    (values lower upper)))

(defun opt-variables (constraints objectives)
  (let ((hash-vars (make-hash-table :test #'equal)))
    (labels ((add-term (term)
               (with-term (factor variable) term
                 (declare (ignore factor))
                 (setf (gethash variable hash-vars) t))))
      (dolist (c constraints)
        (with-constraint (type terms value) c
          (declare (ignore value))
          (assert (find type '(= <= >=)))
          (map nil #'add-term terms)))
      (map nil #'add-term objectives))
    (hash-table-keys hash-vars)))

(defun qp-constraint-rows (variables constraint &key)
  (with-constraint (type terms value) constraint
    (let ((v (make-vec (opt-variable-count variables))))
      ;; get initial row
      (dolist (term terms)
        (with-term (factor variable) term
          (setf (vecref v (opt-variable-position variables variable))
                factor)))
      ;; munge constraint
      (ecase type
        (<= (values (list v) (list value)))
        (>= (values (list (g* -1 v))
                    (list (- value))))
        (= (values (list v (g* -1 v))
                   (list value (- value))))
        ))))


(defun qp-objectives (variables objectives)
  (let ((c (make-vec (opt-variable-count variables))))
    (dolist (o objectives)
      (with-term (f v) o
        (setf (vecref c (opt-variable-position variables v))
              f)))
    c))

(defun qp (constraints objectives
           &key
             (result-type :alist)
             (default-lower 0d0)
             (default-upper most-positive-double-float))
  (let* ((vars (opt-variables constraints objectives))
         (n-var (opt-variable-count vars))
         (a-rows)
         (b-list))
    ;; construct constraints
    (dolist (c constraints)
      (multiple-value-bind (row b-elt) (qp-constraint-rows vars c)
        (setq a-rows (append row a-rows)
              b-list (append b-elt b-list))))
    ;; upper/lower
    (multiple-value-bind (lower upper) (opt-bounds vars constraints
                                                   :default-lower default-lower
                                                   :default-upper default-upper)
      (let* ((qp (make-qp :a (apply #'row-matrix a-rows)
                          :b (make-array (length b-list)
                                         :element-type 'double-float
                                         :initial-contents b-list)
                          :c (qp-objectives vars objectives)
                          :d (make-matrix n-var n-var)
                          :l lower
                          :u upper))
             (solution-vector (qp-solve qp)))
        (ecase result-type
          (:alist (loop for v in vars
                     for x across solution-vector
                     collect (cons v x))))))))
