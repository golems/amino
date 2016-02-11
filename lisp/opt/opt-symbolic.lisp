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

;;;;;;;;;;;;;;;;;;;
;;; Expressions ;;;
;;;;;;;;;;;;;;;;;;;

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


(defmacro with-linear-term ((factor variable) term &body body)
  (with-gensyms (op fun term-var)
    `(let ((,term-var ,term))
       (flet ((,fun (,factor ,variable)
                ,@body))
         (if (and (listp ,term-var)
                  (eq '* (car ,term-var)))
             (destructuring-bind (,op ,factor ,variable) ,term-var
               (declare (ignore ,op))
               (,fun ,factor ,variable))
             (,fun 1d0 ,term-var))))))

(defmacro with-quadratic-term ((factor variable-1 variable-2) term &body body)
  (with-gensyms (op fun term-var)
    `(let ((,term-var ,term))
       (flet ((,fun (,factor ,variable-1 ,variable-2)
                ,@body))
         (destructuring-bind (,op ,factor ,variable-1 ,variable-2) ,term-var
           (declare (ignore ,op))
           (,fun ,factor ,variable-1 ,variable-2))))))

(defun linear-term (factor variable)
  `(* ,factor ,variable))

(defun quadratic-term (factor variable-1 variable-2)
  `(* ,factor ,variable-1 ,variable-2))

(defun opt-constraint (type terms value)
  `(,type (+ ,@terms) ,value))

(defun opt-constraint-bounds (lower terms upper)
  `((>= (+ ,@terms) ,lower)
    (<= (+ ,@terms) ,upper)))

(defun linear-term-p (term)
  (or (atom term)
      (not (eq '* (car term)))
      (= 3 (length term))))

(defun quadratic-term-p (term)
  (= 4 (length term)))

;;;;;;;;;;;;;;;;;;;;;
;;; Normalization ;;;
;;;;;;;;;;;;;;;;;;;;;

(defstruct normalized-constraint
  (lower nil :type (or double-float null))
  (terms nil :type list)
  (upper nil :type (or double-float null)))

(defun normalized-constraint-bounds-p (normalized-constraint)
  (let ((terms (normalized-constraint-terms normalized-constraint)))
    (and terms (null (cdr terms)))))

(defun normalized-constraint-length (normalized-constraint)
  (length (normalized-constraint-terms normalized-constraint)))

(defun normalized-linear-term (variable factor)
  (cons variable factor))

(defmacro with-normalized-term ((factor variable) term &body body)
  `(destructuring-bind (,variable . ,factor) ,term
     (declare (type double-float ,factor))
     ,@body))

(defun normalize-terms (terms)
  (let ((m (make-tree-map #'gsymbol-compare)))
    (dolist (term terms)
      (with-linear-term (f v) term
        (let ((f1 (multiple-value-bind (f0 key present) (tree-map-find m v)
                    (declare (ignore key))
                    (if present
                        (+ f f0)
                        (coerce f 'double-float)))))
          (declare (type double-float f1))
          (tree-map-insertf m v f1))))
    (map-tree-map :inorder 'list #'normalized-linear-term m)))

(defun update-normalized-constraint (nc type value)
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
           (error "Unable to satisfy constraint ~A" nc))
         (setf (normalized-constraint-lower nc) value))
       (if-let ((current (normalized-constraint-upper nc)))
         (unless (= current value)
           (error "Unable to satisfy constraint ~A" nc))
         (setf (normalized-constraint-upper nc) value))))
  nc)

(defun normalize-constraints (constraints)
  (let ((hash (make-hash-table :test #'equal)))
    (dolist (c constraints)
      (with-constraint (type terms value) c
        (let* ((terms (normalize-terms terms))
               (nc (or (gethash terms hash)
                       (setf (gethash terms hash)
                             (make-normalized-constraint :terms terms)))))
          (update-normalized-constraint nc type value))))
    (hash-table-values hash)))

;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Matrix Generation ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;

(defun opt-variable-position (variables var)
  (position var variables :test #'equal))

(defun opt-variable-count (variables)
  (length variables))

(defun opt-variables (constraints objectives)
  (let ((hash-vars (make-hash-table :test #'equal)))
    (labels ((add-term (term)
               (with-linear-term (factor variable) term
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
  (declare (type double-float default-lower default-upper))
  (let* ((n (opt-variable-count variables))
         (has-bound (make-array n :element-type 'bit  :initial-element 0))
         (lower (make-vec n :initial-element default-lower))
         (upper (make-vec n :initial-element default-upper)))
    (dolist (c constraints)
      (when (normalized-constraint-bounds-p c)
        (destructuring-bind ((v . f)) (normalized-constraint-terms c)
            (multiple-value-bind (xl xu)
                (let ((l (normalized-constraint-lower c))
                      (u (normalized-constraint-upper c)))
                  (if (< f 0)
                      (values u l)
                      (values l u)))
              (let ((i (opt-variable-position variables v)))
                (assert (zerop (aref has-bound i)))
                (setf (aref has-bound i) 1)
                (when xl (setf (aref lower i) (/ xl f)))
                (when xu (setf (aref upper i) (/ xu f))))))))
    ;; Result
    (values lower upper)))

(defun map-opt-terms (function terms)
  (dolist (term terms)
    (with-normalized-term (factor variable) term
      (funcall function variable factor))))


(defun opt-terms-vec (variables terms &optional vec)
  (let ((vec (or vec (make-vec (opt-variable-count variables)))))
    ;; get initial row
    (map-opt-terms (lambda (variable factor)
                     (setf (vecref vec (opt-variable-position variables variable))
                           factor))
                   terms)
    vec))

(defun opt-constraint-crs (variables constraints
                           &key
                             (default-lower most-negative-double-float)
                             (default-upper most-positive-double-float))
  (let* ((constraints (loop for c in constraints
                         unless (normalized-constraint-bounds-p c)
                         collect c))
         (m (length constraints))
         (n (opt-variable-count variables))
         (e (loop for c in constraints
               summing (normalized-constraint-length c)))
         (lower (make-vec m :initial-element default-lower))
         (upper (make-vec m :initial-element default-upper))
         (A (make-crs-matrix m n e)))
    (loop
       for c in constraints
       for i from 0
       for terms = (normalized-constraint-terms c)
       for l = (normalized-constraint-lower c)
       for u = (normalized-constraint-upper c)
       do
         (map-opt-terms (lambda (var factor)
                          (amino-type::crs-matrix-add-elt A
                                                          (opt-variable-position variables var)
                                                          factor))
                        terms)
         (amino-type::crs-matrix-inc-row A)
         (when l (setf (vecref lower i) l))
         (when u (setf (vecref upper i) u)))
    (values lower A upper)))

(defun opt-linear-objective (variables linear-terms)
  (opt-terms-vec variables linear-terms))

(defun opt-quadratic-objective (variables quadratic-terms)
  (let ((hash (make-hash-table :test #'equal)))
    ;; Collect terms
    (dolist (o quadratic-terms)
      (with-quadratic-term (f v1 v2) o
        (let ((i (opt-variable-position variables v1))
              (j (opt-variable-position variables v2)))
          (let ((key (if (< i j)
                         (cons i j)
                         (cons j i))))
            (setf (gethash key hash)
                  (+ f (gethash key hash 0d0)))))))
    ;; construct sparse matrix
    (let* ((keys (sort (hash-table-keys hash) (lambda (a b)
                                                (destructuring-bind (i1 . j1) a
                                                  (destructuring-bind (i2 . j2) b
                                                    ;; order by row, then colum
                                                    (if (= i1 i2)
                                                       (< j1 j2)
                                                       (< i1 i2)))))))
           (n (opt-variable-count variables))
           (crs (make-crs-matrix n n (length keys)))
           (i0 0))
      (dolist (key keys)
        (destructuring-bind (i . j) key
          (unless (= i i0)
            (incf i)
            (amino-type::crs-matrix-inc-row crs))
          (amino-type::crs-matrix-add-elt crs j (gethash key hash))))
      (amino-type::crs-matrix-inc-row crs)
      crs)))


(defun opt-objective (variables objectives)
  (opt-linear-objective variables objectives))
;; TODO: fix quadratic things

  ;; (let ((linear-terms)
  ;;       (quadratic-terms))
  ;;   ;; filter terms
  ;;   (dolist (o objectives)
  ;;     (cond ((linear-term-p o) (push o linear-terms))
  ;;           ((quadratic-term-p o) (push o quadratic-terms))
  ;;           (t (error "Unrecognized term type: ~A" o))))
  ;;   ;; Result
  ;;   (values (when linear-terms
  ;;             (opt-linear-objective variables linear-terms))
  ;;           (when quadratic-terms
  ;;             (opt-quadratic-objective variables quadratic-terms)))))

;;;;;;;;;;;;;;;;;
;;; Container ;;;
;;;;;;;;;;;;;;;;;

(defun compare-normterms (a b)
  (with-normalized-term (a-f a-v) a
    (with-normalized-term (b-f b-v) b
      (or-compare (gsymbol-compare a-v b-v)
                  (double-compare a-f b-f)))))

(defun compare-normterm-list (a b)
  (if a
      (if b
          (or-compare (compare-normterms (car a) (car b))
                      (compare-normterm-list (cdr a) (cdr b)))
          1)
      (if b
          -1
          0)))


(defclass optexp ()
  ((constraints ;; terms -> normalized constaint
    :initform (make-tree-map #'compare-normterm-list)
    :initarg :constraints
    :accessor optexp-constraints)
   (objective ;; variable -> coefficient
    :initform (make-tree-map #'gsymbol-compare)
    :initarg :objective
    :accessor optexp-objective)
   (variables
    :initform (make-tree-set #'gsymbol-compare)
    :initarg :variables
    :accessor optexp-variables)
   (binary-variables
    :initform (make-tree-set #'gsymbol-compare)
    :initarg :binary-variables
    :accessor optexp-binary-variables)
   (integer-variables
    :initform (make-tree-set #'gsymbol-compare)
    :initarg :integer-variables
    :accessor optexp-integer-variables)
   (default-x-lower
     :initform 0
     :initarg :default-x-lower
     :accessor optexp-default-x-lower)
   (default-x-upper
     :initform most-positive-double-float
     :initarg :default-x-upper
     :accessor optexp-default-x-upper)
   (default-b-lower
     :initform most-negative-double-float
     :initarg :default-b-lower
     :accessor optexp-default-x-upper
     :accessor optexp-default-b-lower)
   (default-b-upper
     :initform most-positive-double-float
     :initarg :default-b-upper
     :accessor optexp-default-b-upper)))

(defmacro optexp-slot (optexp slot)
  `(slot-value ,optexp ',slot))

(defun make-optexp ()
  (make-instance 'optexp))

(defun optexp-variable-list (oe)
  (tree-set-list (optexp-variables oe)))

(defun update-optexp (optexp &key
                               (constraints (optexp-slot optexp constraints))
                               (objective (optexp-slot optexp objective))
                               (variables (optexp-slot optexp variables))
                               (binary-variables (optexp-slot optexp binary-variables))
                               (integer-variables (optexp-slot optexp integer-variables))
                               (default-x-lower (optexp-slot optexp default-x-lower))
                               (default-x-upper (optexp-slot optexp default-x-upper))
                               (default-b-lower (optexp-slot optexp default-b-lower))
                               (default-b-upper (optexp-slot optexp default-b-upper)))
  (make-instance 'optexp
                 :constraints constraints
                 :objective objective
                 :variables variables
                 :binary-variables binary-variables
                 :integer-variables integer-variables
                 :default-x-lower default-x-lower
                 :default-x-upper default-x-upper
                 :default-b-lower default-b-lower
                 :default-b-upper default-b-upper))

(defun normalized-terms-variables (variables terms)
  (fold (lambda (set term)
          (with-normalized-term (factor variable) term
            (declare (ignore factor))
            (tree-set-insert set variable)))
        variables terms))


(defun optexp-add-constraint (oe type terms rhs)
  (let* ((rhs (coerce rhs 'double-float))
         (terms (normalize-terms terms))
         (map (optexp-constraints oe))
         (nc (if-let ((nc (tree-map-find map terms)))
               (copy-structure nc)
               (make-normalized-constraint :terms terms))))
    (setq nc (update-normalized-constraint nc type rhs))
    (update-optexp oe
                   :constraints (tree-map-insert map terms nc)
                   :variables (normalized-terms-variables (optexp-variables oe) terms))))

(defun optexp= (oe terms rhs)
  (optexp-add-constraint oe '= terms rhs))

(defun optexp<= (oe terms rhs)
  (optexp-add-constraint oe '<= terms rhs))

(defun optexp>= (oe terms rhs)
  (optexp-add-constraint oe '>= terms rhs))

(defun optexp-add-constraint-exp (oe constraint)
  (with-constraint (type terms rhs) constraint
    (optexp-add-constraint oe type terms rhs)))

(defun optexp-add-bounds (oe lower terms upper)
  (let* ((lower (coerce lower 'double-float))
         (upper (coerce upper 'double-float))
         (terms (normalize-terms terms))
         (map (optexp-constraints oe))
         (nc (if-let ((nc (tree-map-find map terms)))
               (progn (setq nc (copy-structure nc))
                      (setq nc (update-normalized-constraint nc '>= lower))
                      (setq nc (update-normalized-constraint nc '<= upper)))
               (make-normalized-constraint :lower lower :terms terms :upper upper))))
    (update-optexp oe
                   :constraints (tree-map-insert map terms nc)
                   :variables (normalized-terms-variables (optexp-variables oe) terms))))

(defun optexp-add-objective (oe factor variable)
  (let* ((map (optexp-objective oe))
         (factor (coerce factor 'double-float))
         (old-factor (tree-map-find map variable))
         (new-factor (if old-factor
                         (+ old-factor factor)
                         factor)))
    (update-optexp oe
                   :objective (tree-map-insert map variable new-factor)
                   :variables (tree-set-insert (optexp-variables oe) variable))))

(defun optexp-set-binary (oe variable)
  (update-optexp oe
                 :binary-variables (tree-set-insert (optexp-binary-variables oe)
                                                    variable)))

(defun optexp-set-integer (oe variable)
  (update-optexp oe
                 :integer-variables (tree-set-insert (optexp-integer-variables oe)
                                                    variable)))

(defun optexp (&key optexp constraints objective binary-variables integer-variables
                 default-x-lower default-x-upper
                 default-b-lower default-b-upper)
  (let ((oe (or optexp (make-optexp))))
    ;; constraints
    (setq oe (fold (lambda (oe c)
                     (with-constraint (type terms value) c
                       (optexp-add-constraint oe type terms value)))
                   oe constraints))
    ;; objective
    (setq oe (fold (lambda (oe o)
                     (with-linear-term (factor variable) o
                       (optexp-add-objective oe factor variable)))
                   oe objective))
    ;; binary
    (setq oe (fold #'optexp-set-binary oe binary-variables))
    ;; integer
    (setq oe (fold #'optexp-set-binary oe integer-variables))
    ;; default bounds
    (when default-x-lower
      (setf (optexp-default-x-lower oe) (coerce default-x-lower 'double-float)))
    (when default-x-upper
      (setf (optexp-default-x-upper oe) (coerce default-x-upper 'double-float)))
    (when default-b-lower
      (setf (optexp-default-b-lower oe) (coerce default-b-lower 'double-float)))
    (when default-b-upper
      (setf (optexp-default-b-upper oe) (coerce default-b-upper 'double-float)))
    ;; result
    oe))

(defun optexp-constraint-matrix (oe)
  (opt-constraint-crs (optexp-variable-list oe)
                      (tree-map-values (optexp-constraints oe))
                      :default-lower (optexp-default-b-lower oe)
                      :default-upper (optexp-default-b-upper oe)))

(defun optexp-bounds-matrix (oe)
  (opt-bounds (optexp-variable-list oe)
              (tree-map-values (optexp-constraints oe))
              :default-lower (optexp-default-x-lower oe)
              :default-lower (optexp-default-x-upper oe)))

;; TODO: quadratic objectives
(defun optexp-objective-matrix (oe)
  (opt-objective (optexp-variable-list oe)
                 (map-tree-map :inorder 'list #'normalized-linear-term
                               (optexp-objective oe))))


;; (opt-constraint-matrices '(x y)
;;                          '((:<= (+ (* 120 x) (* 210 y)) 1500)
;;                            (:<= (+ (* 110 x) (* 30 y)) 4000)
;;                            (:<= (+ (* 1 x) (* 1 y)) 75)))


;; (defun opt-constraint-bounds-p (constraint)
;;   (with-constraint (type terms value) constraint
;;     (declare (ignore type value))
;;     (and terms (null (cdr terms)))))

;; (defun opt-type-flip (type)
;;   (ecase type
;;     (= '=)
;;     (<= '>=)
;;     (>= '<=)))

;; (defun opt-constraints-equality-p (constraints)
;;   (dolist (c constraints)
;;     (unless (opt-constraint-bounds-p c)
;;       (with-constraint (type terms value) c
;;         (declare (ignore terms value))
;;         (unless (eq type '=)
;;           (return-from opt-constraints-equality-p nil)))))
;;   t)


;; (defun opt-constraint-matrices (variables constraints
;;                                 &key
;;                                   (default-lower most-negative-double-float)
;;                                   (default-upper most-positive-double-float))
;;   (let* ((constraints (loop for c in constraints
;;                          unless (normalized-constraint-bounds-p c)
;;                          collect c))
;;          (m (length constraints))
;;          (n (opt-variable-count variables))
;;          (lower (make-vec m :initial-element default-lower))
;;          (upper (make-vec m :initial-element default-upper))
;;          (A (make-matrix m n)))
;;     (loop
;;        for c in constraints
;;        for i from 0
;;        for row = (matrix-block A i 0 1 n)
;;        for terms = (normalized-constraint-terms c)
;;        for l = (normalized-constraint-lower c)
;;        for u = (normalized-constraint-upper c)
;;        do
;;          (opt-terms-vec variables terms row)
;;          (when l (setf (vecref lower i) l))
;;          (when u (setf (vecref upper i) u)))
;;     (values lower A upper)))
