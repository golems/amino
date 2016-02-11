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

(defparameter *default-lp-solver* :clp)

(defstruct lp-matrices
  variables
  A-crs
  b-lower
  b-upper
  c
  q
  x-lower
  x-upper)


(defun opt-var-count (problem)
  (amino-type::%crs-matrix-cols (lp-matrices-a-crs problem)))

(defun optexp-matrix (oe)
  (let* ((matrices (make-lp-matrices :variables (optexp-variable-list oe))))
    ;; constraints
    (symbol-macrolet ((b-lower (lp-matrices-b-lower matrices))
                      (A-crs (lp-matrices-a-crs matrices))
                      (b-upper (lp-matrices-b-upper matrices)))
      (multiple-value-setq (b-lower A-crs b-upper)
        (optexp-constraint-matrix oe)))
    ;; objectives
    (symbol-macrolet ((c (lp-matrices-c matrices))
                      (q (lp-matrices-q matrices)))
      (multiple-value-setq (c q)
        (optexp-objective-matrix oe)))
    ;; bounds
    (symbol-macrolet ((x-lower (lp-matrices-x-lower matrices))
                      (x-upper (lp-matrices-x-upper matrices)))
      (multiple-value-setq (x-lower x-upper)
        (optexp-bounds-matrix oe)))
    matrices))


(defun opt-result (result-type variables vec)
  (assert (= (length variables)
             (length vec)))
  (ecase result-type
    ((:vector vector vec)
     vec)
    ((list :list) (loop for x across vec
                     collect x))
    ((hash-table :hash :hash-table :hashtable)
     (let ((h (make-hash-table :test #'equal)))
       (loop for v in variables
          for x across vec
          do (setf (gethash v h)
                   x))
       h))
    (:alist
     (loop for v in variables
        for x across vec
        collect (cons v x)))))


(defun opt-set-type (cx variables var type)
  (let* ((i (opt-variable-position variables var))
         (r (progn
              (assert i)
              (aa-opt-set-type cx i type))))
    (unless (zerop r)
      (error "Could not set variable type of `~A' to `~A'"
             var type))
    (values)))

(defun optexp-solve (oe
                     &key
                       x
                       time
                       (strict t)
                       (solver *default-lp-solver*)
                       (result-type :alist))
  (let* ((matrices (optexp-matrix oe))
         ;; context
         (cx (opt-create (lp-matrices-a-crs matrices)
                         (lp-matrices-b-lower matrices)
                         (lp-matrices-b-upper matrices)
                         (lp-matrices-c matrices)
                         (lp-matrices-x-lower matrices)
                         (lp-matrices-x-upper matrices)
                         :solver solver))
         (x (or x (make-vec (opt-var-count matrices))))
         (r))
    ;; parameters
    (when-let ((q-crs (lp-matrices-q matrices)))
      (opt-set-quad-obj cx q-crs))
    ;; variable type
    (flet ((set-type (type vv)
             (let ((variables (lp-matrices-variables matrices)))
               (do-tree-set (v vv)
                 (opt-set-type cx variables v type)))))
      (set-type :integer (optexp-integer-variables oe))
      (set-type :binary (optexp-binary-variables oe)))
    ;; solve
    (with-foreign-simple-vector (x n-x) x :output
      (sb-int:with-float-traps-masked (:divide-by-zero :overflow  :invalid)
        (setq r (if time
                    (time (aa-opt-solve cx n-x x))
                    (aa-opt-solve cx n-x x)))))
    ;; check
    (when (and strict (not (zerop r)))
      (error "Could not solve LP: ~D" r))
    ;; result
    (if (eq :matrix result-type)
        matrices
        (opt-result result-type
                    (lp-matrices-variables matrices)
                    x))))

(defun lp (constraints objectives
           &key
             x
             time
             (strict t)
             binary-variables
             integer-variables
             (solver *default-lp-solver*)
             (result-type :alist)
             (default-x-lower 0d0)
             (default-x-upper most-positive-double-float)
             (default-b-lower most-negative-double-float)
             (default-b-upper most-positive-double-float))
  (optexp-solve (optexp :constraints constraints
                        :objective objectives
                        :binary-variables binary-variables
                        :integer-variables integer-variables
                        :default-x-lower default-x-lower
                        :default-x-upper default-x-upper
                        :default-b-lower default-b-lower
                        :default-b-upper default-b-upper)
                :x x
                :time time
                :strict strict
                :solver solver
                :result-type result-type))

;; (defstruct (qp-matrices (:include lp-matrices))
;;   D
;;   c0)


;; (lp-solve (lp-matrices '((<= (+ (* 120 x) (* 210 y)) 15000)
;;                          (<= (+ (* 110 x) (* 30 y)) 4000)
;;                          (<= (+ (* 1 x) (* 1 y)) 75)
;;                          (<= (+ (* 1 x)) 1000)
;;                          (<= (+ (* 1 y)) 1000))
;;                        '((* 1 x) (* 1 y))
;;                        :variables '(x y))
;;           :solver :clp)



;; (def-la-cfun ("aa_opt_lp_clp" aa-opt-lp-clp
;;                               :pointer-type :pointer) :int
;;   (m size-t)
;;   (n size-t)
;;   (A :matrix)
;;   (b-lower :pointer)
;;   (b-upper :pointer)
;;   (c :pointer)
;;   (x-lower :pointer)
;;   (x-upper :pointer)
;;   (x :pointer))


;; (def-la-cfun ("aa_opt_lp_lpsolve" aa-opt-lp-lpsolve
;;                               :pointer-type :pointer) :int
;;   (m size-t)
;;   (n size-t)
;;   (A :matrix)
;;   (b-lower :pointer)
;;   (b-upper :pointer)
;;   (c :pointer)
;;   (x-lower :pointer)
;;   (x-upper :pointer)
;;   (x :pointer))



;; (defun lp-solve (lp-matrices
;;                  &key
;;                    (strict nil)
;;                    (solver *default-lp-solver*)
;;                    x)
;;   (let ((r)
;;         (x (or x (make-vec (opt-var-count lp-matrices))))
;;         (solver (solver-function solver)))
;;     (with-foreign-matrix (a ld-a m n) (lp-matrices-a lp-matrices) :input
;;       (with-foreign-simple-vector (b-lower n-b-lower) (lp-matrices-b-lower lp-matrices) :input
;;         (with-foreign-simple-vector (b-upper n-b-upper) (lp-matrices-b-upper lp-matrices) :input
;;           (with-foreign-simple-vector (c n-c) (lp-matrices-c lp-matrices) :input
;;             (with-foreign-simple-vector (x-lower n-x-lower) (lp-matrices-x-lower lp-matrices) :input
;;               (with-foreign-simple-vector (x-upper n-x-upper) (lp-matrices-x-upper lp-matrices) :input
;;                 (with-foreign-simple-vector (x n-x) x :output
;;                   (assert (= m n-b-lower n-b-upper))
;;                   (assert (= n n-c n-x-lower n-x-upper n-x))
;;                   (sb-int:with-float-traps-masked (:divide-by-zero :overflow)
;;                     (setq r
;;                           (funcall solver
;;                                    m n
;;                                    a ld-a
;;                                    b-lower b-upper
;;                                    c
;;                                    x-lower x-upper
;;                                    x))))))))))
;;     (when (and strict (not (zerop r)))
;;       (error "Could not solve LP: ~D" r))
;;     x))




;; (defstruct qp
;;   type
;;   A
;;   B
;;   C
;;   (c0 0d0 :type double-float)
;;   D
;;   L
;;   U)

;; (defun qp-matrices (constraints objectives
;;                     &key
;;                       (default-lower 0d0)
;;                       (default-upper most-positive-double-float))
;;   )

;; (defun qp (constraints objectives
;;            &key
;;              (result-type :alist)
;;              (default-lower 0d0)
;;              (default-upper most-positive-double-float))
;;   (let* ((vars (opt-variables constraints objectives))
;;          (n-var (opt-variable-count vars))
;;          (equality (opt-constraints-equality-p  constraints))
;;          (a-rows)
;;          (b-list))
;;     ;; construct constraints
;;     (dolist (c constraints)
;;       (unless (opt-constraint-bounds-p c)
;;         (multiple-value-bind (row b-elt)
;;             (qp-constraint-rows vars c
;;                                 :type (if equality := :<=))
;;           (setq a-rows (append row a-rows)
;;                 b-list (append b-elt b-list)))))
;;     ;; upper/lower
;;     (multiple-value-bind (lower upper) (opt-bounds vars constraints
;;                                                    :default-lower default-lower
;;                                                    :default-upper default-upper)
;;       (let* ((qp (make-qp :type (if equality := :<=)
;;                           :a (apply #'row-matrix a-rows)
;;                           :b (make-array (length b-list)
;;                                          :element-type 'double-float
;;                                          :initial-contents b-list)
;;                           :c (qp-objectives vars objectives)
;;                           :d (make-matrix n-var n-var)
;;                           :l lower
;;                           :u upper))
;;              (solution-vector (qp-solve qp)))
;;         (ecase result-type
;;           (:alist (loop for v in vars
;;                      for x across solution-vector
;;                      collect (cons v x))))))))
;; (defun qp-solve (qp &optional (x (make-vec (qp-var-count qp))))
;;   (let ((r))
;;     (with-foreign-matrix (a ld-a rows-a cols-a) (qp-a qp) :input
;;       (with-foreign-simple-vector (b n-b) (qp-b qp) :input
;;         (with-foreign-simple-vector (c n-c) (qp-c qp) :input
;;           (with-foreign-matrix (d ld-d rows-d cols-d) (qp-d qp) :input
;;             (with-foreign-simple-vector (l n-l) (qp-l qp) :input
;;               (with-foreign-simple-vector (u n-u) (qp-u qp) :input
;;                 (with-foreign-simple-vector (x n-x) x :output
;;                   (assert (= n-x cols-a rows-d cols-d n-c n-l n-u))
;;                   (assert (= rows-a n-b))
;;                   ;(break)
;;                   (setq r
;;                         (aa-opt-qp-solve-cgal (qp-type qp)
;;                                               rows-a cols-a
;;                                               a ld-a
;;                                               b c (qp-c0 qp)
;;                                               d ld-d
;;                                               l u
;;                                               x)))))))))
;;     (assert (zerop r))
;;     x))
