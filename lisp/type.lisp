;;;; -*- mode: lisp -*-
;;;;
;;;; Copyright (c) 2012, Georgia Tech Research Corporation
;;;; All rights reserved.
;;;;
;;;; Author(s): Neil T. Dantam <ntd@gatech.edu>
;;;; Georgia Tech Humanoid Robotics Lab
;;;; Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
;;;;
;;;;
;;;; This file is provided under the following "BSD-style" License:
;;;;
;;;;
;;;;   Redistribution and use in source and binary forms, with or
;;;;   without modification, are permitted provided that the following
;;;;   conditions are met:
;;;;
;;;;   * Redistributions of source code must retain the above copyright
;;;;     notice, this list of conditions and the following disclaimer.
;;;;
;;;;   * Redistributions in binary form must reproduce the above
;;;;     copyright notice, this list of conditions and the following
;;;;     disclaimer in the documentation and/or other materials provided
;;;;     with the distribution.
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

;;; Matrix type
;;; - Default
;;;   - Column Major
;;;   - Double Float
;;;   - Vectors are column
;;; - Fields
;;;   - Data Vector
;;;   - Offset
;;;   - Stride
;;;   - Rows
;;;   - Cols
(in-package :amino)

(defconstant +x+ 0)
(defconstant +y+ 1)
(defconstant +z+ 2)
(defconstant +w+ 3)

(define-condition matrix-storage (error)
  ((message
    :initarg :message)))

(defmethod print-object ((object matrix-storage) stream)
  (print-unreadable-object (object stream :type t :identity t)
    (format stream ": ~A"
            (slot-value object 'message))))

(defun matrix-storage-error (format &rest args)
  (error 'matrix-storage
         :message (apply #'format nil format args)))


(defstruct real-array
  (data nil :type  (simple-array double-float (*))))

(defstruct (matrix (:constructor %make-matrix)
                   (:conc-name %matrix-))
  "Descriptor for a matrix following LAPACK conventions."
  (data nil :type (or (simple-array double-float (*))
                      (simple-array float (*))
                      (simple-array fixnum (*))
                      (simple-array (signed-byte 32) (*))
                      (simple-array (signed-byte 64) (*))
                      (simple-array t (*))))
  (offset 0 :type (integer 0 #.most-positive-fixnum))
  (stride 0 :type (integer 1 #.most-positive-fixnum))
  (cols 0 :type (integer 1 #.most-positive-fixnum))
  (rows 0 :type (integer 1 #.most-positive-fixnum)))


(deftype double-matrix ()
  `(or matrix list (simple-array double-float (*))))

(defun make-matrix (m n)
  "Make a new matrix with M rows and N cols."
  (if (= 1 n)
      (make-array m :element-type 'double-float)
      (%make-matrix :data (make-array (* m n)
                                      :element-type 'double-float)
                    :offset 0
                    :stride m
                    :rows m
                    :cols n)))

(defun matrix-data (m)
  (etypecase m
    ((simple-array * (*))  m)
    (real-array (real-array-data m))
    (list (map-into (make-array (length m) :element-type 'double-float)
                    (lambda (k) (coerce k 'double-float))
                    m))
    (matrix (%matrix-data m))))

(defun matrix-offset (m)
  (etypecase m
    ((simple-array * (*))  0)
    (list 0)
    (real-array 0)
    (matrix (%matrix-offset m))))

(defun matrix-stride (m)
  (etypecase m
    ((simple-array * (*))  (length m))
    (real-array (length (real-array-data m)))
    (list (length m))
    (matrix (%matrix-stride m))))

(defun matrix-rows (m)
  (etypecase m
    ((simple-array * (*)) (length m))
    (list (length m))
    (matrix (%matrix-rows m))))

(defun matrix-cols (m)
  (etypecase m
    ((simple-array * (*)) 1)
    (list 1)
    (matrix (%matrix-cols m))))

(defun matrix-type (matrix)
  "Return the element type of MATRIX."
  (array-element-type (matrix-data matrix)))

(defun matrix-size (matrix)
  "Returns number of elements in matrix."
  (* (matrix-rows matrix)
     (matrix-cols matrix)))

(defun matrix-index (matrix i j)
  "Compute the array index for element at row i, col j."
  (let ((rows (matrix-rows matrix))
        (cols (matrix-cols matrix))
        (stride (matrix-stride matrix))
        (offset (matrix-offset matrix)))
    (assert (< i rows))
    (assert (< j cols))
    (+ offset
       (* j stride)
        i)))

;; (defun matrix-view-array (array)
;;   (ecase (array-rank array)
;;     (1
;;      (%make-matrix :data array
;;                    :offset 0
;;                    :stride (length array)
;;                    :rows (length array)
;;                    :cols 1))
;;     (2)))

(defun matref (matrix i j)
  "Return element at row i, col j."
  (aref (matrix-data matrix)
        (matrix-index matrix i j)))

(defun vecref (matrix i)
  "Return I'th element of column vector MATRIX"
  (etypecase matrix
    (simple-vector (svref matrix i))
    (array (aref matrix i))
    (matrix (matref matrix i 0))))

(defun (setf matref) (value matrix i j)
  "Set element at row i, col j."
  (setf (aref (matrix-data matrix)
              (matrix-index matrix i j))
        value))

(defun row-matrix (&rest rows)
  "Create a matrix from the given rows."
  (let ((m (length rows))
        (n (length (car rows))))
    (let ((matrix (make-matrix m n)))
      (loop
         with type = (matrix-type matrix)
         for row in rows
         for i from 0
         do (loop
               for x in row
               for j from 0
               do (setf (matref matrix i j)
                        (coerce x type))))
      matrix)))

(defun col-matrix (&rest cols)
  "Create a matrix from the given columns."
  (let ((n (length cols))
        (m (length (car cols))))
    (let ((matrix (make-matrix m n)))
      (loop
         with type = (matrix-type matrix)
         for col in cols
         for j from 0
         do (loop
               for x in col
               for i from 0
               do (setf (matref matrix i j)
                        (coerce x type))))
      matrix)))

(defun matrix-copy (matrix &optional
                    (copy (make-matrix (matrix-rows matrix)
                                       (matrix-cols matrix))))
  "Create a copy of MATRIX."
  (let ((m (matrix-rows matrix))
        (n (matrix-cols matrix)))
    (dotimes (j n)
      (dotimes (i m)
        (setf (matref copy i j)
              (matref matrix i j))))
    copy))


(defun matrix-counts-in-bounds-p (data-length offset stride rows cols)
  "Check if given counts are within array bounds."
  (declare (type fixnum data-length offset stride rows cols))
  (and (>= stride rows)
       (<= (+ offset
              (* stride cols))
           data-length)))

(defun matrix-in-bounds-p (matrix)
  (matrix-counts-in-bounds-p (length (matrix-data matrix))
                             (matrix-offset matrix)
                             (matrix-stride matrix)
                             (matrix-rows matrix)
                             (matrix-cols matrix)))


(declaim (inline matrix-block))
(defun matrix-block (matrix i j m n)
  "Make a new descriptor for a sub-block of MATRIX.
MATRIX: original matrix.
I: first row in MATRIX of the block.
J: first col in MATRIX of the block.
M: rows in the block.
N: cols in the block."
  ;; TODO: Check bounds
  (%make-matrix :data (matrix-data matrix)
                :offset (matrix-index matrix i j)
                :stride (matrix-stride matrix)
                :rows m
                :cols n))

(defun matrix-row (matrix i)
  "Block for row I"
  (matrix-block matrix i 0 1 (matrix-cols matrix)))

(defun matrix-col (matrix j)
  "Block for col J"
  (matrix-block matrix 0 j (matrix-rows matrix) 1))


(defun matrix-contiguous-p (matrix)
  "Is the matrix stored contiguously?"
  (or (= (matrix-stride matrix)
         (matrix-rows matrix))
      (= 1 (matrix-cols matrix))))

(defmethod print-object ((object matrix) stream)
  "Print a matrix."
  (format stream "~&#S(MATRIX: ")
  (dotimes (i (matrix-rows object))
    (unless (= i 0)
      (dotimes (j 11)
        (write-char #\  stream)))

    (write-char #\[ stream)
    (write-char #\  stream)
    (dotimes (j (matrix-cols object))
      (format stream "~A " (let ((x (matref object i j)))
                             (if (and (< x 1d-15) (> x -1d-15 ))
                                 0d0 x))))
    (write-char #\] stream)
    (unless (= (1+ i) (matrix-rows object))
      (terpri stream)))
  (format stream ")"))


;;;;;;;;;;;;;;;
;;; VECTORS ;;;
;;;;;;;;;;;;;;;

;; Lisp array vectors
(defun make-vec (n)
  (make-array n :element-type 'double-float))

(defun vec (&rest args)
  (let ((vec (make-vec (length args))))
    (loop
       for i from 0
       for x in args
       do (setf (aref vec i)
                (coerce x 'double-float)))
    vec))

(defun veccat (&rest args)
  (let* ((n (loop for x in args summing (length x)))
         (y (make-vec n))
         (i -1))
    (dolist (x args)
      (dotimes (j (length x))
        (setf (aref y (incf i))
              (aref x j))))
    y))

(defun vec-copy (vec &key (start 0) (end (length vec)))
  ;; TODO: matrix version
  (let* ((n (- end start))
         (new (make-vec n)))
    (dotimes (i n)
      (setf (aref new i)
            (aref vec (+ start i))))
    new))

(defun matrix-vector-store-p (matrix)
  "Is the matrix stored in a way that looks like a vector?"
  (or (= (matrix-stride matrix)
         (matrix-rows matrix))
      (= 1 (matrix-rows matrix))
      (= 1 (matrix-cols matrix))))

(defun matrix-vector-n-p (matrix n inc)
  (let ((rows (matrix-rows matrix))
        (cols (matrix-cols matrix))
        (stride (matrix-stride matrix)))
  (or
   ;; column vector storage
   (and (= n rows stride)
        (= inc 1 cols))
   ;; row vector storage
   (and (= n cols)
        (= 1 rows)
        (= inc stride)))))

(defmacro check-vector-storage (place)
  `(progn
     (check-type ,place matrix)
     (unless (matrix-vector-store-p ,place)
       (matrix-storage-error "~A is not stored as a vector" ,place))))

(defun matrix-counts-vector-increment (rows cols stride)
  (cond
    ((or (= 1 cols)
         (= rows stride))
     1)
    ((= 1 rows) stride)
    (t (matrix-storage-error "stored as a vector"))))

(defun matrix-vector-increment (matrix)
  (declare (type matrix matrix))
  (matrix-counts-vector-increment (matrix-rows matrix)
                                  (matrix-cols matrix)
                                  (matrix-stride matrix)))

(defun make-row-vector (n)
  (make-matrix 1 n))

(defun make-col-vector (n)
  (make-matrix n 1))

(defun row-vector (&rest args)
  (declare (dynamic-extent args))
  (row-matrix args))

(defun col-vector (&rest args)
  (declare (dynamic-extent args))
  (col-matrix args))



(defun wrap-col-vector (column)
  (declare (type (simple-array double-float (*)) column))
  (%make-matrix :data column
                :offset 0
                :stride (length column)
                :cols 1
                :rows (length column)))

(defmacro with-matrix ((var value) &body body)
  "Ensure value is a matrix descriptor.
Will wrap simple-vectors in a descriptor struct."
  (with-gensyms (body-fun desc value-sym n)
    `(flet ((,body-fun (,var) ,@body))
       (let ((,value-sym ,value))
         (etypecase ,value-sym
           (matrix (,body-fun ,value-sym))
           ((simple-array double-float (*))
            (let* ((,n (length ,value-sym))
                   (,desc (%make-matrix :data ,value-sym
                                        :stride ,n
                                        :rows ,n
                                        :cols 1)))
              ;; (declare (dynamic-extent ,desc))
              (,body-fun ,desc))))))))
