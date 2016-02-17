;;;; -*- mode: lisp -*-
;;;;
;;;; Copyright (c) 2016, Rice University
;;;; All rights reserved.
;;;;
;;;; Author(s): Neil T. Dantam <ntd@gatech.edu>
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
;;;;   * Redistributions in binary form must reproduce the above
;;;;     copyright notice, this list of conditions and the following
;;;;     disclaimer in the documentation and/or other materials provided
;;;;     with the distribution.
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
;;;;   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
;;;;   NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
;;;;   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
;;;;   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;;;   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
;;;;   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
;;;;   EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

(in-package :amino-type)

(deftype foreign-int ()
  '(signed-byte #.(* 8 (cffi:foreign-type-size :int))))

(defstruct (crs-matrix (:include real-array)
                       (:constructor %make-crs-matrix)
                       (:conc-name %crs-matrix-))
  "Descriptor for a Compressed-Row-Storage matrix following LAPACK conventions."
  (cols 0 :type (integer 1 #.most-positive-fixnum))
  (rows 0 :type (integer 1 #.most-positive-fixnum))
  (col-ind nil :type (simple-array foreign-int (*)))
  (row-ptr nil :type (simple-array foreign-int (*)))
  (data-fill 0 :type (integer 0 #.most-positive-fixnum))
  (row-fill 0 :type (integer 0 #.most-positive-fixnum)))

(defun crs-matrix-inc-row (crs-matrix)
  ;; set row ptr
  (setf (aref (%crs-matrix-row-ptr crs-matrix) (%crs-matrix-row-fill crs-matrix))
        (%crs-matrix-data-fill crs-matrix))
  ;; increment fill pointer
  (incf (%crs-matrix-row-fill crs-matrix)))

(defun make-crs-matrix (row-count column-count element-count)
  (let ((result (%make-crs-matrix :cols column-count
                                  :rows row-count
                                  :col-ind (make-array element-count :element-type '(foreign-int))
                                  :row-ptr (make-array (1+ row-count) :element-type '(foreign-int))
                                  :data (make-array element-count :element-type 'double-float))))
    (crs-matrix-inc-row result)
    result))

(defun crs-matrix-add-elt (crs-matrix column element)
  (assert (< column (%crs-matrix-cols crs-matrix)))
  (let ((k (%crs-matrix-data-fill crs-matrix))
        (element (coerce element 'double-float)))
    ;; set element and column index
    (setf (aref (%crs-matrix-data crs-matrix) k) element
          (aref (%crs-matrix-col-ind crs-matrix) k) column))
  ;; increment fill
  (incf (%crs-matrix-data-fill crs-matrix)))


(defun crs-matrix-add-row (crs-matrix columns values)
  (assert (= (length columns) (length values)))
  (flet ((fun (column value)
           (crs-matrix-add-elt crs-matrix column value)))
    (declare (dynamic-extent #'fun))
    (map nil #'fun columns values))
  (crs-matrix-inc-row crs-matrix)
  crs-matrix)


(defun crs-matrix (columns-sequence values-sequence)
  (let ((m (length columns-sequence))
        (n -1)
        (e 0))
    (assert (= m (length values-sequence)))
    ;; Find counts
    (map nil (lambda (columns values)
               (let ((k (length columns)))
                 (assert (= k (length values)))
                 (incf e k))
               (map nil (lambda (j)
                          (setq n (max n j)))
                    columns))
         columns-sequence
         values-sequence)
    (incf n) ;; 0 indexes
    (let ((crs-matrix (make-crs-matrix m n e)))
      (map nil (lambda (columns values)
                 (crs-matrix-add-row crs-matrix columns values))
           columns-sequence values-sequence)
      crs-matrix)))


(defmacro with-foreign-crs ((row-count col-count element-count values col-indices row-ptr)
                            crs &body body)
  (with-gensyms (fun v-crs v-data v-col-ind v-row-ptr)
    `(flet ((,fun (,row-count ,col-count ,element-count ,values ,col-indices ,row-ptr)
              ,@body))
       (declare (dynamic-extent #',fun))
       (let* ((,v-crs ,crs)
              (,row-count (%crs-matrix-rows ,v-crs))
              (,col-count (%crs-matrix-cols ,v-crs))
              (,v-data (%crs-matrix-data ,v-crs))
              (,v-col-ind (%crs-matrix-col-ind ,v-crs))
              (,v-row-ptr (%crs-matrix-row-ptr ,v-crs))
              (,element-count (length ,v-data)))
         (unless (= ,element-count (length ,v-col-ind))
           (error "Mismatched column indices and values arrays in CRS matrix"))
         (unless (= (1+ ,row-count) (length ,v-row-ptr))
           (error "Mismatched row count and row pointer array length"))
         ;; TODO: check elements?
         (with-pointer-to-vector-data (,values ,v-data)
           (with-pointer-to-vector-data (,col-indices ,v-col-ind)
             (with-pointer-to-vector-data (,row-ptr ,v-row-ptr)
               (,fun ,row-count ,col-count ,element-count ,values ,col-indices ,row-ptr))))))))

;; (crs-matrix '((0 4)
;;               (0 1 5)
;;               (1 2 3)
;;               (0 2 3 4)
;;               (1 3 4 5)
;;               (1 4 5))

;;             '((10 -2)
;;               (3 9 3)
;;               (7 8 7)
;;               (3 8 7 5)
;;               (8 9 9 13)
;;               (4 2 -1)))
