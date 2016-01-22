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

(defmacro def-opt-create (name)
  `(defcfun ,name
       opt-cx-t
     (m size-t)
     (n size-t)
     (A-values :pointer)
     (A-cols :pointer)
     (A-row-ptr :pointer)
     (b-lower :pointer)
     (b-upper :pointer)
     (c :pointer)
     (x-lower :pointer)
     (x-upper :pointer)))

(def-opt-create aa-opt-lpsolve-crscreate)
(def-opt-create aa-opt-clp-crscreate)
(def-opt-create aa-opt-glpk-crscreate)


(defcfun aa-opt-solve :int
  (cx opt-cx-t)
  (n size-t)
  (x :pointer))


(defun opt-create-function (solver)
  (ecase solver
    (:clp #'aa-opt-clp-crscreate)
    (:glpk #'aa-opt-glpk-crscreate)
    (:lpsolve #'aa-opt-lpsolve-crscreate)))


(defun opt-create (a-crs b-lower b-upper c x-lower x-upper
                   &key (solver :clp))
  (let ((r)
        (solver (opt-create-function solver)))
    (amino-type::with-foreign-crs (m n e a-ptr col-ind row-ptr) a-crs
      (with-foreign-simple-vector (b-lower n-b-lower) b-lower :input
        (with-foreign-simple-vector (b-upper n-b-upper) b-upper :input
          (with-foreign-simple-vector (c n-c) c :input
            (with-foreign-simple-vector (x-lower n-x-lower) x-lower :input
              (with-foreign-simple-vector (x-upper n-x-upper) x-upper :input
                (assert (= m n-b-lower n-b-upper))
                (assert (= n n-c n-x-lower n-x-upper))
                (sb-int:with-float-traps-masked (:divide-by-zero :overflow)
                  (setq r
                        (funcall solver
                                 m n
                                 a-ptr col-ind row-ptr
                                 b-lower b-upper
                                 c
                                 x-lower x-upper)))))))))
    r))
