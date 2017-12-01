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

;; CGAL
(define-foreign-library libamino-opt-cgal
  (:unix (:or "libamino-opt-cgal.so"
              "libamino-opt-cgal.so"
              (:default "libamino-opt-cgal")))
  (t (:default "libamino-opt-cgal")))

(define-foreign-library libcgal
  (:unix (:or "libCGAL.so"
              "libCGAL.so"
              (:default "libCGAL")))
  (t (:default "libCGAL")))

(define-foreign-library libgmp
  (:unix (:or "libgmp.so"
              "libgmp.so"
              (:default "libgmp")))
  (t (:default "libgmp")))

(amino::use-foreign-library-if cgal libgmp libcgal libamino-opt-cgal)

;; LPSOLVE
(define-foreign-library libamino-opt-lpsolve
  (:unix (:or "libamino-opt-lpsolve.so"
              "libamino-opt-lpsolve.so"
              (:default "libamino-opt-lpsolve")))
  (t (:default "libamino-opt-lpsolve")))

(define-foreign-library liblpsolve
  (:unix (:or "liblpsolve55.so"
              "liblpsolve55.so"
              (:default "liblpsolve55")))
  (t (:default "liblpsolve55")))

(amino::use-foreign-library-if lpsolve liblpsolve libamino-opt-lpsolve)

;; CLP
(define-foreign-library libamino-opt-clp
  (:unix (:or "libamino-opt-clp.so"
              "libamino-opt-clp.so"
              (:default "libamino-opt-clp")))
  (t (:default "libamino-opt-clp")))

(define-foreign-library libclp
  (:unix (:or "liblClp.so"
              "liblClp.so"
              (:default "libClp")))
  (t (:default "libClp")))

(amino::use-foreign-library-if clp libclp libamino-opt-clp)

;; GLPK
(define-foreign-library libamino-opt-glpk
  (:unix (:or "libamino-opt-glpk.so"
              "libamino-opt-glpk.so"
              (:default "libamino-opt-glpk")))
  (t (:default "libamino-opt-glpk")))

(define-foreign-library libglpk
  (:unix (:or "liblglpk.so"
              "liblglpk.so"
              (:default "libglpk")))
  (t (:default "libglpk")))

(amino::use-foreign-library-if glpk libglpk libamino-opt-glpk)

;; Type
(amino-ffi::def-foreign-container opt-cx opt-cx-t
  :destructor aa-opt-destroy)
