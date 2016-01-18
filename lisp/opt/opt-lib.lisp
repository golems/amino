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
  (:unix (:or "libamino_opt_cgal.so"
              "libamino_opt_cgal.so"
              (:default "libamino_opt_cgal")))
  (t (:default "libamino_opt_cgal")))

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


(use-foreign-library libgmp)
(use-foreign-library libcgal)
(use-foreign-library libamino-opt-cgal)

;; LPSOLVE
(define-foreign-library libamino-opt-lpsolve
  (:unix (:or "libamino_opt_lpsolve.so"
              "libamino_opt_lpsolve.so"
              (:default "libamino_opt_lpsolve")))
  (t (:default "libamino_opt_lpsolve")))

(define-foreign-library liblpsolve
  (:unix (:or "liblpsolve55.so"
              "liblpsolve55.so"
              (:default "liblpsolve55")))
  (t (:default "liblpsolve55")))

(use-foreign-library liblpsolve)
(use-foreign-library libamino-opt-lpsolve)

;; CLP
(define-foreign-library libamino-opt-clp
  (:unix (:or "libamino_opt_clp.so"
              "libamino_opt_clp.so"
              (:default "libamino_opt_clp")))
  (t (:default "libamino_opt_clp")))

(define-foreign-library libclp
  (:unix (:or "liblClp.so"
              "liblClp.so"
              (:default "libClp")))
  (t (:default "libClp")))

(use-foreign-library libclp)
(use-foreign-library libamino-opt-clp)

;; GLPK
(define-foreign-library libamino-opt-glpk
  (:unix (:or "libamino_opt_glpk.so"
              "libamino_opt_glpk.so"
              (:default "libamino_opt_glpk")))
  (t (:default "libamino_opt_glpk")))

(define-foreign-library libglpk
  (:unix (:or "liblglpk.so"
              "liblglpk.so"
              (:default "libglpk")))
  (t (:default "libglpk")))

(use-foreign-library libglpk)
(use-foreign-library libamino-opt-glpk)
