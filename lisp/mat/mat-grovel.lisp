;;;; -*- mode: lisp -*-
;;;;
;;;; Copyright (c) 2018, Colorado School of Mines
;;;; All rights reserved.
;;;;
;;;; Author(s): Neil T. Dantam <ndantam@mines.edu>
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

(progn
  (in-package :amino)

  ;; Look for source directory includes
  (cc-flags #.(concatenate 'string "-I"
                           (namestring (asdf:system-source-directory :amino))
                           "../include")
            #.(when (boundp 'cl-user::*top-srcdir*)
                (concatenate 'string "-I"
                             cl-user::*top-srcdir*
                             "/include"))
            #.(when (boundp 'cl-user::*top-builddir*)
                (concatenate 'string "-I"
                             cl-user::*top-builddir*
                             "/include"))
            "-std=gnu99")
  (include "amino.h")
  (include "amino/mat.h")

  ;; Types
  (ctype aa-lb-size "aa_lb_size")

  (cenum transpose-t
         ((:tranpose "CblasTrans"))
         ((:no-transpose "CblasNoTrans")))

  ;; Structs
  (cstruct aa-dvec "struct aa_dvec"
           (len"len" :type aa-lb-size)
           (data "data" :type :pointer)
           (inc"inc" :type aa-lb-size))



  (cstruct aa-dmat "struct aa_dmat"
           (rows "rows" :type aa-lb-size)
           (cols "cols" :type aa-lb-size)
           (data "data" :type :pointer)
           (ld "ld" :type aa-lb-size)))
