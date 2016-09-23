;;;; -*- mode: lisp -*-
;;;;
;;;; Copyright (c) 2015, Rice University
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

(progn
  (in-package :robray)

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
  (include "amino/rx.h")

  ;; Types
  (ctype rx-frame-id "aa_rx_frame_id")
  (ctype rx-config-id "aa_rx_config_id")

  ;; Frames
  (constant (+frame-id-root+ "AA_RX_FRAME_ROOT")
            :type integer)
  (constant (+config-id-none+ "AA_RX_CONFIG_NONE")
            :type integer)
  (cenum frame-type
         ((:frame-fixed "AA_RX_FRAME_FIXED"))
         ((:frame-revolute "AA_RX_FRAME_REVOLUTE"))
         ((:frame-prismatic "AA_RX_FRAME_PRISMATIC")))

  ;; Geometry
  (cenum ref-op
         ((:steal "AA_MEM_STEAL"))
         ((:borrow "AA_MEM_BORROW"))
         ((:copy "AA_MEM_COPY")))

  ;; Geometry
  (cenum geom-shape
         ((:no-shape "AA_RX_NOSHAPE"))
         ((:mesh "AA_RX_MESH"))
         ((:box "AA_RX_BOX"))
         ((:sphere "AA_RX_SPHERE"))
         ((:cylinder "AA_RX_CYLINDER"))
         ((:cone "AA_RX_CONE"))
         ((:grid "AA_RX_GRID")))


  (cstruct shape-box "struct aa_rx_shape_box"
           (dimension "dimension" :type :double :count 3))
  (cstruct shape-sphere "struct aa_rx_shape_sphere"
           (radius "radius" :type :double))
  (cstruct shape-cylinder "struct aa_rx_shape_cylinder"
           (radius "radius" :type :double)
           (height "height" :type :double))
  (cstruct shape-cone "struct aa_rx_shape_cone"
           (start-radius "start_radius" :type :double)
           (end-radius "end_radius" :type :double)
           (height "height" :type :double))
  (cstruct shape-grid "struct aa_rx_shape_grid"
           (dimension "dimension" :type :double :count 2)
           (delta "delta" :type :double :count 2)
           (width "width" :type :double))
  )
