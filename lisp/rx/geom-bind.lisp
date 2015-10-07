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

(in-package :robray)

;;;;;;;;;;;;;;;;;;;;;;;;
;;; Geometry Options ;;;
;;;;;;;;;;;;;;;;;;;;;;;;

(cffi:defcfun aa-rx-geom-opt-destroy :void
  (value :pointer))

(cffi:defcfun aa-rx-geom-opt-create rx-geom-opt-t)

(defun make-geom-opt ()
  (amino-ffi::foreign-container-finalizer (aa-rx-geom-opt-create)
                                          #'%make-rx-geom-opt
                                          #'aa-rx-geom-opt-destroy))

(cffi:defcfun aa-rx-geom-opt-set-no-shadow :void
  (opts rx-geom-opt-t)
  (value :boolean))

(cffi:defcfun aa-rx-geom-opt-set-collision :void
  (opts rx-geom-opt-t)
  (value :boolean))

(cffi:defcfun aa-rx-geom-opt-set-visual :void
  (opts rx-geom-opt-t)
  (value :boolean))

(cffi:defcfun aa-rx-geom-opt-set-color3 :void
  (opts rx-geom-opt-t)
  (r amino-ffi::coercible-double)
  (g amino-ffi::coercible-double)
  (b amino-ffi::coercible-double))

(cffi:defcfun aa-rx-geom-opt-set-specular3 :void
  (opts rx-geom-opt-t)
  (r amino-ffi::coercible-double)
  (g amino-ffi::coercible-double)
  (b amino-ffi::coercible-double))

(cffi:defcfun aa-rx-geom-opt-set-alpha :void
  (opts rx-geom-opt-t)
  (a amino-ffi::coercible-double))


(cffi:defcfun aa-rx-geom-opt-get-no-shadow :boolean
  (opts rx-geom-opt-t))
(cffi:defcfun aa-rx-geom-opt-get-visual :boolean
  (opts rx-geom-opt-t))
(cffi:defcfun aa-rx-geom-opt-get-collision :boolean
  (opts rx-geom-opt-t))

(cffi:defcfun aa-rx-geom-opt-get-color-red :double
  (opts rx-geom-opt-t))
(cffi:defcfun aa-rx-geom-opt-get-color-blue :double
  (opts rx-geom-opt-t))
(cffi:defcfun aa-rx-geom-opt-get-color-green :double
  (opts rx-geom-opt-t))
(cffi:defcfun aa-rx-geom-opt-get-alpha :double
  (opts rx-geom-opt-t))

(cffi:defcfun aa-rx-geom-opt-get-specular-red :double
  (opts rx-geom-opt-t))
(cffi:defcfun aa-rx-geom-opt-get-specular-blue :double
  (opts rx-geom-opt-t))
(cffi:defcfun aa-rx-geom-opt-get-specular-green :double
  (opts rx-geom-opt-t))


(defun rx-geom-opt-color (opt)
  (vec (aa-rx-geom-opt-get-color-red opt)
       (aa-rx-geom-opt-get-color-blue opt)
       (aa-rx-geom-opt-get-color-green opt)))

(defun rx-geom-opt-specular (opt)
  (vec (aa-rx-geom-opt-get-specular-red opt)
       (aa-rx-geom-opt-get-specular-blue opt)
       (aa-rx-geom-opt-get-specular-green opt)))

(defun rx-geom-opt-alist (opt)
  `((:color . ,(rx-geom-opt-color opt))
    (:alpha .  ,(aa-rx-geom-opt-get-alpha opt))
    (:specular . ,(rx-geom-opt-specular opt))
    (:visual . ,(aa-rx-geom-opt-get-visual opt))
    (:collision . ,(aa-rx-geom-opt-get-collision opt))
    (:no-shadow . ,(aa-rx-geom-opt-get-no-shadow opt))))

(defmethod print-object ((object rx-geom-opt) stream)
  (print-unreadable-object (object stream :type t)
    (print (rx-geom-opt-alist object) stream)))

(defun alist-rx-geom-opt (alist)
  (let ((opt (aa-rx-geom-opt-create)))
    (when-let ((a (assoc :color alist)))
      (with-vec3 (r g b) (cdr a)
        (aa-rx-geom-opt-set-color3 opt r g b)))
    (when-let ((a (assoc :alpha alist)))
      (aa-rx-geom-opt-set-alpha opt (cdr a)))
    (when-let ((a (assoc :specular alist)))
      (with-vec3 (r g b) (cdr a)
        (aa-rx-geom-opt-set-specular3 opt r g b)))
    (when-let ((a (assoc :visual alist)))
      (aa-rx-geom-opt-set-visual opt (cdr a)))
    (when-let ((a (assoc :collision alist)))
      (aa-rx-geom-opt-set-collision opt (cdr a)))
    (when-let ((a (assoc :no-shadow alist)))
      (aa-rx-geom-opt-set-no-shadow opt (cdr a)))
    opt))

;;;;;;;;;;;;;;;;
;;; Geometry ;;;
;;;;;;;;;;;;;;;;

(cffi:defcfun aa-rx-geom-destroy :void
  (geom :pointer))

(cffi:defcfun aa-rx-geom-box rx-geom-t
  (opt rx-geom-opt-t)
  (dimension amino::vector-3-t))

(cffi:defcfun aa-rx-geom-sphere rx-geom-t
  (opt rx-geom-opt-t)
  (radius amino-ffi::coercible-double))

(cffi:defcfun aa-rx-geom-cylinder rx-geom-t
  (opt rx-geom-opt-t)
  (height amino-ffi::coercible-double)
  (radius amino-ffi::coercible-double))

(cffi:defcfun aa-rx-geom-cone rx-geom-t
  (opt rx-geom-opt-t)
  (height amino-ffi::coercible-double)
  (start-radius amino-ffi::coercible-double)
  (end-radius amino-ffi::coercible-double))

(cffi:defcfun aa-rx-geom-grid rx-geom-t
  (opt rx-geom-opt-t)
  (dimension amino::vector-2-t)
  (delta amino::vector-2-t)
  (width amino-ffi::coercible-double))

(cffi:defcfun aa-rx-geom-shape :pointer
  (geom rx-geom-t)
  (shape-type :pointer))

(defun rx-geom-shape (geom)
  (cffi:with-foreign-object (shape-type 'geom-shape)
    (let* ((ptr (aa-rx-geom-shape geom shape-type))
           (shape (cffi:mem-ref shape-type 'geom-shape)))
      ;; Do not registor the destructor, memory is owned by the
      ;; containing geometry object.
      (ecase shape
        (:box (%make-rx-shape-box ptr))
        (:sphere (%make-rx-shape-sphere ptr))
        (:cylinder (%make-rx-shape-cylinder ptr))
        (:cone (%make-rx-shape-cone ptr))
        (:grid (%make-rx-shape-grid ptr))))))

;;;;;;;;;;;;;;
;;; Shapes ;;;
;;;;;;;;;;;;;;

;;; Box ;;;
(defun rx-shape-box-x (object)
  (cffi:mem-aref (rx-shape-box-slot-value object 'dimension) :double 0))

(defun rx-shape-box-y (object)
  (cffi:mem-aref (rx-shape-box-slot-value object 'dimension) :double 1))

(defun rx-shape-box-z (object)
  (cffi:mem-aref (rx-shape-box-slot-value object 'dimension) :double 2))

(defmethod print-object ((object rx-shape-box) stream)
  (print-unreadable-object (object stream :type t)
    (format stream "~F ~F ~F (~x)"
            (rx-shape-box-x object)
            (rx-shape-box-y object)
            (rx-shape-box-z object)
            (cffi:pointer-address (rx-shape-box-pointer object)))))

;;; Sphere ;;;
(amino-ffi::def-foreign-container-accessor rx-shape-sphere radius)

(defmethod print-object ((object rx-shape-sphere) stream)
  (print-unreadable-object (object stream :type t)
    (format stream "~F (~x)"
            (rx-shape-sphere-radius object)
            (cffi:pointer-address (amino-ffi::foreign-container-pointer object)))))

;;; Cylinder ;;;
(amino-ffi::def-foreign-container-accessor rx-shape-cylinder radius)
(amino-ffi::def-foreign-container-accessor rx-shape-cylinder height)

(defmethod print-object ((object rx-shape-cylinder) stream)
  (print-unreadable-object (object stream :type t)
    (format stream "height: ~F radius: ~F (~x)"
            (rx-shape-cylinder-height object)
            (rx-shape-cylinder-radius object)
            (cffi:pointer-address (amino-ffi::foreign-container-pointer object)))))

;;; Cone ;;;
(amino-ffi::def-foreign-container-accessor rx-shape-cone height)
(amino-ffi::def-foreign-container-accessor rx-shape-cone start-radius)
(amino-ffi::def-foreign-container-accessor rx-shape-cone end-radius)
(defmethod print-object ((object rx-shape-cone) stream)
  (print-unreadable-object (object stream :type t)
    (format stream "height: ~F start-radius: ~F end-radius: ~F (~x)"
            (rx-shape-cone-height object)
            (rx-shape-cone-start-radius object)
            (rx-shape-cone-end-radius object)
            (cffi:pointer-address (amino-ffi::foreign-container-pointer object)))))


;;; Grid ;;;
(defun rx-shape-grid-x (object)
  (cffi:mem-aref (rx-shape-grid-slot-value object 'dimension) :double 0))
(defun rx-shape-grid-y (object)
  (cffi:mem-aref (rx-shape-grid-slot-value object 'dimension) :double 1))

(defun rx-shape-grid-delta-x (object)
  (cffi:mem-aref (rx-shape-grid-slot-value object 'delta) :double 0))
(defun rx-shape-grid-delta-y (object)
  (cffi:mem-aref (rx-shape-grid-slot-value object 'delta) :double 1))
(amino-ffi::def-foreign-container-accessor rx-shape-grid width)

;;;;;;;;;;;;
;;; Mesh ;;;
;;;;;;;;;;;;

(cffi:defcfun aa-rx-mesh-create :pointer)
(cffi:defcfun aa-rx-mesh-destroy :void
  (mesh :pointer))

(cffi:defcfun aa-rx-mesh-set-vertices :void
  (mesh rx-mesh-t)
  (n amino-ffi:size-t)
  (vectors :float)
  (copy :boolean))

(cffi:defcfun aa-rx-mesh-set-normals :void
  (mesh rx-mesh-t)
  (n amino-ffi:size-t)
  (normals :float)
  (copy :boolean))

(cffi:defcfun aa-rx-mesh-set-indices :void
  (mesh rx-mesh-t)
  (n amino-ffi:size-t)
  (indices :unsigned-int)
  (copy :boolean))

(cffi:defcfun aa-rx-mesh-set-rgba :void
  (mesh rx-mesh-t)
  (width amino-ffi:size-t)
  (height amino-ffi:size-t)
  (rgba :uint8)
  (copy :boolean))

(cffi:defcfun aa-rx-mesh-set-texture :void
  (mesh rx-mesh-t)
  (opt rx-geom-opt-t))
