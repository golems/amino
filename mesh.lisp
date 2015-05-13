(in-package :robray)


(defstruct mesh-data
  name
  (vertices  (make-array 0 :element-type 'double-float)
             :type (simple-array double-float (*)))
  (normals  (make-array 0 :element-type 'double-float)
            :type (simple-array double-float (*)))
  (vertex-indices  (make-array 0 :element-type 'fixnum)
                   :type (simple-array fixnum (*)))
  (normal-indices  (make-array 0 :element-type 'fixnum)
                   :type (simple-array fixnum (*)))
  texture-properties
  (texture-indices  (make-array 0 :element-type 'fixnum)
                    :type (simple-array fixnum (*)))
  )
