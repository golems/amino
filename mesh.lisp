(in-package :robray)


(defstruct mesh-data
  name
  (vertices nil :type (or null (simple-array double-float (*))))
  (normals nil :type (or null (simple-array double-float (*))))
  (vertex-indices nil :type (or null (simple-array fixnum (*))))
  (normal-indices nil :type (or null (simple-array fixnum (*))))
  texture-properties
  (texture-indices nil :type (or null (simple-array fixnum (*)))))
