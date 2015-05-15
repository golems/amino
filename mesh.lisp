(in-package :robray)


(defstruct mesh-data
  name
  (vertex-vectors nil :type (or null (simple-array double-float (*))))
  (vertex-indices nil :type (or null (simple-array fixnum (*))))
  (normal-vectors nil :type (or null (simple-array double-float (*))))
  (normal-indices nil :type (or null (simple-array fixnum (*))))
  (uv-vectors nil :type (or null (simple-array double-float (*))))
  (uv-indices nil :type (or null (simple-array fixnum (*))))
  texture-properties
  (texture-indices nil :type (or null (simple-array fixnum (*)))))
