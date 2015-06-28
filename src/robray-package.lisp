(defpackage :robray
  (:use :cl :alexandria :amino :sycamore :sycamore-util)
  (:export

   ;; Frame Types
   :scene-frame-fixed
   :scene-frame-revolute
   :scene-frame-prismatic

   ;; Geometry
   :scene-box
   :scene-sphere
   :scene-cylinder
   :scene-cone


   ;; Draw interface
   :draw-options
   :draw-options-default
   :merge-draw-options
   :draw-option

   :draw-tf-axis

   :item-cylinder-axis
   :item-cone-axis
   :item-frame-axis

   ;; Render options
   :*render-options*
   :get-render-option
   :render-options-default
   :render-options
   :merge-render-options
   :render-options-fast
   :render-options-medium
   :render-options-full-hd

   ;; Scene Graph Manipulation
   :scene-graph
   :scene-graph-f
   :scene-graph-remove-frame
   :scene-graph-reparent
   :load-scene-file

   ;; variables
   :*render-host-alist
   :*urdf-package-alist
   ))
