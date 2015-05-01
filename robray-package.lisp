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
   :get-draw-option

   :draw-geometry
   :draw-tf-axis

   :draw-items
   :item-cylinder-axis
   :item-cone-axis
   :item-frame-axis

   ;; Scene Graph Manipulation
   :scene-graph :scene-graph-add-frame :scene-graph-remove-frame :scene-graph-add-visual
   :scene-graph-reparent
   ))
