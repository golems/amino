(in-package :robray)


(defstruct keyframe
  time)

(defstruct (joint-keyframe (:include keyframe))
  configurations)

(defun alist-fixup-pairs (alist)
  (loop for elt in alist
     for cdr = (cdr elt)
     collect
       (cond
         ((atom cdr) elt)
         ((null (cdr cdr))
          (cons (car elt)
                (car cdr)))
         (t (error "Assocation list entry ~A is not a pair" elt)))))



(defun joint-keyframe (time configurations)
  (make-joint-keyframe
   :time time
   :configurations
   (etypecase configurations
     (list (alist-tree-map (alist-fixup-pairs configurations)
                           #'string-compare))
     (tree-map  configurations)
     (hash-table (hash-table-tree-map configurations #'string-compare)))))

(defun keyframe-pair-compare (a b)
  (let ((a-start (keyframe-time (car a)))
        (a-end (keyframe-time (cdr a)))
        (b-start (keyframe-time (car b)))
        (b-end (keyframe-time (cdr b))))
    (assert (<= a-start a-end))
    (assert (<= b-start b-end))
    (cond
      ((or (< a-end b-start)
           (and (= a-end b-start)
                (< a-start a-end)))
       -1)
      ((or (< b-end a-start)
           (and (= b-end a-start)
                (< b-start b-end)))
       1)
      (t 0))))

(defun keyframe-set (keyframes)
  ;; TODO: fill in unspecified variables
  (let* ((keyframes (sort (copy-list keyframes) #'< :key #'keyframe-time))
         (keyframe-pairs (loop for rest on keyframes
                            while (cdr rest)
                            collect (cons (first rest) (second rest)))))
    (apply #'tree-set #'keyframe-pair-compare keyframe-pairs)))

(defun keyframe-pair-find (keyframes time)
  (let* ((dummy-keyframe (make-keyframe :time time))
         (dummy-pair (cons dummy-keyframe dummy-keyframe)))
    (declare (dynamic-extent dummy-pair))
    (tree-set-find keyframes dummy-pair)))


(defun linterp (time t0 x0 t1 x1)
  (+ x0 (* time (/ (- x1 x0)
                   (- t1 t0)))))

(defun keyframe-configuration-function (keyframes)
  (let ((set (keyframe-set keyframes)))
    (lambda (time)
      (let* ((pair (or (keyframe-pair-find set time)
                       (tree-set-max set)))
             (start-keyframe (car pair))
             (start-map (joint-keyframe-configurations start-keyframe))
             (start-time (keyframe-time start-keyframe))
             (end-keyframe (cdr pair))
             (end-map (joint-keyframe-configurations end-keyframe))
             (end-time (keyframe-time end-keyframe)))
        (fold-tree-map (lambda (map name end-value)
                         (let ((start-value (tree-map-find start-map name 0d0)))
                           (tree-map-insert map name
                                            (linterp time
                                                     start-time start-value
                                                     end-time end-value))))
                       (make-tree-map #'string-compare)
                       end-map)))))




;; (defun animate-timed-frame-function (&key
;;                                        (time-start 0d0)
;;                                        (frames-per-second *frames-per-second*))
;;   (let ((frame-period (coerce (/ 1 frames-per-second) 'double-float)))
;;     (lambda (frame-index)
;;       (let ((frame-number (+ frame-start frame-index)))
;;         (values frame-number (+ time-start (* frame



(defun scene-graph-animate (configuration-function
                            &key
                              (output-directory (robray-cache-directory "animation/"))
                              (time-start 0d0)
                              (time-end 1d0)
                              (scene-graph *scene-graph*)
                              (width *width*)
                              (height *height*)
                              (frames-per-second *frames-per-second*)
                              include
                              )
  (declare (ignore width height))
  (ensure-directories-exist output-directory)
  (let* ((frame-period (coerce (/ 1 frames-per-second) 'double-float)))
    (format t "~&period:     ~A" frame-period)
    (loop
       for frame from 0
       for time = (+ time-start (* frame frame-period))
       while (<= time time-end)
       for configuration = (funcall configuration-function time)
       do
         (scene-graph-pov-frame scene-graph configuration
                                :output (format nil "frame-~D.pov" frame)
                                :directory output-directory
                                :include include))))

;;; Animation Pipeline
;;; - Given: configuration variables and fixed frames
;;; - Find: Relative TF Tree, Absolute Transforms
