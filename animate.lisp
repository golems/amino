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


(defun frame-files (directory)
  (directory (concatenate 'string directory "/*.pov")))

(defun animate-encode (&key
                         (directory *robray-tmp-directory*)
                         (frames-per-second *frames-per-second*)
                         (output-file "robray.mp4")
                         (video-codec "libx264"))
  ;; avconv -f image2 -r 30 -i frame-%d.png foo.mp4
  (sb-ext:run-program "avconv"
                      (list "-f" "image2"
                            "-r" (write-to-string frames-per-second)
                            "-i" "frame-%d.png"
                            "-codec:v" video-codec
                            "-loglevel" "warning"
                            "-y"
                            output-file)
                      :search t :wait t :directory directory
                      :output *standard-output*))


(defun animate-render (&key
                         (directory *robray-tmp-directory*)
                         (wait t)
                         (jobs 4)
                         (status-stream t))
  ;; This function is single threaded, multi-processed.  We start
  ;; multiple povray child processes to do rendering work.  When a
  ;; child terminates, the STATUS-HOOK handler starts another child to
  ;; render the next frame.  After we've started the initial round of
  ;; children, we wait on a semaphore.  When the last child finished
  ;; the last frame, it posts on the semaphore.
  (let* ((file-worklist (frame-files directory))
         (n (length file-worklist))
         (i 0)
         (process-list)
         (semaphore (when wait (sb-thread:make-semaphore :count 0))))
    (labels ((process-file (file)
               (format status-stream "~&~A Starting render of ~A" (percent) file)
               (push (sb-ext:run-program "povray" (pov-args file
                                                            :quality .5)
                                         :search t :wait nil
                                         :status-hook (status-hook file))
                     process-list))
             (percent () (format nil "[~3D%]" (round (* 100 (/ i n)))))
             (status-hook (file)
               (lambda (process)
                 (incf i)
                 (format status-stream "~&~A Finished render of ~A" (percent) file)
                 (process-handler process)))
             (process-handler (process)
               ;; TODO: check process result
               (setq process-list (remove process process-list :test #'eq))
               (cond
                 (file-worklist
                  (process-file (pop file-worklist)))
                 ((and (null process-list)
                       semaphore)
                  (sb-thread:signal-semaphore semaphore)))))
      ;; start jobs
      (loop
         for i below jobs
         while file-worklist
         do
           (process-file (pop file-worklist))))
      ;; Maybe wait
    (when semaphore
      (sb-thread:wait-on-semaphore semaphore))))



(defun scene-graph-animate (configuration-function
                            &key
                              (render-frames t)
                              (encode-video t)
                              (output-directory *robray-tmp-directory*)
                              (time-start 0d0)
                              (time-end 1d0)
                              (scene-graph *scene-graph*)
                              (width *width*)
                              (height *height*)
                              (frames-per-second *frames-per-second*)
                              include)
  (declare (ignore width height))
  (ensure-directories-exist output-directory)
  (map nil #'delete-file (frame-files output-directory))
  ;; Write frames
  (loop
     with frame-period = (coerce (/ 1 frames-per-second) 'double-float)
     for frame from 0
     for time = (+ time-start (* frame frame-period))
     while (<= time time-end)
     for configuration = (funcall configuration-function time)
     do
       (let ((frame-file (format nil "frame-~D.pov" frame)))
         (scene-graph-pov-frame scene-graph configuration
                                :output frame-file
                                :directory output-directory
                                :include include)))
  ;; Convert Frames
  (when render-frames
    (animate-render :directory output-directory
                    :wait t)
    ;; Encode Video
    (when encode-video
      (animate-encode :directory output-directory
                      :frames-per-second frames-per-second))))


;;; Animation Pipeline
;;; - Given: configuration variables and fixed frames
;;; - Find: Relative TF Tree, Absolute Transforms
