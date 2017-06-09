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
     (list (alist-configuration-map configurations))
     (tree-map  configurations)
     (hash-table (hash-table-tree-map configurations #'frame-name-compare)))))

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

(defun keyframe-set (keyframes &key
                                 clip-start)
  ;; TODO: fill in unspecified variables
  (let* ((keyframes (loop for rest on (sort (copy-list keyframes) #'< :key #'keyframe-time)
                       while (and clip-start
                                  (configuration-map-equal (joint-keyframe-configurations (first rest))
                                                           (joint-keyframe-configurations (second rest))))
                       finally (return rest)))
         (keyframe-pairs (loop for rest on keyframes
                            while (cdr rest)
                            collect (cons (first rest) (second rest)))))
    (apply #'tree-set #'keyframe-pair-compare keyframe-pairs)))

(defun keyframe-set-start (keyframes)
  (joint-keyframe-time (car (tree-set-min keyframes))))

(defun keyframe-set-end (keyframes)
  (joint-keyframe-time (cdr (tree-set-max keyframes))))

(defun keyframe-set-end-config (keyframes)
  (joint-keyframe-configurations (cdr (tree-set-max keyframes))))

(defun keyframe-pair-find (keyframes time)
  (let* ((dummy-keyframe (make-keyframe :time time))
         (dummy-pair (cons dummy-keyframe dummy-keyframe)))
    (declare (dynamic-extent dummy-pair))
    (tree-set-find keyframes dummy-pair)))



(defun linterp (time t0 x0 t1 x1)
  (+ x0 (* (- time t0)
           (/ (- x1 x0)
              (- t1 t0)))))

(defun keyframe-configuration-function (keyframes &key
                                                    (time-offset 0d0))
  (let ((set (etypecase keyframes
               (list (keyframe-set keyframes))
               (tree-set keyframes))))
    (lambda (time)
      (let* ((time (+ time time-offset))
             (pair (keyframe-pair-find set time)))
        (when pair
          ;(print pair)
          (let* ((start-keyframe (car pair))
                 (start-map (joint-keyframe-configurations start-keyframe))
                 (start-time (keyframe-time start-keyframe))
                 (end-keyframe (cdr pair))
                 (end-map (joint-keyframe-configurations end-keyframe))
                 (end-time (keyframe-time end-keyframe)))
            (assert (>= time start-time))
            (assert (<= time end-time))
            (fold-tree-map (lambda (map name end-value)
                             (let ((start-value (tree-map-find start-map name 0d0)))
                               (tree-map-insert map name
                                                (linterp time
                                                         start-time start-value
                                                         end-time end-value))))
                           (make-tree-map #'frame-name-compare)
                           end-map)))))))

(defun load-keyframes (path &key
                              frames
                              (initial-time 0d0)
                              time-step
                              default-map)
  (keyframe-set
   (loop for x in (load-trajectory-data path)
      for time = initial-time then (+ time time-step)
      collect (joint-keyframe time (pairlist-configuration-map frames x default-map)))))


;; (defun animate-timed-frame-function (&key
;;                                        (time-start 0d0)
;;                                        (frames-per-second *frames-per-second*))
;;   (let ((frame-period (coerce (/ 1 frames-per-second) 'double-float)))
;;     (lambda (frame-index)
;;       (let ((frame-number (+ frame-start frame-index)))
;;         (values frame-number (+ time-start (* frame


(defun frame-files (directory)
  (directory (merge-pathnames (make-pathname :name :wild
                                             :type "pov")
                              directory)))

(defun image-files (directory)
  (directory (merge-pathnames (make-pathname :name :wild
                                             :type "png")
                              directory)))

(defun last-frame-number (files)
  (frame-number (car (last (sort-frames files)))))

(defun codec-avconv-arg (codec)
  (ecase codec
    (:x264 "libx264")
    (:theora "libtheora")
    (:vp8 "libvpx")))

(defun codec-container (codec)
  (ecase codec
    (:x264 "mp4")
    (:theora "ogv")
    (:vp8 "webm")))

(defun animate-encode (&key
                         (options *render-options*)
                         (directory *robray-tmp-directory*)
                         (pad 15)
                         (output-file "robray"))
  ;; pad
  (let* ((images (directory (merge-pathnames "frame-*.png" directory)))
         (last-frame-number (frame-number (car (last (sort-frames images))))))
    (map nil #'delete-file (directory (merge-pathnames "vframe-*.png" directory)))
    (loop for file in images
       for n = (frame-number file)
       do (sb-ext:run-program "ln"
                              (list "-v"
                                    (namestring file)
                                    (format nil "vframe-~D.png" n))
                              :search t :wait t :directory directory
                              :output *standard-output*))
    (loop
       for i from 1 below pad
       for n = (+ i last-frame-number)
       ;until (zerop (mod n frames-per-second))
       do
         (sb-ext:run-program "cp"
                             (list "-v"
                                   (format nil "frame-~D.png" last-frame-number)
                                   (format nil "vframe-~D.png" n))
                             :search t :wait t :directory directory
                             :output *standard-output*)))
  ;; avconv -f image2 -r 30 -i frame-%d.png foo.mp4
  (let* ((codec (draw-option options :codec))
         (args (list "-f" "image2"
                     "-r" (write-to-string (get-render-option options :frames-per-second))
                     "-i" "vframe-%d.png"
                     "-codec:v" (codec-avconv-arg codec)
                     "-loglevel" "warning"
                     "-y"
                     (rope-string (rope output-file "." (codec-container codec))))))
    (format t "~&avconv ~{~A ~}" args)
    (sb-ext:run-program "avconv"
                        args
                        :search t :wait t :directory directory
                        :output *standard-output*)
    (map nil #'delete-file (directory  (merge-pathnames "vframe-*.png" directory)))
    (format t "~&done")))


(defun animate-render (&key
                         (directory *robray-tmp-directory*)
                         (options *render-options*)
                         (wait t)
                         (jobs 16)
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
         (semaphore (when wait (sb-thread:make-semaphore :count 0)))
         (pov-args (pov-args "FILE" :options options :other (list "+WT1" ))))
    (format status-stream "~&povray ~{~A ~}" pov-args)
    (labels ((process-file (file)
               (format status-stream "~&~A Starting render of ~A" (percent) file)
               (push (sb-ext:run-program "povray" (cons (namestring file) (cdr pov-args))
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


(defun finish-render (&key output-directory render-frames encode-video options)
  ;; Convert Frames
  (when render-frames
    (load-config)
    (net-render :directory output-directory
                :options options)
    ;; Encode Video
    (when encode-video
      (animate-encode :directory output-directory
                      :options options))))

(defun scene-graph-frame-animate (frame-configuration-function
                                  &key
                                    (camera-tf (tf nil))
                                    (render-frames t)
                                    (encode-video t)
                                    (output-directory *robray-tmp-directory*)
                                    append
                                    (scene-graph *scene-graph*)
                                    (options *render-options*)
                                    include
                                    include-text
                                    )
  (ensure-directories-exist output-directory)

  ;; Write frames
  (loop
     for frame-arg from 0
     for frame-file from (let ((frame-files (frame-files output-directory)))
                           (if append
                               (if frame-files
                                   (1+ (last-frame-number frame-files))
                                   0)
                               (progn
                                 (map nil #'delete-file frame-files)
                                 0)))
     for configuration = (funcall frame-configuration-function frame-arg)
     while configuration
     do
       (let ((frame-file (format nil "frame-~D.pov" frame-file)))
         ;(print frame-file)
         ;(print configuration)
         (render-scene-graph scene-graph
                             :camera-tf camera-tf
                             :options options
                             :configuration-map configuration
                             :output frame-file
                             :directory output-directory
                             :include include
                             :include-text include-text
                             )))
  (finish-render :output-directory output-directory
                 :render-frames render-frames
                 :encode-video encode-video
                 :options options))


(defun scene-graph-time-animate (configuration-function
                                 &key
                                   append
                                   (render-frames t)
                                   (encode-video t)
                                   (output-directory *robray-tmp-directory*)
                                   (time-start 0d0)
                                   ;(time-end 1d0)
                                   (scene-graph *scene-graph*)
                                   (options *render-options*)
                                   include
                                   (camera-tf (tf nil))
                                   include-text)
  (let ((frame-period (/ 1 (coerce (get-render-option options :frames-per-second) 'double-float))))
    (scene-graph-frame-animate (lambda (frame)
                                 (let ((time (+ time-start (* frame frame-period))))
                                   ;;(format t "~&f: ~D, t: ~F" frame time)
                                   (funcall configuration-function time)))
                               :append append
                               :scene-graph scene-graph
                               :render-frames render-frames
                               :encode-video encode-video
                               :output-directory output-directory
                               :options options
                               :include include
                               :include-text include-text
                               :camera-tf camera-tf
                               )))

(defun net-process-host (compute-available semaphore)
  (loop do
     ;; Try to find it
       (maphash (lambda (host jobs-availble)
                  (unless (zerop jobs-availble)
                    (if (= 1 jobs-availble)
                        (remhash host compute-available)
                        (decf (gethash host compute-available)))
                    (return-from net-process-host host)))
                compute-available)
     ;; Else wait
       (sb-thread:wait-on-semaphore semaphore)))

(defun net-process (semaphore compute-available work-queue work-generator)
  "WORK-GENERATOR: (lambda (host item)) => (lambda (release-host-thunk))"
  (let ((active-jobs 0))
    ;; Start jobs
    (loop
       for item = (pop work-queue)
       while item
       for host = (net-process-host compute-available semaphore)
       do (let ((host host))
            (incf active-jobs)
            (funcall (funcall work-generator host item)
                     (lambda ()
                       (decf active-jobs)
                       (incf (gethash host compute-available 0))
                       (sb-thread:signal-semaphore semaphore)))))
    ;; Wait  for jobs to finish
    (loop until (zerop active-jobs)
       do (sb-thread:wait-on-semaphore semaphore))))


(defstruct net-args
  host
  povray
  jobs
  threads
  nice)

(defun net-args (host &key (jobs 1) (threads 1) (nice 0) (povray "povray"))
  (make-net-args :host host
                 :jobs jobs
                 :threads threads
                 :povray povray
                 :nice nice))



(defun frame-number (item)
  (parse-integer (ppcre:regex-replace "^.*-([0-9]+).(pov|png)$" (namestring item) "\\1")))

(defun sort-frames (frames)
  (sort frames #'< :key #'frame-number))

(defun net-render (&key
                     (directory *robray-tmp-directory*)
                     (options *render-options*)
                     (status-stream *standard-output*))
  (load-config)
  (let* ((render-host-alist *render-host-alist*)
         (compute-available (make-string-hash-table))
         (host-args (make-string-hash-table))
         (work-queue (sort-frames (frame-files directory)))
         (semaphore (sb-thread:make-semaphore :count 0))
         (n (length work-queue))
         (i 0))
    (map nil #'delete-file (image-files directory))
    ;; TODO: better directory use
    ;; - proper temp directories on remote hosts
    ;; -
    (labels ((povfile-base (item)
               (concatenate 'string
                            (pathname-name item) "." (pathname-type item)))
             (percent () (format nil "[~3D%]" (floor (* 100 (/ i n)))))
             (my-pov-args (host item)
               (let ((args (gethash host host-args)))
                 (pov-args (povfile-base item)
                           :output (unless (string-equal host "localhost") "-")
                           :options options
                           :threads (net-args-threads args))))
             (pov-error (item)
               (merge-pathnames (make-pathname :name (pathname-name item)
                                               :type "txt")
                                directory))
             (pov-local (item status-hook)
               (let ((args (gethash "localhost" host-args)))
                 (sb-ext:run-program "nice"
                                     (list* "-n"
                                            (format nil "~D" (net-args-nice args))
                                            (net-args-povray args)
                                            (my-pov-args "localhost" item))
                                     :search t :wait nil
                                     :error (pov-error item)
                                     :if-error-exists :supersede
                                     :directory directory
                                     :status-hook status-hook)))
             (pov-remote (host item status-hook)
               (let ((args (gethash host host-args))
                     (output (concatenate 'string
                                          (namestring directory)
                                          (pov-output-file (povfile-base item)))))
                 (sb-ext:run-program (find-script "povremote")
                                     (list* host
                                            (net-args-povray args)
                                            (format nil "~D" (net-args-nice args))
                                            (my-pov-args host item))
                                     :search nil :wait nil
                                     :directory directory
                                     :error (pov-error item)
                                     :if-error-exists :supersede
                                     :output output
                                     :status-hook status-hook
                                     :if-output-exists :supersede)))
             (work-generator (host item)
               (lambda (release-host-thunk)
                 (format status-stream "~&~A ~A ~A(~A)"
                         (percent) (povfile-base item) #\Tab host )
                 (incf i)
                 (flet ((status-hook (process)
                          (declare (ignore process))
                          ;(format status-stream "~&~A ~A finished ~A" (percent) host item)
                          (funcall release-host-thunk)))
                   (if (string= host "localhost")
                     (pov-local item #'status-hook)
                     (pov-remote host item #'status-hook)))))

             (rsync-inc (host jobs)
               (format status-stream "~&Rsyncing to ~A" host)
               (sb-ext:run-program "rsync"
                                   (list "-r"
                                         "--include=*.inc"
                                         "--include=*.pov"
                                         "--include=povray/**.png"
                                         "--include=povray/**.tiff"
                                         "--include=povray/**.jpeg"
                                         "--include=povray/**.png"
                                         "--include=povray/**.ppm"
                                         "--include=povray/**.pgm"
                                         "--include=povray/**.sys"
                                         "--include=**/"
                                         "--exclude=*"
                                         (namestring directory)
                                         (format nil "~A:~A" host (namestring *robray-tmp-directory*)))
                                   :search t :wait nil
                                   :output status-stream
                                   :error *error-output*
                                   :status-hook (lambda (process)
                                                  (declare (ignore process))
                                                  (format status-stream "~&Finished rsync to ~A" host)
                                                  (setf (gethash host compute-available) jobs)
                                                  (sb-thread:signal-semaphore semaphore)))))

      ;; Initialize Hosts
      (dolist (host-vars render-host-alist)
        (let* ((net-args (apply #'net-args host-vars))
               (host (net-args-host net-args))
               (jobs (net-args-jobs net-args)))
          (setf (gethash host host-args) net-args)
          (format status-stream "~&~A: povray ~{~A~^ ~}" host (my-pov-args host (elt work-queue 0)))
          (if (string-equal host "localhost")
              (setf (gethash host compute-available)
                    jobs)
              (rsync-inc host jobs))))
      ;; Process
      (net-process semaphore compute-available work-queue #'work-generator))))
    ;;   ;; start jobs
    ;;   (loop
    ;;      for i below jobs
    ;;      while file-worklist
    ;;      do
    ;;        (process-file (pop file-worklist))))
    ;;   ;; Maybe wait
    ;; (when semaphore
    ;;   (sb-thread:wait-on-semaphore semaphore))))






;;; This version is slower because the file is reparsed for each
;;; frame, and the re-parsing is serialized
;; (defun scene-graph-animate-2 (configuration-function
;;                             &key
;;                               (render-frames t)
;;                               (encode-video t)
;;                               (output-directory *robray-tmp-directory*)
;;                               (pov-file "frame")
;;                               (time-start 0d0)
;;                               (time-end 1d0)
;;                               (scene-graph *scene-graph*)
;;                               (width *width*)
;;                               (height *height*)
;;                               (frames-per-second *frames-per-second*)
;;                               include)
;;   (declare (ignore width height))
;;   (ensure-directories-exist output-directory)
;;   (map nil #'delete-file (frame-files output-directory))
;;   ;; Write pov
;;   (let ((frame-period (coerce (/ 1 frames-per-second) 'double-float))
;;         (frame-end (round (* (- time-end time-start)
;;                              frames-per-second))))
;;     ;; POV file
;;     (scene-graph-pov-frame scene-graph
;;                            :frame-start 0
;;                            :frame-end (round (* (- time-end time-start)
;;                                                 frames-per-second))
;;                            :frame-configuration-function (lambda (frame)
;;                                                            (funcall configuration-function
;;                                                                     (+ time-start (* frame frame-period))))
;;                            :output (format nil "~A.pov" pov-file)
;;                            :directory output-directory
;;                            :include include)

;;     ;; Run POV
;;     (when render-frames
;;       (sb-ext:run-program "povray" (pov-args (concatenate 'string output-directory
;;                                                           pov-file ".pov")
;;                                              :quality .5
;;                                              :other (list (format nil "+KFF~D" frame-end)))
;;                           :search t :wait t))))


;;; Animation Pipeline
;;; - Given: configuration variables and fixed frames
;;; - Find: Relative TF Tree, Absolute Transforms
