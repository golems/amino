(in-package :robray)


(defun load-trajectory-data (place)
  (etypecase place
    (rope (with-open-file (stream (rope-string place) :direction :input)
            (load-trajectory-data stream)))
    (stream
     (loop for line = (read-line place nil nil)
        while line
        collect (parse-float-sequence line)))))


(defun %load-trajectory (stream names time time-step default-map clip-start)
  ;; find header
  (let ((lineno 0))
    (unless names
      (block find-header
        (loop for line = (progn (incf lineno)
                                (read-line stream nil nil))
           while line
           for line2 = (strip-hash-comment line)
           do (when line2
                (setq names (split-spaces line2))
                (return-from find-header))))
      (when (equalp (car names) "time")
        (setq names (cdr names))))
    ;; load data
    (keyframe-set
     (loop for line = (progn (incf lineno)
                             (read-line stream nil nil))
        for i from 0
        while line
        for line2 = (strip-hash-comment line)
        when line2
        collect (let ((line-data (parse-float-sequence line2)))
                  (multiple-value-bind (time config)
                      (if time-step
                          (values (+ time (* i time-step))
                                  line-data)
                          (values (car line-data)
                                  (cdr line-data)))
                    (joint-keyframe time
                                    (pairlist-configuration-map names config default-map))))))
    :clip-start clip-start))

(defun load-trajectory (place &key names default-map (time 0d0) time-step clip-start)
  (labels ((helper (stream)
             (%load-trajectory stream names time time-step default-map clip-start)))
    (etypecase place
      (rope
       (with-open-file (stream (rope-string place) :direction :input)
         (helper stream)))
      (stream
       (helper place)))))
