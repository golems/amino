(in-package :robray)

(defun alist-get-default (alist key default-alist)
  (cdr (or (assoc key alist)
           (assoc key default-alist))))

(defun make-string-hash-table ()
  (make-hash-table :test #'equal))

(defmacro string-case (keyform &body cases)
  (let ((value (gensym "value")))
    `(let ((,value ,keyform))
       (cond
         ,@(loop for case in cases
              collect (destructuring-bind (key &rest body) case
                        (cond
                          ((eq key 'otherwise)
                           `(t ,@body))
                          ((listp key)
                           `((or ,@(loop for subkey in key
                                      collect `(string= ,value ,subkey)))
                             ,@body))
                          (t `((string= ,value ,key) ,@body)))))))))


(defun clean-pathname (path &optional (separator "/"))
  (let ((regex (format nil "~A+" separator)))
    (ppcre:regex-replace-all regex (namestring path) separator)))

(declaim (inline format-pathname))
(defun format-pathname (control &rest args)
  (clean-pathname (apply #'format nil control args)))

(defun ensure-directory (path &optional (separator "/"))
  (clean-pathname (concatenate 'string path separator) separator))

(defun file-dirname (path)
  (multiple-value-bind (start end reg-start reg-end)
      (ppcre:scan "^(.*)/[^/]*$" path)
    (declare (ignore end))
    (if start
        (subseq path (aref reg-start 0) (aref reg-end 0))
        nil)))

(defun file-type (path)
  (multiple-value-bind (start end reg-start reg-end)
      (ppcre:scan "^.*\\.([^/\.]+)$" path)
    (declare (ignore end))
    (if start
        (subseq path (aref reg-start 0) (aref reg-end 0))
        nil)))

(defun file-basename (path)
  (multiple-value-bind (start end reg-start reg-end)
      (ppcre:scan "^.*/([^/]+)\\.[^/\.]*$" path)
    (declare (ignore end))
    (if start
        (subseq path (aref reg-start 0) (aref reg-end 0))
        nil)))


(defun output-file (file &optional directory)
  (etypecase directory
    (null file)
    (string (concatenate 'string (ensure-directory directory)
                         file))
    (pathname (output-file file (namestring directory)))
    (list (output-file file (make-pathname :directory directory)))))

(defun file-resolve (path directory)
  ;; All the world's POSIX, dammit
  (if (eq #\/ (schar path 0))
      path
      (output-file path directory)))


(defun output (object place
               &key
                 directory
                 (if-exists :supersede))
  (cond
    ((streamp place)
     (print object place)
     (values))
    ((eq place t)
     (print object *standard-output*)
     (values))
    ((null place)
     object)
    ((ropep place)
     (let ((file (output-file (rope-string place)
                              directory)))
       (ensure-directories-exist file)
       (with-open-file (place file :direction :output :if-exists if-exists)
         (print object place)))
     (values))
    (t (error "Unknown place type: ~A" place))))

(defun name-mangle (identifier)
  "Convert an identifer to a conventional identifier"
  (let ((sub (substitute-if #\_
                            (lambda (x)
                              (or (eq x #\-)
                                  (eq x #\Space)
                                  (eq x #\.)))
                            identifier)))
    (if (amino::char-isdigit (aref sub 0))
        (concatenate 'string "g_" sub)
        sub)))

;;;;;;;;;;;;;;;;;;;
;;; DOM-HELPERS ;;;
;;;;;;;;;;;;;;;;;;;
(defun dom-load (file)
  "Create the DOM for an XML file."
  (cxml:parse-file file (cxml-dom:make-dom-builder)))

(defun dom-select-if (node function &key direct)
  (labels ((collect (node)
             (loop for child across (dom:child-nodes node)
                when (funcall function child)
                collect child))
           (rec (node)
             (nconc (loop for child across (dom:child-nodes node)
                       when (dom:element-p child)
                       nconc (rec child))
                    (collect node))))
    (if direct
        (collect node)
        (rec node))))


(defun dom-singleton (result singleton)
  (if singleton
      (progn
        (assert (= 1 (length result)))
        (elt result 0))
      result))


(defun dom-select-tag (node tag-name
                       &key
                         direct
                         singleton)
  (let ((result (dom-select-if node
                               (lambda (node)
                                 (and (dom:element-p node)
                                      (string= tag-name (dom:tag-name node))))
                               :direct direct)))
    (dom-singleton result singleton)))

(defun dom-select-path (node path
                        &key
                          singleton
                          (undefined-error t)
                          default)
  (labels ((rec (nodes path)
             (cond
               ((null path) nodes)
               ((consp (car path))
                (rec (loop
                        with attr = (caar path)
                        with value = (cdar path)
                        for n in nodes
                        when (progn (and (dom:has-attribute n attr)
                                         (string= value (dom:get-attribute n attr))))
                        collect n)
                     (cdr path)))
               ((eq #\@ (aref (car path) 0))
                (assert (null (cdr path)))
                (loop
                   with attribute = (subseq (car path) 1)
                   for n in nodes
                   collect (dom:get-attribute n attribute)))
               (t
                (rec (loop for n in nodes
                        nconc (dom-select-if n (lambda (node)
                                                 (and (dom:element-p node)
                                                      (string= (car path) (dom:tag-name node))))
                                             :direct t))
                     (cdr path))))))
    (let* ((result (rec (list node) path))
           (length (length result)))
      ;(print result)
      (cond
        ((zerop length)
         (when undefined-error
           (error "Could not find ~A in document" path))
         default)
        (singleton
         (assert (= 1 length))
         (elt result 0))
        (t result)))))

;; (defun split-spaces (string function)
;;   (declare ;(optimize (speed 3) (safety 0))
;;            (type simple-string string)
;;            (type function function))
;;   (let ((length (length string)))
;;     (labels ((find-start (start)
;;                (loop for i from start
;;                   until (or (>= i length)
;;                             (not (amino::char-isblank (aref string i))))
;;                   finally (return i)))
;;              (find-end (start)
;;                (loop for i from start
;;                   while (and (< i length)
;;                                (not (amino::char-isblank (aref string i))))
;;                   finally (return i))))
;;       (loop
;;          for start = (find-start 0) then (find-start end)
;;          for end = (find-end start)
;;          while (< start length)
;;          collect (let ((string (subseq string start end)))
;;                    (funcall function string))))))


(defun parse-integer-list (text)
  (declare (type simple-string text))
  (loop
     with length = (length text)
     with end = 0
     for start = 0 then end
     for number = (when (< start length)
                    (multiple-value-bind (i new-end)
                        (parse-integer text :start start :junk-allowed t)
                      (assert (> new-end start))
                      (setq end new-end)
                      i))
     while number
     collect number))

(defun parse-float-sequence (text &optional (start 0))
  (map 'list #'parse-float (ppcre:split "\\s+" text :start start)))

(defun list-double-vector (list)
  (make-array (length list)
              :element-type 'double-float
              :initial-contents list))

(defun list-fixnum-vector (list)
  (make-array (length list)
              :element-type 'fixnum
              :initial-contents list))

(declaim (inline array-cat))

(defun array-cat (element-type arguments)
  (let* ((n
          (etypecase arguments
            (list (loop for x in arguments summing (length x)))
            (array (loop for x across arguments summing (length x)))))
         (y (make-array n :element-type element-type))
         (start 0))
    (etypecase arguments
      (list (dolist (x arguments)
              (replace y x :start1 start)
              (incf start (length x))))
      (array (dotimes (i (length arguments))
               (let ((x (aref arguments i)))
                 (replace y x :start1 start)
                 (incf start (length x))))))
    y))

(defun invert-scale (x)
    (- 1d0 (clamp x 0d0 1d0)))

(defun find-script (name)
  (let ((pathname
         (clean-pathname (concatenate 'string
                                      (namestring (asdf:system-source-directory :robray))
                                      "/share/exec/"
                                      name))))
    (assert (probe-file pathname) ()
            "Script '~A' not found" name)
    pathname))

(defun load-trajectory (pathname)
  (let ((data (with-open-file (in pathname :direction :input)
                (loop for line = (read-line in nil nil)
                   while line
                   collect (parse-float-sequence line)))))
    data))
