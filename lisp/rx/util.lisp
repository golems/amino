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

(defun alist-get (alist key &optional default)
  (if-let ((cons (assoc key alist)))
    (cdr cons)
    default))

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
    (ppcre:regex-replace-all regex
                             (etypecase path
                               (pathname (namestring path))
                               (rope (rope-string path) ))
                             separator)))

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

(defun create-parent-directories (pathname)
  (let ((parents (file-dirname pathname)))
    (when parents
      (uiop/run-program:run-program (list "mkdir" "-p" (file-dirname pathname))
                                    :output *standard-output*
                                    :error-output *error-output*))))

(defun file-rope (&rest elements)
  (rope-map #'identity elements :separator '/))

(defun output-file (file &optional directory)
  (etypecase directory
    (null file)
    (string (concatenate 'string (ensure-directory directory)
                         file))
    (rope (output-file file (rope-string directory)))
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
     (rope-write (rope object) :stream place :escape nil)
     (values))
    ((eq place t)
     (princ object *standard-output*)
     (values))
    ((null place)
     object)
    ((ropep place)
     (let ((place (output-file (rope-string place)
                              directory)))
       (ensure-directories-exist place)
       (with-open-file (s place :direction :output
                          :if-exists if-exists
                          :if-does-not-exist :create)
         (output object s)))
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
    (if (digit-char-p (aref sub 0))
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

(let ((scanner (ppcre:create-scanner "\\s+")))
  (defun split-spaces (text &optional (start 0))
    (ppcre:split scanner text :start start)))

(defun parse-float-sequence (text &optional (start 0))
  (map 'list #'parse-float (split-spaces text start)))

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

(defparameter *robray-root*
  (format-pathname "~A/../"
                   (namestring (asdf:system-source-directory :amino))))

(defun find-script (name)
  (let ((pathname (format-pathname "~A/share/exec/~A"
                                   *robray-root* name)))
    (assert (probe-file pathname) ()
            "Script '~A' not found" name)
    pathname))

;; (defun load-trajectory (pathname)
;;   (let ((data (with-open-file (in pathname :direction :input)
;;                 (loop for line = (read-line in nil nil)
;;                    while line
;;                    collect (parse-float-sequence line)))))
;;     data))

(defun strip-hash-comment (line)
  (declare (type simple-string line))
  (when line
    (let* ((i (position #\# line))
           (stripped (if i
                         (subseq line 0 i)
                         line)))
      (if (ppcre:scan "^\\s*$" stripped)
          nil
          stripped))))

(defparameter *config-load-time* (make-hash-table :test #'equal))

(defparameter *config-directories*
  `(,(format-pathname "~A/share/config" *robray-root*)
    "/etc/robray"
    "/usr/local/etc/robray"
    ,(format-pathname "~A/~A" (namestring (user-homedir-pathname)) ".robray")
    ,(format-pathname "~A/~A" (namestring (user-homedir-pathname)) ".config/robray")))

(defun load-config (&optional reload)
  (labels ((try-dir (dir)
             (map nil #'try-load (directory (format-pathname "~A/*.lisp" dir))))
           (try-load (file)
             (when (probe-file file)
               (let ((time (file-write-date file)))
                 (when (or reload
                           (null (nth-value 1 (gethash file *config-load-time*)))
                           (> time (gethash file *config-load-time* )))
                   (format t "~&; loading file '~A'" file)
                   (load file)
                   (setf (gethash file *config-load-time*)
                         time
                         reload t))))))
    (map nil #'try-dir *config-directories*)))



(defmacro catch-all (&body body)
  (with-gensyms (e)
    `(handler-case
         (progn ,@body)
       (condition (,e)
         (format *error-output* "~&ERROR: ~A~%" ,e)
         #+sbcl
         (sb-ext:exit :code -1)
         (break)))))

(defun capture-program-output (args &key
                                      directory)
  (let* ((status)
         (output (with-output-to-string (s)
                   (multiple-value-bind (output error-output %status)
                       (uiop:run-program args
                                         :directory directory
                                         :ignore-error-status t
                                         :output s
                                         :error-output s)
                     (declare (ignore output error-output))
                     (setq status %status)))))
    (values output status)))
