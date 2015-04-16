(in-package :robray)

(defparameter *robray-cache-directory*
  (concatenate 'string
               "/tmp/" (sb-posix:getenv "USER") "-cache/robray"))


(defmacro string-case (keyform &body cases)
  (let ((value (gensym "value")))
    `(let ((,value ,keyform))
       (cond
         ,@(loop for case in cases
              collect (destructuring-bind (key &rest body) case
                        (cond
                          ((eq key 'otherwise)
                           `(t ,@body))
                          (t `((string= ,value ,key) ,@body)))))))))

(defun parse-float-sequence (text)
  (map 'list #'parse-float (ppcre:split " " text)))

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
      (cond
        ((zerop length)
         (when undefined-error
           (error "Could not find ~A in document" path))
         default)
        (singleton
         (assert (= 1 length))
         (elt result 0))
        (t result)))))
