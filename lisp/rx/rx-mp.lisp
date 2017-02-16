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


;;; MP struct ;;;

(cffi:defcfun aa-rx-mp-create rx-mp-t
  (ssg rx-sg-sub-t))




(cffi:defcfun aa-rx-mp-set-start :void
  (mp rx-mp-t)
  (n-all size-t)
  (q-all :pointer))


(cffi:defcfun (mutable-scene-graph-config-count "aa_rx_sg_config_count") size-t
  (m-sg rx-sg-t))

(cffi:defcfun (mutable-scene-graph-frame-count "aa_rx_sg_frame_count") size-t
  (m-sg rx-sg-t))

(cffi:defcfun aa-rx-mp-set-goal :int
  (mp rx-mp-t)
  (n-subset size-t)
  (q-subset :pointer))

(cffi:defcfun aa-rx-mp-set-wsgoal :int
  (mp rx-mp-t)
  (n-e size-t)
  (frames :pointer)
  (E :pointer)
  (ld-e size-t))

(cffi:defcfun aa-rx-mp-plan :int
  (mp rx-mp-t)
  (timeout amino-ffi::coercible-double)
  (n-path :pointer)
  (p-path :pointer))

(cffi:defcfun aa-rx-sg-cl-init :void
  (sg rx-sg-t))

(cffi:defcfun aa-rx-mp-allow-collision :void
  (mp rx-mp-t)
  (id0 rx-frame-id)
  (id1 rx-frame-id)
  (allowed :boolean))

(cffi:defcfun aa-rx-mp-set-simplify :void
  (mp rx-mp-t)
  (simplify :boolean))

(cffi:defcfun aa-rx-mp-set-track-collisions :void
  (mp rx-mp-t)
  (track :boolean))

(cffi:defcfun aa-rx-mp-get-collisions :pointer
  (mp rx-mp-t))

(defun motion-planner (sub-scene-graph)
  (let ((mp (aa-rx-mp-create sub-scene-graph)))
    (setf (rx-mp-sub-scene-graph mp)
          sub-scene-graph)
    mp))

(defun motion-planner-sub-scene-graph (motion-planner)
  (rx-mp-sub-scene-graph motion-planner))

(defun motion-planner-mutable-scene-graph (motion-planner)
  (sub-scene-graph-mutable-scene-graph (motion-planner-sub-scene-graph motion-planner)))

(defun motion-planner-scene-graph (motion-planner)
  (sub-scene-graph-scene-graph (motion-planner-sub-scene-graph motion-planner)))

(defun motion-planner-set-start (motion-planner start)
  (let* ((ssg (rx-mp-sub-scene-graph motion-planner))
         (n-all (sub-scene-graph-all-config-count ssg))
         (q-start (make-vec n-all)))
      (sub-scene-graph-all-config-vector ssg start q-start)
      (with-foreign-simple-vector (pointer length) q-start :input
        (aa-rx-mp-set-start motion-planner length pointer))))

(defun motion-planner-set-joint-goal (motion-planner joint-goal)
  (let* ((ssg (rx-mp-sub-scene-graph motion-planner))
         (start-vec (sub-scene-graph-config-vector ssg joint-goal))
         (result))
    (with-foreign-simple-vector (pointer length) start-vec :input
      (assert (= length
                 (sub-scene-graph-config-count ssg)))
      (setq result
            (aa-rx-mp-set-goal motion-planner length pointer)))
    (zerop result)))

(defun motion-planner-set-work-goal (motion-planner work-goal)
  ;; TODO: multiple goals
  ;; TODO: frame argument
  (let* ((array (tf-array work-goal))
         (result))
    (with-foreign-simple-vector (pointer length) array :input
      (assert (= 7 length))
      (setq result
            (aa-rx-mp-set-wsgoal motion-planner
                                 1 (cffi:null-pointer) pointer 7)))
    (zerop result)))

(defun motion-planner-allow-collision (motion-planner frame-0 frame-1 &optional (allowed t))
  (let* ((ssg (rx-mp-sub-scene-graph motion-planner))
         (m-sg (sub-scene-graph-mutable-scene-graph ssg))
         (id-0 (mutable-scene-graph-frame-id m-sg frame-0))
         (id-1 (mutable-scene-graph-frame-id m-sg frame-1)))
    (aa-rx-mp-allow-collision motion-planner id-0 id-1 allowed)))

(defun motion-planner-allow-all (motion-planner allowed-set)
  (do-collision-set ((frame-0 frame-1) allowed-set)
    (motion-planner-allow-collision motion-planner frame-0 frame-1)))

(defstruct (motion-plan (:constructor %make-motion-plan))
  sub-scene-graph
  config-names
  mutable-scene-graph
  scene-graph
  path)

(defun make-motion-plan (&key
                           sub-scene-graph
                           mutable-scene-graph
                           scene-graph
                           config-names
                           path)
  (let* ((mutable-scene-graph (cond (mutable-scene-graph
                                     mutable-scene-graph)
                                    (sub-scene-graph
                                     (sub-scene-graph-mutable-scene-graph sub-scene-graph))
                                    (scene-graph
                                     (mutable-scene-graph scene-graph))))
         (config-names (cond (config-names config-names)
                             (sub-scene-graph
                              (sub-scene-graph-config-name-array sub-scene-graph))
                             (mutable-scene-graph
                              (mutable-scene-graph-config-name-array mutable-scene-graph)))))
    (%make-motion-plan :sub-scene-graph sub-scene-graph
                       :mutable-scene-graph mutable-scene-graph
                       :config-names config-names
                       :scene-graph (cond (scene-graph scene-graph)
                                          (mutable-scene-graph
                                           (mutable-scene-graph-scene-graph mutable-scene-graph))
                                          (sub-scene-graph
                                           (sub-scene-graph-scene-graph sub-scene-graph)))
                       :path path)))

(defun motion-plan-valid-p (motion-plan)
  (when (motion-plan-p motion-plan)
    (not (zerop (length (motion-plan-path motion-plan))))))

;; (defun motion-plan-mutable-scene-graph (motion-plan)
;;   (sub-scene-graph-mutable-scene-graph
;;    (motion-plan-sub-scene-graph motion-plan)))

;; (defun motion-plan-scene-graph (motion-plan)
;;   (mutable-scene-graph-scene-graph
;;    (motion-plan-mutable-scene-graph motion-plan)))

(defun motion-plan-config-names-list (motion-plan)
  (let ((cn (motion-plan-config-names motion-plan)))
    (etypecase cn
      (list cn)
      (vector
       (loop for name across cn
          collect name)))))

(defun motion-plan-config-count (motion-plan)
  (length (motion-plan-config-names motion-plan)))

(defun motion-plan-all-config-count (motion-plan)
  (mutable-scene-graph-config-count (motion-plan-mutable-scene-graph motion-plan)))

(defun motion-plan-point-count (motion-plan)
  (/ (length (motion-plan-path motion-plan))
     (motion-plan-all-config-count motion-plan)))

(defun motion-plan-path-list (motion-plan)
  (let* ((names (motion-plan-config-names motion-plan))
         (m-sg (motion-plan-mutable-scene-graph motion-plan))
         (m (motion-plan-all-config-count motion-plan))
         (path (motion-plan-path motion-plan))
         (indices (map 'list (lambda (name)
                               (mutable-scene-graph-config-index m-sg name))
                       names)))
    (loop for j below (motion-plan-point-count motion-plan)
       collect
         (loop for i in indices
            collect (aref path (+ i (* j m)))))))


(defun motion-plan-length (motion-plan)
  (let* ((path (motion-plan-path motion-plan))
         (m-sg (motion-plan-mutable-scene-graph motion-plan))
         (n-config (mutable-scene-graph-config-count m-sg))
         (n-path (/ (length path)
                    n-config)))
    (check-type n-path integer)
    n-path))

(defun sequence-motion-plan (sub-scene-graph points)
  (let* ((n-p  (length points))
         (n-q (sub-scene-graph-all-config-count sub-scene-graph))
         (path (make-matrix n-q n-p)))
    ;; fill in path
    (loop for j from 0
       for p in points
       do (loop for i below n-q
             for name = (sub-scene-graph-all-config-name sub-scene-graph i)
             for q = (robray::configuration-map-find p name)
             do
               (setf (matref path i j) q)))
    (make-motion-plan :sub-scene-graph sub-scene-graph
                      :path (amino::matrix-data path))))

(defun motion-planner-collisions (planner)
  "Return set of collisions encountered during planning"
  (let* ((ptr (aa-rx-mp-get-collisions planner))
         (m-sg (motion-planner-mutable-scene-graph planner))
         (n-frame (mutable-scene-graph-frame-count m-sg))
         (result nil))
    (when (cffi:null-pointer-p ptr)
      (error "No collision data for motion planner"))
    (dotimes (i n-frame)
      (dotimes (j i)
        (when (aa-rx-cl-set-get-ptr ptr i j)
          (push (cons (mutable-scene-graph-frame-name m-sg i)
                      (mutable-scene-graph-frame-name m-sg j))
                result))))
    result))

(defun motion-plan (sub-scene-graph start-map
                    &key
                      jointspace-goal
                      workspace-goal
                      (simplify t)
                      (track-collisions nil)
                      (timeout 1d0))
  ;;(print workspace-goal)
  ;;(declare (optimize (speed 0) (debug 3)))
  (let* ((ssg sub-scene-graph)
         (m-sg (let ((m-sg (sub-scene-graph-mutable-scene-graph ssg)))
                 (aa-rx-sg-cl-init m-sg)
                 m-sg))
         (sg (mutable-scene-graph-scene-graph m-sg))
         (planner (motion-planner sub-scene-graph))
         (n-all (sub-scene-graph-all-config-count ssg)))
    ;; check things
    (check-scene-graph sg)

    ;; Setup Planner
    (aa-rx-mp-set-simplify planner simplify)
    (aa-rx-mp-set-track-collisions planner track-collisions)
    (motion-planner-allow-all planner (scene-graph-allowed-collisions sg))

    ;; Set start state
    (motion-planner-set-start planner start-map)

    ;; Set goal state
    (unless (cond
              (jointspace-goal
               (motion-planner-set-joint-goal planner jointspace-goal))
              (workspace-goal
               (motion-planner-set-work-goal planner workspace-goal))
              (t (error "No goal given")))
      ;; bail out early if goal setting fails
      (print 'bailing-out)
      ;(print (motion-planner-collisions planner))
      (return-from motion-plan (values nil planner)))

    ;; Call Planner
    (multiple-value-bind (result n-path path-ptr)
        (cffi:with-foreign-object (plan-length 'amino-ffi:size-t)
          (cffi:with-foreign-object (plan-ptr :pointer)
            (let ((result (aa-rx-mp-plan planner timeout plan-length plan-ptr)))
              (values result
                      (cffi:mem-ref plan-length 'amino-ffi:size-t)
                      (cffi:mem-ref plan-ptr :pointer)))))
      ;; Collisions
      ;; (when track-collisions
      ;;   (format t "~&Displaying collisions: ~%")
      ;;   (let ((ptr (aa-rx-mp-get-collisions planner)))
      ;;     (dotimes (i n-all)
      ;;       (dotimes (j i)
      ;;         (if (aa-rx-cl-set-get-ptr ptr i j)
      ;;             (format t "~&collisoin: ~D -> ~D~%" i j)
      ;;             (format t "~&OK: ~D -> ~D~%" i j))))))

      ;; Result
      (values (when (zerop result)
                ;; got a plan
                (let* ((m (* n-path n-all))
                       (result (make-vec m)))
                  ;; copy plan to lisp-space
                  (with-foreign-simple-vector (pointer length) result :output
                    (amino-ffi:libc-memcpy pointer path-ptr
                                           (* length (cffi:foreign-type-size :double))))
                  ;; free c-space plan
                  (amino-ffi:libc-free path-ptr)
                  (make-motion-plan :path result
                                    :sub-scene-graph sub-scene-graph)))
              planner))))

;; (defun win-view-plan (motion-plan)
;;   (win-pause)
;;   (win-set-scene-graph (motion-plan-mutable-scene-graph motion-plan))
;;   (win-set-display-plan (motion-plan-path motion-plan))
;;   (win-unpause))

(defun motion-plan-refmap (motion-plan i)
  (let* ((m-sg (motion-plan-mutable-scene-graph motion-plan))
         (n-all (mutable-scene-graph-config-count m-sg))
         (path (motion-plan-path motion-plan))
         (map (make-configuration-map)))
    (loop for j below n-all
       for k from (* i n-all)
       for name = (mutable-scene-graph-config-name m-sg j)
       do (tree-map-insertf map name (aref path k)))
    map))

(defun motion-plan-endpoint-map (motion-plan)
  (motion-plan-refmap motion-plan
                      (1- (motion-plan-length motion-plan))))

(defun motion-plan-refarray (motion-plan i)
  (let* ((m-sg (motion-plan-mutable-scene-graph motion-plan))
         (n-all (mutable-scene-graph-config-count m-sg))
         (path (motion-plan-path motion-plan))
         (array (make-vec n-all)))
    (loop for j below n-all
       for k from (* i n-all)
       do (setf (aref array j)
                (aref path k)))
    array))

(defun motion-plan-endpoint-array (motion-plan)
  (let* ((ssg (motion-plan-sub-scene-graph motion-plan))
         (n-sub (sub-scene-graph-config-count ssg))
         (path (motion-plan-path motion-plan))
         (i-0 (- (length path) n-sub)))
    (subseq path i-0)))



(defun motion-plan-rope (mp)
  (let* ((ssg (motion-plan-sub-scene-graph mp))
         (path (motion-plan-path mp))
         (m (sub-scene-graph-all-config-count ssg))
         (n (/ (length path) m))
         (names (loop for i below m collect (sub-scene-graph-all-config-name ssg i))))
    (rope
     (format nil "~&# ~{~A, ~}" names)
     (format nil "~{~&~{~F ~}~}"
             (loop for j below n
                collect
                  (loop for i below m
                     collect (aref path (+ i (* j m)))))))))

(defun output-motion-plans (list directory)
  (loop for mp in list
     for rope = (motion-plan-rope mp)
     for i from 0
     do (output-rope rope
                     (rope directory "/" (format nil "mp-~D.dat" i))
                     :if-exists :supersede)))



;;;;;;;;;;;;;;;
;;; MP Seq. ;;;
;;;;;;;;;;;;;;;

(cffi:defcfun (make-mp-seq "aa_rx_mp_seq_create") rx-mp-seq-t)

(cffi:defcfun aa-rx-mp-seq-append-all :void
  (mp-seq rx-mp-seq-t)
  (m-sg rx-sg-t)
  (n-path size-t)
  (q-all-path :pointer))


;; TODO: retain reference to the mutable scene graph
(defun mp-seq-append-mp (mp-seq motion-plan)
  (let* ((m-sg (motion-plan-mutable-scene-graph motion-plan))
         (path (motion-plan-path motion-plan))
         (n-path (motion-plan-length motion-plan)))
    (with-foreign-simple-vector (path-ptr path-length) path :input
      (declare (ignore path-length))
      (aa-rx-mp-seq-append-all mp-seq m-sg n-path path-ptr)))
  mp-seq)
