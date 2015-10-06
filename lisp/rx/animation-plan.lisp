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


;; TODO: complete the following structure definitions

(defstruct animation-plan
  "Stores an sequence of actions for an animation")

(defstruct action-trajectory
  "Stores an trajectory for an animation.")

(defstruct action-insert
  "An action to insert objects into the scene graph."

(defstruct action-remove
  "An action to remove objects from the scene graph."


;;; TODO: complete the following function definitions
;;;   - All operations should be non-desctructive, i.e., the arguments
;;;     to the functions should not be modified.

(defun animation-plan ()
  "Create an empty animation plan")

(defun animation-append-trajectory (plan source)
  "Append a trajectory action from source.
PLAN: an animation-plan
SOURCE: the trajectory file, sequence of waypoints")

(defun animation-append-reparent (plan frame-name new-parent)
  "Append operations to reparent an object.
PLAN: an animation-plan
FRAME-NAME: the frame to reparent
NEW-PARENT: the name of the new parent of frame FRAME-NAME")


(defun animation-create (plan &key
                                (render-frames t)
                                (encode-video t)
                                (output-directory *robray-tmp-directory*)
                                (options *render-options*)
                                include)
  "Create an animation from PLAN.
RENDER-FRAMES: call the raytracer to render each frame
ENCODE-VIDEO: encode the frames into a video
OUTPUT-DIRECTORY: working directory for rendering and encoding
OPTIONS: options for the rendering
INCLUDE: include files for the raytracer")
