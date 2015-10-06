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
