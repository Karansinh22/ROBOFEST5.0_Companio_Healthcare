;; Auto-generated. Do not edit!


(when (boundp 'autodock_core::AutoDockingFeedback)
  (if (not (find-package "AUTODOCK_CORE"))
    (make-package "AUTODOCK_CORE"))
  (shadow 'AutoDockingFeedback (find-package "AUTODOCK_CORE")))
(unless (find-package "AUTODOCK_CORE::AUTODOCKINGFEEDBACK")
  (make-package "AUTODOCK_CORE::AUTODOCKINGFEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude AutoDockingFeedback.msg.html


(intern "*STATE_INVALID*" (find-package "AUTODOCK_CORE::AUTODOCKINGFEEDBACK"))
(shadow '*STATE_INVALID* (find-package "AUTODOCK_CORE::AUTODOCKINGFEEDBACK"))
(defconstant autodock_core::AutoDockingFeedback::*STATE_INVALID* 0)
(intern "*STATE_IDLE*" (find-package "AUTODOCK_CORE::AUTODOCKINGFEEDBACK"))
(shadow '*STATE_IDLE* (find-package "AUTODOCK_CORE::AUTODOCKINGFEEDBACK"))
(defconstant autodock_core::AutoDockingFeedback::*STATE_IDLE* 1)
(intern "*STATE_PREDOCK*" (find-package "AUTODOCK_CORE::AUTODOCKINGFEEDBACK"))
(shadow '*STATE_PREDOCK* (find-package "AUTODOCK_CORE::AUTODOCKINGFEEDBACK"))
(defconstant autodock_core::AutoDockingFeedback::*STATE_PREDOCK* 2)
(intern "*STATE_PARALLEL_CORRECTION*" (find-package "AUTODOCK_CORE::AUTODOCKINGFEEDBACK"))
(shadow '*STATE_PARALLEL_CORRECTION* (find-package "AUTODOCK_CORE::AUTODOCKINGFEEDBACK"))
(defconstant autodock_core::AutoDockingFeedback::*STATE_PARALLEL_CORRECTION* 3)
(intern "*STATE_STEER_DOCK*" (find-package "AUTODOCK_CORE::AUTODOCKINGFEEDBACK"))
(shadow '*STATE_STEER_DOCK* (find-package "AUTODOCK_CORE::AUTODOCKINGFEEDBACK"))
(defconstant autodock_core::AutoDockingFeedback::*STATE_STEER_DOCK* 4)
(intern "*STATE_LAST_MILE*" (find-package "AUTODOCK_CORE::AUTODOCKINGFEEDBACK"))
(shadow '*STATE_LAST_MILE* (find-package "AUTODOCK_CORE::AUTODOCKINGFEEDBACK"))
(defconstant autodock_core::AutoDockingFeedback::*STATE_LAST_MILE* 5)
(intern "*STATE_ACTIVATE_CHARGER*" (find-package "AUTODOCK_CORE::AUTODOCKINGFEEDBACK"))
(shadow '*STATE_ACTIVATE_CHARGER* (find-package "AUTODOCK_CORE::AUTODOCKINGFEEDBACK"))
(defconstant autodock_core::AutoDockingFeedback::*STATE_ACTIVATE_CHARGER* 6)
(intern "*STATE_RETRY*" (find-package "AUTODOCK_CORE::AUTODOCKINGFEEDBACK"))
(shadow '*STATE_RETRY* (find-package "AUTODOCK_CORE::AUTODOCKINGFEEDBACK"))
(defconstant autodock_core::AutoDockingFeedback::*STATE_RETRY* 7)
(intern "*STATE_PAUSE*" (find-package "AUTODOCK_CORE::AUTODOCKINGFEEDBACK"))
(shadow '*STATE_PAUSE* (find-package "AUTODOCK_CORE::AUTODOCKINGFEEDBACK"))
(defconstant autodock_core::AutoDockingFeedback::*STATE_PAUSE* 8)

(defun autodock_core::AutoDockingFeedback-to-symbol (const)
  (cond
        ((= const 0) 'autodock_core::AutoDockingFeedback::*STATE_INVALID*)
        ((= const 1) 'autodock_core::AutoDockingFeedback::*STATE_IDLE*)
        ((= const 2) 'autodock_core::AutoDockingFeedback::*STATE_PREDOCK*)
        ((= const 3) 'autodock_core::AutoDockingFeedback::*STATE_PARALLEL_CORRECTION*)
        ((= const 4) 'autodock_core::AutoDockingFeedback::*STATE_STEER_DOCK*)
        ((= const 5) 'autodock_core::AutoDockingFeedback::*STATE_LAST_MILE*)
        ((= const 6) 'autodock_core::AutoDockingFeedback::*STATE_ACTIVATE_CHARGER*)
        ((= const 7) 'autodock_core::AutoDockingFeedback::*STATE_RETRY*)
        ((= const 8) 'autodock_core::AutoDockingFeedback::*STATE_PAUSE*)
        (t nil)))

(defclass autodock_core::AutoDockingFeedback
  :super ros::object
  :slots (_progress _status _state ))

(defmethod autodock_core::AutoDockingFeedback
  (:init
   (&key
    ((:progress __progress) 0.0)
    ((:status __status) "")
    ((:state __state) 0)
    )
   (send-super :init)
   (setq _progress (float __progress))
   (setq _status (string __status))
   (setq _state (round __state))
   self)
  (:progress
   (&optional __progress)
   (if __progress (setq _progress __progress)) _progress)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; float32 _progress
    4
    ;; string _status
    4 (length _status)
    ;; uint8 _state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _progress
       (sys::poke _progress (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;; uint8 _state
       (write-byte _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _progress
     (setq _progress (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get autodock_core::AutoDockingFeedback :md5sum-) "a81bac62ffc117ad82928737b405b684")
(setf (get autodock_core::AutoDockingFeedback :datatype-) "autodock_core/AutoDockingFeedback")
(setf (get autodock_core::AutoDockingFeedback :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
######################################################################
# FEEDBACK

## progress approx, 0 - 1.0 
float32 progress

## verbose description of the progress
string status

## state
uint8 STATE_INVALID = 0
uint8 STATE_IDLE = 1
uint8 STATE_PREDOCK = 2
uint8 STATE_PARALLEL_CORRECTION = 3
uint8 STATE_STEER_DOCK = 4
uint8 STATE_LAST_MILE = 5
uint8 STATE_ACTIVATE_CHARGER = 6
uint8 STATE_RETRY = 7
uint8 STATE_PAUSE = 8

uint8 state


")



(provide :autodock_core/AutoDockingFeedback "a81bac62ffc117ad82928737b405b684")


