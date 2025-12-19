; Auto-generated. Do not edit!


(cl:in-package autodock_core-msg)


;//! \htmlinclude AutoDockingFeedback.msg.html

(cl:defclass <AutoDockingFeedback> (roslisp-msg-protocol:ros-message)
  ((progress
    :reader progress
    :initarg :progress
    :type cl:float
    :initform 0.0)
   (status
    :reader status
    :initarg :status
    :type cl:string
    :initform "")
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass AutoDockingFeedback (<AutoDockingFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AutoDockingFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AutoDockingFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autodock_core-msg:<AutoDockingFeedback> is deprecated: use autodock_core-msg:AutoDockingFeedback instead.")))

(cl:ensure-generic-function 'progress-val :lambda-list '(m))
(cl:defmethod progress-val ((m <AutoDockingFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autodock_core-msg:progress-val is deprecated.  Use autodock_core-msg:progress instead.")
  (progress m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <AutoDockingFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autodock_core-msg:status-val is deprecated.  Use autodock_core-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <AutoDockingFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autodock_core-msg:state-val is deprecated.  Use autodock_core-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<AutoDockingFeedback>)))
    "Constants for message type '<AutoDockingFeedback>"
  '((:STATE_INVALID . 0)
    (:STATE_IDLE . 1)
    (:STATE_PREDOCK . 2)
    (:STATE_PARALLEL_CORRECTION . 3)
    (:STATE_STEER_DOCK . 4)
    (:STATE_LAST_MILE . 5)
    (:STATE_ACTIVATE_CHARGER . 6)
    (:STATE_RETRY . 7)
    (:STATE_PAUSE . 8))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'AutoDockingFeedback)))
    "Constants for message type 'AutoDockingFeedback"
  '((:STATE_INVALID . 0)
    (:STATE_IDLE . 1)
    (:STATE_PREDOCK . 2)
    (:STATE_PARALLEL_CORRECTION . 3)
    (:STATE_STEER_DOCK . 4)
    (:STATE_LAST_MILE . 5)
    (:STATE_ACTIVATE_CHARGER . 6)
    (:STATE_RETRY . 7)
    (:STATE_PAUSE . 8))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AutoDockingFeedback>) ostream)
  "Serializes a message object of type '<AutoDockingFeedback>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'progress))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AutoDockingFeedback>) istream)
  "Deserializes a message object of type '<AutoDockingFeedback>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'progress) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AutoDockingFeedback>)))
  "Returns string type for a message object of type '<AutoDockingFeedback>"
  "autodock_core/AutoDockingFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AutoDockingFeedback)))
  "Returns string type for a message object of type 'AutoDockingFeedback"
  "autodock_core/AutoDockingFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AutoDockingFeedback>)))
  "Returns md5sum for a message object of type '<AutoDockingFeedback>"
  "a81bac62ffc117ad82928737b405b684")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AutoDockingFeedback)))
  "Returns md5sum for a message object of type 'AutoDockingFeedback"
  "a81bac62ffc117ad82928737b405b684")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AutoDockingFeedback>)))
  "Returns full string definition for message of type '<AutoDockingFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%######################################################################~%# FEEDBACK~%~%## progress approx, 0 - 1.0 ~%float32 progress~%~%## verbose description of the progress~%string status~%~%## state~%uint8 STATE_INVALID = 0~%uint8 STATE_IDLE = 1~%uint8 STATE_PREDOCK = 2~%uint8 STATE_PARALLEL_CORRECTION = 3~%uint8 STATE_STEER_DOCK = 4~%uint8 STATE_LAST_MILE = 5~%uint8 STATE_ACTIVATE_CHARGER = 6~%uint8 STATE_RETRY = 7~%uint8 STATE_PAUSE = 8~%~%uint8 state~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AutoDockingFeedback)))
  "Returns full string definition for message of type 'AutoDockingFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%######################################################################~%# FEEDBACK~%~%## progress approx, 0 - 1.0 ~%float32 progress~%~%## verbose description of the progress~%string status~%~%## state~%uint8 STATE_INVALID = 0~%uint8 STATE_IDLE = 1~%uint8 STATE_PREDOCK = 2~%uint8 STATE_PARALLEL_CORRECTION = 3~%uint8 STATE_STEER_DOCK = 4~%uint8 STATE_LAST_MILE = 5~%uint8 STATE_ACTIVATE_CHARGER = 6~%uint8 STATE_RETRY = 7~%uint8 STATE_PAUSE = 8~%~%uint8 state~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AutoDockingFeedback>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'status))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AutoDockingFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'AutoDockingFeedback
    (cl:cons ':progress (progress msg))
    (cl:cons ':status (status msg))
    (cl:cons ':state (state msg))
))
