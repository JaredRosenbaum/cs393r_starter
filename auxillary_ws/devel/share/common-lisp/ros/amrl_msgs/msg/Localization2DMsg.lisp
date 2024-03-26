; Auto-generated. Do not edit!


(cl:in-package amrl_msgs-msg)


;//! \htmlinclude Localization2DMsg.msg.html

(cl:defclass <Localization2DMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pose
    :reader pose
    :initarg :pose
    :type amrl_msgs-msg:Pose2Df
    :initform (cl:make-instance 'amrl_msgs-msg:Pose2Df))
   (map
    :reader map
    :initarg :map
    :type cl:string
    :initform ""))
)

(cl:defclass Localization2DMsg (<Localization2DMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Localization2DMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Localization2DMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amrl_msgs-msg:<Localization2DMsg> is deprecated: use amrl_msgs-msg:Localization2DMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Localization2DMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amrl_msgs-msg:header-val is deprecated.  Use amrl_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <Localization2DMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amrl_msgs-msg:pose-val is deprecated.  Use amrl_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'map-val :lambda-list '(m))
(cl:defmethod map-val ((m <Localization2DMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amrl_msgs-msg:map-val is deprecated.  Use amrl_msgs-msg:map instead.")
  (map m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Localization2DMsg>) ostream)
  "Serializes a message object of type '<Localization2DMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'map))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'map))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Localization2DMsg>) istream)
  "Deserializes a message object of type '<Localization2DMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'map) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'map) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Localization2DMsg>)))
  "Returns string type for a message object of type '<Localization2DMsg>"
  "amrl_msgs/Localization2DMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Localization2DMsg)))
  "Returns string type for a message object of type 'Localization2DMsg"
  "amrl_msgs/Localization2DMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Localization2DMsg>)))
  "Returns md5sum for a message object of type '<Localization2DMsg>"
  "4ce450daa8564e2fb59b919aebbbe26e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Localization2DMsg)))
  "Returns md5sum for a message object of type 'Localization2DMsg"
  "4ce450daa8564e2fb59b919aebbbe26e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Localization2DMsg>)))
  "Returns full string definition for message of type '<Localization2DMsg>"
  (cl:format cl:nil "std_msgs/Header header~%Pose2Df pose~%string map~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: amrl_msgs/Pose2Df~%float32 x~%float32 y~%float32 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Localization2DMsg)))
  "Returns full string definition for message of type 'Localization2DMsg"
  (cl:format cl:nil "std_msgs/Header header~%Pose2Df pose~%string map~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: amrl_msgs/Pose2Df~%float32 x~%float32 y~%float32 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Localization2DMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     4 (cl:length (cl:slot-value msg 'map))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Localization2DMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'Localization2DMsg
    (cl:cons ':header (header msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':map (map msg))
))
