;; Auto-generated. Do not edit!


(when (boundp 'amrl_msgs::Localization2DMsg)
  (if (not (find-package "AMRL_MSGS"))
    (make-package "AMRL_MSGS"))
  (shadow 'Localization2DMsg (find-package "AMRL_MSGS")))
(unless (find-package "AMRL_MSGS::LOCALIZATION2DMSG")
  (make-package "AMRL_MSGS::LOCALIZATION2DMSG"))

(in-package "ROS")
;;//! \htmlinclude Localization2DMsg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass amrl_msgs::Localization2DMsg
  :super ros::object
  :slots (_header _pose _map ))

(defmethod amrl_msgs::Localization2DMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pose __pose) (instance amrl_msgs::Pose2Df :init))
    ((:map __map) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _pose __pose)
   (setq _map (string __map))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:map
   (&optional __map)
   (if __map (setq _map __map)) _map)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; amrl_msgs/Pose2Df _pose
    (send _pose :serialization-length)
    ;; string _map
    4 (length _map)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; amrl_msgs/Pose2Df _pose
       (send _pose :serialize s)
     ;; string _map
       (write-long (length _map) s) (princ _map s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; amrl_msgs/Pose2Df _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; string _map
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _map (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get amrl_msgs::Localization2DMsg :md5sum-) "4ce450daa8564e2fb59b919aebbbe26e")
(setf (get amrl_msgs::Localization2DMsg :datatype-) "amrl_msgs/Localization2DMsg")
(setf (get amrl_msgs::Localization2DMsg :definition-)
      "std_msgs/Header header
Pose2Df pose
string map

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: amrl_msgs/Pose2Df
float32 x
float32 y
float32 theta

")



(provide :amrl_msgs/Localization2DMsg "4ce450daa8564e2fb59b919aebbbe26e")


