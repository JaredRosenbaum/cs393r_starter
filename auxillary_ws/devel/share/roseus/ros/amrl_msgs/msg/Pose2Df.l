;; Auto-generated. Do not edit!


(when (boundp 'amrl_msgs::Pose2Df)
  (if (not (find-package "AMRL_MSGS"))
    (make-package "AMRL_MSGS"))
  (shadow 'Pose2Df (find-package "AMRL_MSGS")))
(unless (find-package "AMRL_MSGS::POSE2DF")
  (make-package "AMRL_MSGS::POSE2DF"))

(in-package "ROS")
;;//! \htmlinclude Pose2Df.msg.html


(defclass amrl_msgs::Pose2Df
  :super ros::object
  :slots (_x _y _theta ))

(defmethod amrl_msgs::Pose2Df
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:theta __theta) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _theta (float __theta))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _theta
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta
     (setq _theta (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get amrl_msgs::Pose2Df :md5sum-) "a130bc60ee6513855dc62ea83fcc5b20")
(setf (get amrl_msgs::Pose2Df :datatype-) "amrl_msgs/Pose2Df")
(setf (get amrl_msgs::Pose2Df :definition-)
      "float32 x
float32 y
float32 theta

")



(provide :amrl_msgs/Pose2Df "a130bc60ee6513855dc62ea83fcc5b20")


