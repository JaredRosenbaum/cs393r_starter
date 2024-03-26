
(cl:in-package :asdf)

(defsystem "amrl_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Localization2DMsg" :depends-on ("_package_Localization2DMsg"))
    (:file "_package_Localization2DMsg" :depends-on ("_package"))
    (:file "Pose2Df" :depends-on ("_package_Pose2Df"))
    (:file "_package_Pose2Df" :depends-on ("_package"))
  ))