
(cl:in-package :asdf)

(defsystem "pr_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "ResumeTrajectory" :depends-on ("_package_ResumeTrajectory"))
    (:file "_package_ResumeTrajectory" :depends-on ("_package"))
    (:file "Idle" :depends-on ("_package_Idle"))
    (:file "_package_Idle" :depends-on ("_package"))
    (:file "Enable" :depends-on ("_package_Enable"))
    (:file "_package_Enable" :depends-on ("_package"))
    (:file "AskUser" :depends-on ("_package_AskUser"))
    (:file "_package_AskUser" :depends-on ("_package"))
    (:file "ArmConfigCheck" :depends-on ("_package_ArmConfigCheck"))
    (:file "_package_ArmConfigCheck" :depends-on ("_package"))
    (:file "AppletCommand" :depends-on ("_package_AppletCommand"))
    (:file "_package_AppletCommand" :depends-on ("_package"))
  ))