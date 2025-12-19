
(cl:in-package :asdf)

(defsystem "autodock_core-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AutoDockingAction" :depends-on ("_package_AutoDockingAction"))
    (:file "_package_AutoDockingAction" :depends-on ("_package"))
    (:file "AutoDockingActionFeedback" :depends-on ("_package_AutoDockingActionFeedback"))
    (:file "_package_AutoDockingActionFeedback" :depends-on ("_package"))
    (:file "AutoDockingActionGoal" :depends-on ("_package_AutoDockingActionGoal"))
    (:file "_package_AutoDockingActionGoal" :depends-on ("_package"))
    (:file "AutoDockingActionResult" :depends-on ("_package_AutoDockingActionResult"))
    (:file "_package_AutoDockingActionResult" :depends-on ("_package"))
    (:file "AutoDockingFeedback" :depends-on ("_package_AutoDockingFeedback"))
    (:file "_package_AutoDockingFeedback" :depends-on ("_package"))
    (:file "AutoDockingGoal" :depends-on ("_package_AutoDockingGoal"))
    (:file "_package_AutoDockingGoal" :depends-on ("_package"))
    (:file "AutoDockingResult" :depends-on ("_package_AutoDockingResult"))
    (:file "_package_AutoDockingResult" :depends-on ("_package"))
  ))