
;(load "package://tableclothdemo/euslisp/demo.l")
(load "demo.l")
(ros::roseus "real-grab-side")
(ros::advertise "tablecloth_bb" jsk_recognition_msgs::BoundingBoxArray 5)
(ros::subscribe "checkerboard_pose" geometry_msgs::PoseStamped #'sub-checkerboard)

