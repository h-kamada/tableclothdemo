
;(load "package://tableclothdemo/euslisp/demo.l")
(load "grab-side.l")
(ros::subscribe "tablecloth_height" std_msgs::Float32 #'sub-height)
(ros::advertise "table_static" geometry_msgs::PoseArray 5)
(pub)