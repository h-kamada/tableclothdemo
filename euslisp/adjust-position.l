
(load "demo.l")
(ros::roseus "adjust-position")
(ros::subscribe "tablecloth_bb" jsk_recognition_msgs::BoundingBoxArray #'sub-tablecloth-bb)
(ros::subscribe "tablecloth_bb_fixed" jsk_recognition_msgs::BoundingBoxArray #'sub-tablecloth-bb-fixed)
(ros::advertise "hoge" geometry_msgs::PoseArray 5)
(ros::advertise "fuga" geometry_msgs::PoseArray 5)

