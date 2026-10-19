# First version supports only complete trajectory replacement.
uint8 REPLACE=0

std_msgs/Header header
uint8 update_mode
string[] joint_names
trajectory_msgs/JointTrajectoryPoint[] points
