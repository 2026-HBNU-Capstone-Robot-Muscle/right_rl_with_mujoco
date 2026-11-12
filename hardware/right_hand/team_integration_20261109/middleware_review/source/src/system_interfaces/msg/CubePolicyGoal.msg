# Calibrated goal orientation for one active logical cube move.
std_msgs/Header header
string scan_id
string execution_id
string target_face
# Quaternion in ROS order x,y,z,w, resolved by the calibrated face/frame mapper.
geometry_msgs/Quaternion goal_orientation
