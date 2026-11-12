# One synchronized software-policy sample. Pose frame IDs must match the configured world_frame.
std_msgs/Header header
string scan_id
string execution_id
string target_face
geometry_msgs/PoseStamped cube_pose
geometry_msgs/PoseStamped palm_pose
# Desired cube orientation, ROS quaternion order x,y,z,w.
geometry_msgs/Quaternion goal_orientation
# LEAP joints named and measured in the reference policy's simulation order, radians.
sensor_msgs/JointState leap_joint_state
# Last commanded positions in the same 16-joint simulation order, radians.
float64[16] commanded_joint_positions
