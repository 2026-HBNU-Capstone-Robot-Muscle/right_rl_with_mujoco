# Abstract planning state for dry-run policy calls; this is not sensor feedback.
std_msgs/Header header
# Initial planned cube orientation in ROS order x,y,z,w.
geometry_msgs/Quaternion cube_orientation
# Unit face normals in the cube frame, concatenated in URFDLB order.
float64[18] face_normals_cube_frame_urfdlb
# Palm position minus cube position in meters, in the policy reference frame.
float64[3] palm_minus_cube_position_m
# LEAP commanded joint positions in the policy's 16-joint order, radians.
float64[16] leap_command_positions_rad
# Custom policy coordinates, ordered right, front, left, back.
float64[4] custom_policy_positions
float64[4] custom_policy_velocities
