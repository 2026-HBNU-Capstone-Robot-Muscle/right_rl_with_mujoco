# Calibrated policy coordinates; publisher converts motor feedback using verified mapping.
std_msgs/Header header
string scan_id
string execution_id
# right, front, left, back order; same coordinate units as sim2real/core.py.
float32[4] policy_positions
# right, front, left, back order; coordinate units per second.
float32[4] policy_velocities
