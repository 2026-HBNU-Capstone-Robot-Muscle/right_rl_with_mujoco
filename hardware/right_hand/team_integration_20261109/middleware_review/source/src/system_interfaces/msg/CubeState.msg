# Cube facelets use Kociemba's URFDLB face order and 54-character encoding.
# facelets is empty unless valid is true and physical-state validation passed.
# observed_faces contains unique values from U, R, F, D, L, B.
std_msgs/Header header
string scan_id
string facelets
bool valid
string[] observed_faces
string[] validation_errors
