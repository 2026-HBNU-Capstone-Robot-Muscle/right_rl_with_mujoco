# One YOLO oriented bounding-box detection for a cube face.
# Pixel coordinates use the source image's top-left as origin.
# z is always 0.0. corners_px retain detector order and form one polygon cycle.
string class_name
float32 confidence
geometry_msgs/Point32[4] corners_px
geometry_msgs/Point32 center_px
