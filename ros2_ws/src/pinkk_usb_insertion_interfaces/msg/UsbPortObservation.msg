# 검출 keypoint와 solvePnP 결과를 원자적으로 전달한다.
std_msgs/Header header
string detection_id
geometry_msgs/Pose pose
pinkk_usb_insertion_interfaces/Keypoint2D[4] keypoints
float32 object_confidence
float32 reprojection_error_px
float32 depth_m
bool valid
string rejection_reason
