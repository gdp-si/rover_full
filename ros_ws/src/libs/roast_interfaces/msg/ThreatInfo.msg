builtin_interfaces/Time stamp
string frame_id

geometry_msgs/PoseStamped threat_pose # threat pose in the global frame
float32 threat_yaw # threat yaw in the global frame
sensor_msgs/Image threat_image # threat images in the robot frame
bool threat_detected # true if threat is detected
bool near_safety_perimeter  # true if robot is near safe zone
uint8 threat_type # "person", "external threats like fire, smoke, etc", "vehicle", "animal", "other"
string threat_description # description of the threat


# Enumerate Threat Types
uint8 OTHER = 0
uint8 FIRE = 1
uint8 SMOKE = 2
uint8 VEHICLE = 3
uint8 ANIMAL = 4
uint8 PERSON = 5
