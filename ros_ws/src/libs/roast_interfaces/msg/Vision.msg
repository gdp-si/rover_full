# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id

# To add images from two cameras (processed)
sensor_msgs/Image[2] images
sensor_msgs/CameraInfo[2] camera_info
sensor_msgs/RegionOfInterest[2] region_of_interest
sensor_msgs/Image stitched_image

# To use if any disparity is available
stereo_msgs/DisparityImage disparity_image
geometry_msgs/Point32[] point_cloud
