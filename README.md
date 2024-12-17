"# ws24-lidar-camera-fusion" 

First Step is to convert rgb8 image to bgr8 by simultaneously running the bagfile and imageconversion.py file 
then run this command to save the images rosrun image_view extract_images image:=/dji_osdk_ros/main_camera_images_bgr8 _filename_format:=frame_bgr_%04d.jpg

as the  map doesn’t exist, ensure the static transform command is running:

rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map velodyne

We need to check if the data points are synchronized as 

             /velodyne_points                                               372 msgs    : sensor_msgs/PointCloud2
             /dji_osdk_ros/main_camera_images                              1122 msgs    : sensor_msgs/Image

so we run this command

 rostopic hz /velodyne_points /dji_osdk_ros/main_camera_images

rosbag record -O snippet_with_bgr8.bag /velodyne_points /dji_osdk_ros/main_camera_images /dji_osdk_ros/main_camera_images_bgr8 /dji_osdk_ros/imu /tf

saving the converted images to a new bag files 

# Conversion of the bag file
To use R3LIVE with the input ROS bag file with our bag files, the following transformations are necessary:

Camera Images:

Input Topic: /dji_osdk_ros/main_camera_images__
Transformation: Convert images from RGB to BGR format.__
Output Topic: /camera/image_color__
Reason: R3LIVE requires images in the BGR format for texture and visual mapping.__

LiDAR Point Cloud:__

Input Topic: /velodyne_points__
Transformation:__
Strip unnecessary fields (ring and time) from the PointCloud2 messages.__
Retain only the x, y, z, and intensity fields.__
Adjust point_step and row_step to reflect the new point structure.__
Output Topic: /livox/lidar__
Reason: R3LIVE requires point cloud data with a simplified structure for LiDAR-based mapping.__
IMU Data:__

Input Topic: /dji_osdk_ros/imu__
Transformation:__
Retain only the header, angular_velocity, and linear_acceleration fields.__
Remove unused fields like orientation and covariance values.__
Output Topic: /livox/imu__
Reason: R3LIVE expects clean IMU data for accurate state estimation and fusion with visual and LiDAR inputs.__

Summary of Changes__
Data Type	Input Topic	Transformation	Output Topic__
Camera Images	/dji_osdk_ros/main_camera_images	Convert RGB → BGR	/camera/image_color__
LiDAR Point Cloud	/velodyne_points	Remove ring and time, keep x, y, z, intensity	/livox/lidar__
IMU Data	/dji_osdk_ros/imu	Retain header, angular_velocity, linear_acceleration	/livox/imu__
