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
