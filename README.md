"# ws24-lidar-camera-fusion" 

First Step is to convert rgb8 image to bgr8 by simultaneously running the bagfile and imageconversion.py file 
then run this command to save the images rosrun image_view extract_images image:=/dji_osdk_ros/main_camera_images_bgr8 _filename_format:=frame_bgr_%04d.jpg
