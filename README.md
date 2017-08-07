Stereo Camera ROS Node

Calibration:
1) Run the ROS camera_calibration node and copy the folder it generates into
   the /doc folder. 
   	$ rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0025 right:=/camera_0/image_raw left:=/camera_1/image_raw right_camera:=/stereo/right left_camera:=/stereo/left --no-service-check

2) Run the stereo_calib executable on these images
	$ rosrun camera_publisher stereo_calib -w=6 -h=8 /home/kyle/Dropbox/catkin_ws/src/camera_publisher/doc/calibrationdata/stereo_calib.xml

Running the Stereo Publisher
1) Run the camera publisher nodes for left/right cameras
	$ roslaunch camera_publisher stereo_camera.launch

	- this launch script publishes raw image data and calibration results on:
		/camera_0/image_raw
		/camera_0/camera_info
		etc.