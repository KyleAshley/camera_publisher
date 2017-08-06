#!/usr/bin/env python
import rospy
import yaml
import sys, os
from camera_publisher.msg import stereoExtrinsics

"""
ROS Node that takes a calibration yaml file and publishes the info on that camera's camera_info topic
Derived from: http://answers.ros.org/question/33929/camera-calibration-parser-in-python/
"""

def yaml_to_Extrinsics(yaml_fname):
    """
    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data
    Returns
    -------
    stereo_extrinsics_msg :
        A StereoExtrinsics message containing the camera calibration
        data
    """
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    stereo_extrinsics_msg = stereoExtrinsics()
    stereo_extrinsics_msg.R = calib_data["R"]["data"]
    stereo_extrinsics_msg.T = calib_data["T"]["data"]
    stereo_extrinsics_msg.Q = calib_data["Q"]["data"]
    return stereo_extrinsics_msg

if __name__ == "__main__":
    # Get fname from command line (cmd line input required)
   
    filename = str(sys.argv[2])
    camera_topic = str(sys.argv[1])

    # Parse yaml file
    stereo_extrinsics_msg = yaml_to_Extrinsics(filename)

    # Initialize publisher node
    rospy.init_node("stereo_extrinsics_publisher"+camera_topic.strip("/"), anonymous=True)
    publisher = rospy.Publisher(camera_topic+"/stereo_extrinsics_info", stereoExtrinsics, queue_size=10)
    rate = rospy.Rate(10)

    # Run publisher
    while not rospy.is_shutdown():
        publisher.publish(stereo_extrinsics_msg)
        rate.sleep()