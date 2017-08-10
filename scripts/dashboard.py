#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from camera_publisher.cfg import camera_paramsConfig
from std_msgs.msg import Int32MultiArray, Int32, Bool, String

class dashboard():

	def __init__(self):

		# nested dictionary of camera parameters adjustable through dynreconf gui
		self.cam_params = ['enable', 'refresh_rate', 'device_id', 'img_resolution']
		paramsLeft = {'enable':False, 'refresh_rate':30, 'device_id':0, 'img_resolution':"1920x1080"}
		paramsRight = {'enable':False, 'refresh_rate':30, 'device_id':1, 'img_resolution':"1920x1080"}
		self.cameras = {'camera_left':paramsLeft, 'camera_right':paramsRight}


		# ROS messaging
		self.camera_left_enable_pub = rospy.Publisher('camera_left/enable', Bool, queue_size=1)
		self.camera_right_enable_pub = rospy.Publisher('camera_right/enable', Bool, queue_size=1)

		self.camera_left_refresh_rate_pub = rospy.Publisher('camera_left/refresh_rate', Int32, queue_size=1)
		self.camera_right_refresh_rate_pub = rospy.Publisher('camera_right/refresh_rate', Int32, queue_size=1)

		self.camera_left_device_id_pub = rospy.Publisher('camera_left/device_id', Int32, queue_size=1)
		self.camera_right_device_id_pub = rospy.Publisher('camera_right/device_id', Int32, queue_size=1)

		self.camera_left_img_res_pub = rospy.Publisher('camera_left/image_resolution', String, queue_size=1)
		self.camera_right_img_res_pub = rospy.Publisher('camera_right/image_resolution', String, queue_size=1)

		publishers0 = {'enable':self.camera_left_enable_pub, \
					   'refresh_rate':self.camera_left_refresh_rate_pub, \
					   'device_id':self.camera_left_device_id_pub, \
					   'image_resolution':self.camera_left_img_res_pub, \
					   }
		publishers1 = {'enable':self.camera_right_enable_pub, \
					   'refresh_rate':self.camera_right_refresh_rate_pub, \
					   'device_id':self.camera_right_device_id_pub, \
					   'image_resolution':self.camera_right_img_res_pub, \
					   }
		self.camera_publishers = {'camera_left':publishers0, 'camera_right':publishers1}

		# setup the callback for the reconfigure server
		self.server = DynamicReconfigureServer(camera_paramsConfig, self.reconfigure)



	def reconfigure(self, config, level):
		# grab new values from GUI
		# check configurable params for each camera
		for p in self.cam_params:
			for c in self.cameras:
				if config[c+'_'+p] != self.cameras[c][p]:
					self.cameras[c][p] = config[c+'_'+p]		# set the new value
					self.camera_publishers[c][p].publish(config[c+'_'+p])

		return config



if __name__ == "__main__":
	rospy.init_node('camera_dashboard', anonymous=True)
	db = dashboard()
	rospy.spin()


