#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from camera_publisher.cfg import camera_paramsConfig
from std_msgs.msg import Int32MultiArray, Int32, Bool, String

class dashboard():

	def __init__(self):

		# nested dictionary of camera parameters adjustable through dynreconf gui
		self.cam_params = ['enable', 'refresh_rate', 'device_id', 'img_res']
		params0 = {'enable':False, 'refresh_rate':30, 'device_id':0, 'img_res':"1920x1080"}
		params1 = {'enable':False, 'refresh_rate':30, 'device_id':1, 'img_res':"1920x1080"}
		self.cameras = {'camera_0':params0, 'camera_1':params1}


		# ROS messaging
		self.camera_0_enable_pub = rospy.Publisher('camera_0/enable', Bool, queue_size=1)
		self.camera_1_enable_pub = rospy.Publisher('camera_1/enable', Bool, queue_size=1)

		self.camera_0_refresh_rate_pub = rospy.Publisher('camera_0/refresh_rate', Int32, queue_size=1)
		self.camera_1_refresh_rate_pub = rospy.Publisher('camera_1/refresh_rate', Int32, queue_size=1)

		self.camera_0_device_id_pub = rospy.Publisher('camera_0/device_id', Int32, queue_size=1)
		self.camera_1_device_id_pub = rospy.Publisher('camera_1/device_id', Int32, queue_size=1)

		self.camera_0_img_res_pub = rospy.Publisher('camera_0/img_res', String, queue_size=1)
		self.camera_1_img_res_pub = rospy.Publisher('camera_1/img_res', String, queue_size=1)

		publishers0 = {'enable':self.camera_0_enable_pub, \
					   'refresh_rate':self.camera_0_refresh_rate_pub, \
					   'device_id':self.camera_0_device_id_pub, \
					   'img_res':self.camera_0_img_res_pub, \
					   }
		publishers1 = {'enable':self.camera_1_enable_pub, \
					   'refresh_rate':self.camera_1_refresh_rate_pub, \
					   'device_id':self.camera_1_device_id_pub, \
					   'img_res':self.camera_1_img_res_pub, \
					   }
		self.camera_publishers = {'camera_0':publishers0, 'camera_1':publishers1}

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


