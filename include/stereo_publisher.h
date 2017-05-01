#include <ros/ros.h>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <stdio.h>
#include <math.h>



class stereoPublisher
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber left_image_sub_, right_image_sub_;

	public:
		stereoPublisher();
		~stereoPublisher();

		cv::Mat img_left, img_right, img_left_gray, img_right_gray;
		// ROS callbacks for left and right image topics
		void leftImageCb(const sensor_msgs::ImageConstPtr& msg);
		void rightImageCb(const sensor_msgs::ImageConstPtr& msg);

	private:

};
