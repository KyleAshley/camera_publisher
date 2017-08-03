#include <ros/ros.h>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include <camera_publisher/stereo_paramsConfig.h>


#include <stdio.h>
#include <math.h>
/*
Stereo Publisher Class:
	- subscribes to left and right RGB image topics
	- computes disparity map and broadcasts results
*/


class stereoPublisher
{

		ros::NodeHandle _nh;
		image_transport::ImageTransport _it;
		image_transport::Subscriber _left_image_sub, _right_image_sub;
		image_transport::Publisher _disparity_pub;

	public:
		stereoPublisher();
		~stereoPublisher();

		cv::Mat img_left, img_right, img_left_gray, img_right_gray;
		cv::Mat img_disparity8U;

		// visulization data
		cv::Mat lr_viz;

		// sgbm parameters
		cv::Ptr<cv::StereoSGBM> sgbm;
		int minDisparities;
		int maxDisparities;
		int blockSize;
		int P1;
		int P2; 
		int disp12MaxDiff;
		int preFilterCap;
		int uniquenessRatio;
		int speckleWindowSize;
		int speckleRange;
		int mode;
		bool updateSGBM;			// flags when changes are made to SGMB params 

		// ROS callbacks for left and right image topics etc.
		void reconfigure_callback(camera_publisher::stereo_paramsConfig &config, uint32_t level);
		void leftImageCb(const sensor_msgs::ImageConstPtr& msg);
		void rightImageCb(const sensor_msgs::ImageConstPtr& msg);

		// ROS publishers for the results
		void publishDisparity(cv::Mat disp);
		cv::Mat computeDisparity(cv::Mat l_img, cv::Mat r_img, bool visualize);

		

};
