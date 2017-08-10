#include <ros/ros.h>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/ximgproc/disparity_filter.hpp"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <dynamic_reconfigure/server.h>
#include <camera_publisher/stereo_paramsConfig.h>
#include <camera_publisher/stereoExtrinsics.h>

#include <stdio.h>
#include <math.h>
#include <vector>

using namespace cv;
using namespace std;
using namespace cv::ximgproc;


/*
Stereo Publisher Class:
	- subscribes to left and right RGB image topics
	- computes disparity map and broadcasts results
*/

struct calibrationData {
     cv::Mat D;  	// 1x5 distortion coeffs
     cv::Mat K; 	// 3x3 camera matrix
     cv::Mat R; 	// 3x3 rectificatoin matrix
     cv::Mat P; 	// 3x4 projection matrix
};

struct stereoExtrinsics {
     cv::Mat R; 	// 3x3 rotation matrix
     cv::Mat T; 	// 1x3 translation matrix
     cv::Mat Q;   	// 4x4 disparity to depth
};

typedef pcl::PointCloud<pcl::PointXYZRGB> PCloud;

class stereoPublisher
{

		// subs
		ros::NodeHandle _nh;
		image_transport::ImageTransport _it;
		image_transport::Subscriber _left_image_sub, _right_image_sub;

		ros::Subscriber _left_info_sub, _right_info_sub;
		ros::Subscriber _extrinsics_sub;

		// pubs
		image_transport::Publisher _disparity_pub;
		ros::Publisher _point_cloud_pub;

	public:
		stereoPublisher();
		~stereoPublisher();

		cv::Mat img_left, img_right, img_left_gray, img_right_gray;
		cv::Mat img_disparity8U_L, img_disparity8U_R, img_disparity_filtered, img_filtered_disp_vis;
		cv::Ptr<DisparityWLSFilter> wls_filter; 	// wls filter for post processing disparity
		float lambda, sigma;


		// two separate stereo matchers for the left and right images (to perform filtering)
		cv::Ptr<StereoMatcher> right_sgbm, left_sgbm;
		

		cv::Mat img_depth;
		PCloud::Ptr cloud;

		calibrationData calib_left, calib_right;
		stereoExtrinsics stereo_extrinsics;
		bool isCalibratedLeft, isCalibratedRight, haveExtrinsics;

		// visulization data
		cv::Mat lr_viz;

		// sgbm parameters
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
		void imageLeftCb(const sensor_msgs::ImageConstPtr& msg); 		// grab image data for left camera
		void imageRightCb(const sensor_msgs::ImageConstPtr& msg);		// grad image data for right camera
		void cameraInfoLeftCb(const sensor_msgs::CameraInfo& msg);		// grab image calibration data for left camera
		void cameraInfoRightCb(const sensor_msgs::CameraInfo& msg);		// grab image calibration data for right camera
		void stereoExtrinsicsCb(const camera_publisher::stereoExtrinsics& msg);		// grab stereo calibration data for right camera

		// ROS publishers for the results
		void publishDisparity(cv::Mat disp);

		void createPointcloudFromRegisteredDepthImage(cv::Mat& depthImage, cv::Mat& rgbImage, cv::Mat& intrinsics);
		cv::Mat computeDisparity(cv::Mat l_img, cv::Mat r_img, bool visualize);

		// calibration
		cv::Mat computeDepth(bool visualize);
		
		void printCalibration();

};
