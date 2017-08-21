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

#include <stdio.h>
#include <math.h>
#include <vector>
#include "opencv2/xfeatures2d.hpp"

using namespace cv;
using namespace std;


class featureDetector
{


	public:
		// ctor / dtor
		featureDetector();
		~featureDetector();



	private:
		// pointer to an opencv feature detector
		cv::Ptr<Feature2D> detector;	

		// keypoints from successive image frames
		std::vector<KeyPoint> keypoints_1, keypoints_2;	


};


int main()
{

	return 0;
}
