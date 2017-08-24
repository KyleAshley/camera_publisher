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
#include <climits>
#include <opencv2/opencv.hpp>

#include "feature_detector.h"

using namespace cv;
using namespace std;

// custom implementation of SIFT feature detection
// - uses opencv gaussian blur function 
class SiftDetector: IFeatureDetector
{


	public:
		// ctor / dtor
		SiftDetector(int num_octaves, int num_levels);
		~SiftDetector();


		void detect(cv::Mat img);

	private:

		int num_octaves;		// number of resized levels in the scale space (recommeded = 4)
		int num_levels;			// number of DoG blur levels in each octave (recommeded = 5)

		// 2D vector of gaussian blurred images
		// - cv::Mat img = gauss_octaves[octave][scale];
		std::vector<std::vector<cv::Mat> >  octaves;

		// keypoints from successive image frames
		std::vector<KeyPoint> keypoints_1;

		cv::Mat filter2D(cv::Mat img, cv::Mat filter);


};

