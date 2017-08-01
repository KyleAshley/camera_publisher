#include "stereo_publisher.h"

using namespace cv;

stereoPublisher::stereoPublisher(): it_(nh_)
{
	 this->left_image_sub_ = it_.subscribe("/camera_0/image_rgb", 1,
      										&stereoPublisher::leftImageCb, this);
	 this->right_image_sub_ = it_.subscribe("/camera_1/image_rgb", 1,
      										&stereoPublisher::rightImageCb, this);
}

stereoPublisher::~stereoPublisher()
{	
}

void stereoPublisher::leftImageCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Update GUI Window
	this->img_left = cv_ptr->image;
	cvtColor(this->img_left, this->img_left_gray, CV_BGR2GRAY);
	cv::imshow("LEFT", cv_ptr->image);
	cv::waitKey(3);
}

void stereoPublisher::rightImageCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Update GUI Window
	this->img_right = cv_ptr->image;
	cvtColor(this->img_right, this->img_right_gray, CV_BGR2GRAY);
	cv::imshow("RIGHT", cv_ptr->image);
	cv::waitKey(3);
}


int main(int argc, char** argv)
{	
	ros::init(argc, argv, "stereo_publisher");
	stereoPublisher sp = stereoPublisher();
	
	std::cout << "waiting for images to come up..." << std::endl;
	std::cout << sp.img_left_gray.empty() << " " << sp.img_right_gray.empty() << std::endl;

	while (sp.img_left_gray.empty() || sp.img_right_gray.empty() )
	{ 
		waitKey(1000);
		ros::spinOnce();
	}
	std::cout << "Starting depth calculation!" << std::endl;
		
	//-- And create the image in which we will save our disparities
	Mat imgDisparity16S = Mat( sp.img_left_gray.rows, sp.img_left_gray.cols, CV_16S );
	Mat imgDisparity8U = Mat( sp.img_left_gray.rows, sp.img_left_gray.cols, CV_8UC1 );

	//-- 2. Call the constructor for StereoBM
	int minDisparities = 32;
	int maxDisparities = 192;   /**< Range of disparity */
	int blockSize = 9; /**< Size of the block window. Must be odd */
	int P1 = 600;
	int P2 = 2400;
	int preFilterCap = 4;
	int uniquessRatio = 1;
	int speckleRange = 2;
	int mode = 0;

	Ptr<StereoSGBM> sbm = StereoSGBM::create( minDisparities, maxDisparities, blockSize,
											P1, P2, preFilterCap, uniquessRatio, 
											speckleRange, mode );

	while (waitKey(10) != 27)
	{
		if( sp.img_left_gray.empty() || sp.img_right_gray.empty() )
		{ std::cout<< " --(!) Error reading images " << std::endl; return -1; }

		//-- 3. Calculate the disparity image
		sbm->compute( sp.img_left_gray, sp.img_right_gray, imgDisparity16S );

		//-- Check its extreme values
		double minVal; double maxVal;

		minMaxLoc( imgDisparity16S, &minVal, &maxVal );

		printf("Min disp: %f Max value: %f \n", minVal, maxVal);

		//-- 4. Display it as a CV_8UC1 image
		imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));

		namedWindow( "disparity", WINDOW_NORMAL );
		imshow( "disparity", imgDisparity8U );
		
		ros::spinOnce();
	}

	return 0;
}