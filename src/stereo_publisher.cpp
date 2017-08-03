#include "stereo_publisher.h"

using namespace cv;
using namespace std;


// constructor just subscribes to image topics
stereoPublisher::stereoPublisher(): _it(_nh)
{
	 this->_left_image_sub = _it.subscribe("/camera_0/image_rgb", 1,
      										&stereoPublisher::leftImageCb, this);
	 this->_right_image_sub = _it.subscribe("/camera_1/image_rgb", 1,
      										&stereoPublisher::rightImageCb, this);
	 this->_disparity_pub = _it.advertise("/stereo_publisher/disparity", 1);

	 // sgbm parameters
	this->mode = StereoSGBM::MODE_SGBM;
	//this->mode = StereoSGBM::MODE_HH;

	this->minDisparities = 0;
	this->maxDisparities = 128;
	this->blockSize = 7;
	this->P1 = 400;
	this->P2 = 2000;
	this->preFilterCap = 5;
	this->uniquenessRatio = 5;
	this->speckleWindowSize = 100;
	this->speckleRange = 1;
	this->updateSGBM = false;
	
	

	this->sgbm = StereoSGBM::create( this->minDisparities, this->maxDisparities, 
									 this->blockSize, this->P1, this->P2, 
									 this->preFilterCap, this->uniquenessRatio, 
									 this->speckleRange, this->mode );
}

stereoPublisher::~stereoPublisher()
{	
}

void stereoPublisher::leftImageCb(const sensor_msgs::ImageConstPtr& msg)
{
	//cout << "Left CB" << endl;
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
	//cout << "Left CB" << endl;
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

// broadcast the new disparity image on the ROS network
void stereoPublisher::publishDisparity(Mat disp)
{
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", disp).toImageMsg();
	this->_disparity_pub.publish(msg);
}

// set SGBM properties when the GUI changes
void stereoPublisher::reconfigure_callback(camera_publisher::stereo_paramsConfig &config, uint32_t level) {
			
			if(config.mode == 0)
				this->mode = StereoSGBM::MODE_SGBM;
			else
				this->mode = StereoSGBM::MODE_HH;

			this->minDisparities = config.minDisparities;

			// make the disparities factors of 16
			if((config.maxDisparities % 16) != 0)
			{
				int val = config.maxDisparities % 16;
				this->maxDisparities = config.maxDisparities - val;
			}
			else if (config.maxDisparities <= 0)
				this->maxDisparities = 16;
			else
				this->maxDisparities = config.maxDisparities;

			// make the blocksize odd
			if(config.blockSize % 2 == 0)
				this->blockSize = config.blockSize + 1;
			else
				this->blockSize = config.blockSize;
			
			this->P1 = config.P1;
			this->P2 = config.P2;
			this->disp12MaxDiff = config.disp12MaxDiff;
			this->preFilterCap = config.preFilterCap;
			this->uniquenessRatio = config.uniquenessRatio;
			this->speckleWindowSize = config.speckleWindowSize;
			this->speckleRange = config.speckleRange;
			this->updateSGBM = true;

			cout << this->maxDisparities << endl;
}

// given a left and right images from the camera, computes and returns the disparity map
Mat stereoPublisher::computeDisparity(Mat l_img, Mat r_img, bool visualize)
{
	//-- And create the image in which we will save our disparities
	Mat img_disparity16S = Mat( l_img.rows, l_img.cols, CV_16S );
	Mat img_disparity8U = Mat( l_img.rows, l_img.cols, CV_8UC1 );

	//-- Call the constructor for SGBM
	if(this->updateSGBM)
	{
		this->sgbm = StereoSGBM::create( this->minDisparities, this->maxDisparities, 
										 this->blockSize, this->P1, this->P2, this->disp12MaxDiff, 
										 this->preFilterCap, this->uniquenessRatio, 
										 this->speckleRange, this->mode );
	}

	// check to make sure there are valid images for L and R camera
	if( l_img.empty() || r_img.empty() )
	{ 
		std::cout<< " --(!) Empty images in computeDisparity " << std::endl;
	 	return img_disparity8U; 
	}
	else
	{
		//-- Calculate the disparity image
		this->sgbm->compute( l_img, r_img, img_disparity16S );

		//-- Check its extreme values
		double minVal; double maxVal;
		minMaxLoc( img_disparity16S, &minVal, &maxVal );
		//printf("Min disp: %f Max value: %f \n", minVal, maxVal);

		//-- Display it as a CV_8UC1 image
		img_disparity16S.convertTo( img_disparity8U, CV_8UC1, 255/(maxVal - minVal));

		if(visualize)
		{
			namedWindow( "disparity", WINDOW_NORMAL );
			imshow( "disparity", img_disparity8U );
		}
		return img_disparity8U;
	}	
}




int main(int argc, char** argv)
{	
	ros::init(argc, argv, "stereo_publisher");
	stereoPublisher sp = stereoPublisher();
	
	// set up dynamic reconfigure
	dynamic_reconfigure::Server<camera_publisher::stereo_paramsConfig> server;
	dynamic_reconfigure::Server<camera_publisher::stereo_paramsConfig>::CallbackType f;

	f = boost::bind(&stereoPublisher::reconfigure_callback, &sp, _1, _2);
	server.setCallback(f);

	while (sp.img_left_gray.empty() || sp.img_right_gray.empty() )
	{ 
		std::cout << "--(!) Stereo Publisher: waiting for images to come up..." << std::endl;
		waitKey(1000);
		ros::spinOnce();
	}

	std::cout << "Starting depth calculation!" << std::endl;
	while (waitKey(10) != 27)
	{
		// calculate the disparity map
		sp.img_disparity8U = sp.computeDisparity(sp.img_left_gray, sp.img_right_gray, true);
		sp.publishDisparity(sp.img_disparity8U);

		ros::spinOnce();
	}

	return 0;
}