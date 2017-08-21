#include "stereo_publisher.h"


/*
// Calibration
// rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0025 right:=/camera_left/image_raw left:=/camera_right/image_raw right_camera:=/stereo/right left_camera:=/stereo/left --no-service-check
*/

// constructor just subscribes to image topics
stereoPublisher::stereoPublisher(): _it(_nh), cloud(new PCloud)
{
	// subscribers
	this->_left_image_sub = _it.subscribe("/camera_left/image_raw", 1,
											&stereoPublisher::imageLeftCb, this);
	this->_right_image_sub = _it.subscribe("/camera_right/image_raw", 1,
											&stereoPublisher::imageRightCb, this);
	this->_left_info_sub = _nh.subscribe("/camera_left/camera_info", 1,
											&stereoPublisher::cameraInfoLeftCb, this);
	this->_right_info_sub = _nh.subscribe("/camera_right/camera_info", 1,
											&stereoPublisher::cameraInfoRightCb, this);
	this->_extrinsics_sub = _nh.subscribe("/stereo/stereo_extrinsics_info", 1,
											&stereoPublisher::stereoExtrinsicsCb, this);
	// publishers
	this->_disparity_pub = _it.advertise("/stereo/disparity", 1);
	this->_point_cloud_pub = _nh.advertise<PCloud> ("/stereo/points", 1);



	 // sgbm parameters
	//this->mode = StereoSGBM::MODE_SGBM;
	this->mode = StereoSGBM::MODE_SGBM_3WAY;
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

	// indicates if camera_info topic is published for the cameras
	this->isCalibratedLeft = false;
	this->isCalibratedRight = false;
	this->haveExtrinsics = false;
	

	this->left_sgbm = StereoSGBM::create( this->minDisparities, this->maxDisparities, 
									 this->blockSize, this->P1, this->P2, 
									 this->preFilterCap, this->uniquenessRatio, 
									 this->speckleRange, this->mode );

	this->lambda = 8000;
	this->sigma = 1.2;
	this->wls_filter = createDisparityWLSFilter(this->left_sgbm);
	this->right_sgbm = createRightMatcher(this->left_sgbm);

}

stereoPublisher::~stereoPublisher()
{	
}

void stereoPublisher::imageLeftCb(const sensor_msgs::ImageConstPtr& msg)
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

void stereoPublisher::imageRightCb(const sensor_msgs::ImageConstPtr& msg)
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
			
			if(config.mode != this->mode)
			{
				if(config.mode == 0)
					this->mode = StereoSGBM::MODE_SGBM;
				else if(config.mode == 1)
					this->mode = StereoSGBM::MODE_HH;
				else if(config.mode == 2)
					this->mode = StereoSGBM::MODE_SGBM_3WAY;

				this->updateSGBM = true;
			}

			if(config.minDisparities != this->minDisparities)
			{
				this->minDisparities = config.minDisparities;
				this->updateSGBM = true;
			}

			// make the disparities factors of 16
			if(config.maxDisparities != this->maxDisparities)
			{
				if((config.maxDisparities % 16) != 0)
				{
					int val = config.maxDisparities % 16;
					this->maxDisparities = config.maxDisparities - val;
				}
				else if (config.maxDisparities <= 0)
					this->maxDisparities = 16;
				else
					this->maxDisparities = config.maxDisparities;

				this->updateSGBM = true;
			}

			if(config.blockSize != this->blockSize)
			{
				// make the blocksize odd
				if(config.blockSize % 2 == 0)
					this->blockSize = config.blockSize + 1;
				else
					this->blockSize = config.blockSize;

				this->updateSGBM = true;
			}
			
			if(config.P1 != this->P1)
			{
				this->updateSGBM = true;
				this->P1 = config.P1;
			}
			if(config.P2 != this->P2)
			{
				this->updateSGBM = true;
				this->P2 = config.P2;
			}
			
			if(config.disp12MaxDiff != this->disp12MaxDiff)
			{
				this->updateSGBM = true;
				this->disp12MaxDiff = config.disp12MaxDiff;
			}

			if(config.preFilterCap != this->preFilterCap)
			{
				this->updateSGBM = true;
				this->preFilterCap = config.preFilterCap;
			}

			if(config.uniquenessRatio != this->uniquenessRatio)
			{
				this->updateSGBM = true;
				this->uniquenessRatio = config.uniquenessRatio;
			}

			if(config.speckleWindowSize != this->speckleWindowSize)
			{
				this->updateSGBM = true;
				this->speckleWindowSize = config.speckleWindowSize;
			}

			if(config.speckleRange != this->speckleRange)
			{
				this->updateSGBM = true;
				this->speckleRange = config.speckleRange;
			}
			
}

// camera_info callback for left stereo camera
void stereoPublisher::cameraInfoLeftCb(const sensor_msgs::CameraInfo& msg)
{
// store the calibration parameters for the right camera
	this->calib_left.D = Mat::zeros(1,5, CV_64F);
	this->calib_left.K = Mat::zeros(3,3, CV_64F);
	this->calib_left.R = Mat::zeros(3,3, CV_64F);
	this->calib_left.P = Mat::zeros(3,4, CV_64F);

	for(int i=0; i<5; i++)
		this->calib_left.D.at<double>(i) = msg.D[i];
	for(int i=0; i<9; i++)
		this->calib_left.K.at<double>(i/3,i%3) = msg.K[i];
	for(int i=0; i<9; i++)
		this->calib_left.R.at<double>(i/3,i%3) = msg.R[i];
	for(int i=0; i<12; i++)
		this->calib_left.P.at<double>(i/4,i%4) = msg.P[i];

	// make sure there is valid data on the calibration topic
	if ( true
	   )
	{
		//cout << "Calibrated Left camera" << endl;
		this->isCalibratedLeft = true;
	}
}		

// camera_info callback for right stereo camera
void stereoPublisher::cameraInfoRightCb(const sensor_msgs::CameraInfo& msg)
{
	// store the calibration parameters for the right camera
	this->calib_right.D = Mat::zeros(1,5, CV_64F);
	this->calib_right.K = Mat::zeros(3,3, CV_64F);
	this->calib_right.R = Mat::zeros(3,3, CV_64F);
	this->calib_right.P = Mat::zeros(3,4, CV_64F);

	for(int i=0; i<5; i++)
		this->calib_right.D.at<double>(i) = msg.D[i];
	for(int i=0; i<9; i++)
		this->calib_right.K.at<double>(i/3,i%3) = msg.K[i];
	for(int i=0; i<9; i++)
		this->calib_right.R.at<double>(i/3,i%3) = msg.R[i];
	for(int i=0; i<12; i++)
		this->calib_right.P.at<double>(i/4,i%4) = msg.P[i];

	// make sure there is valid data on the calibration topic
	if ( true
	   )
	{
		//cout << "Calibrated Right camera" << endl;
		this->isCalibratedRight = true;
	}
}	

// subscribe to stereo extrinisc parameters (custom msg)
void stereoPublisher::stereoExtrinsicsCb(const camera_publisher::stereoExtrinsics& msg)
{
	// store the calibration parameters for the right camera
	this->stereo_extrinsics.R = Mat::zeros(3,3, CV_64F);
	this->stereo_extrinsics.T = Mat::zeros(1,3, CV_64F);
	this->stereo_extrinsics.Q = Mat::zeros(4,4, CV_64F);

	for(int i=0; i<9; i++)
		this->stereo_extrinsics.R.at<double>(i/3,i%3) = msg.R[i];
	for(int i=0; i<3; i++)
		this->stereo_extrinsics.T.at<double>(0, i) = msg.T[i];
	for(int i=0; i<16; i++)
		this->stereo_extrinsics.Q.at<double>(i/4,i%4) = msg.Q[i];

	if(true)
	{
		this->haveExtrinsics = true;
		//cout << "Confirmed Stereo Extrinsics" << endl;
	}
	// make sure there is valid data on the calibration topic

}	

// given a left and right images from the camera, computes and returns the disparity map
Mat stereoPublisher::computeDisparity(Mat l_img, Mat r_img, bool visualize)
{
	//-- And create the image in which we will save our disparities
	Mat img_disparity16S_L;
	Mat img_disparity16S_R;
	this->img_disparity8U_L = Mat( l_img.rows, l_img.cols, CV_8UC1 );
	this->img_disparity8U_R = Mat( l_img.rows, l_img.cols, CV_8UC1 );

	//-- Call the constructor for SGBM
	if(this->updateSGBM)
	{
		cout << "Updating the stereo matching algorithm" << endl;
		this->left_sgbm = StereoSGBM::create( this->minDisparities, this->maxDisparities, 
										 this->blockSize, this->P1, this->P2, this->disp12MaxDiff, 
										 this->preFilterCap, this->uniquenessRatio, 
										 this->speckleRange, this->mode );
		this->right_sgbm = createRightMatcher(this->left_sgbm);
		this->updateSGBM = false;
	}

	// check to make sure there are valid images for L and R camera
	if( l_img.empty() || r_img.empty() )
	{ 
		std::cout<< " --(!) Empty images in computeDisparity " << std::endl;
	 	return this->img_disparity8U_L; 
	}
	else
	{
		// make a local copy of the images for filtering
		Mat l_copy = l_img.clone();
		Mat r_copy = r_img.clone();

		//-- Calculate the disparity image
		this->left_sgbm->compute( l_copy, r_copy, img_disparity16S_L );
		this->right_sgbm->compute( r_copy, l_copy, img_disparity16S_R );

		/*
		//-- Check its extreme values
		double minVal; double maxVal;
		minMaxLoc( img_disparity16S, &minVal, &maxVal );
		minMaxLoc( img_disparity16S, &minVal, &maxVal );
		//printf("Min disp: %f Max value: %f \n", minVal, maxVal);

		//-- Display it as a CV_8UC1 image (If you want to normalize it)
		img_disparity16S.convertTo( img_disparity8U, CV_8UC1, 255/(maxVal - minVal));
		*/

		// do filtering
		Mat filtered_disp, conf_map;
		Rect conf_roi;
		this->wls_filter->setLambda(this->lambda);
		this->wls_filter->setSigmaColor(this->sigma);
		this->wls_filter->filter(img_disparity16S_L, l_img,
								filtered_disp,img_disparity16S_R);
		this->img_disparity_filtered = filtered_disp;
		getDisparityVis(this->img_disparity_filtered, this->img_filtered_disp_vis);

		conf_map = wls_filter->getConfidenceMap();

		// set the disparity images
		img_disparity16S_L.convertTo( img_disparity8U_L, CV_8UC1);
		img_disparity16S_R.convertTo( img_disparity8U_R, CV_8UC1);
		this->img_disparity8U_L = img_disparity8U_L;
		this->img_disparity8U_R = img_disparity8U_L;


		// Get the ROI that was used in the last filter call:
        conf_roi = wls_filter->getROI();

		// upscale raw disparity and ROI back for a proper comparison:
		resize(this->img_disparity8U_L,this->img_disparity8U_L,Size(),2.0,2.0);
		this->img_disparity8U_L = this->img_disparity8U_L*2.0;
		resize(this->img_disparity8U_R,this->img_disparity8U_R,Size(),2.0,2.0);
		this->img_disparity8U_R = this->img_disparity8U_R*2.0;
		conf_roi = Rect(conf_roi.x*2,conf_roi.y*2,conf_roi.width*2,conf_roi.height*2);
		
		if(visualize)
		{
			namedWindow( "disparity_left", WINDOW_NORMAL );
			namedWindow( "disparity_right", WINDOW_NORMAL );
			namedWindow( "disparity_filtered", WINDOW_NORMAL );
			imshow( "disparity_left", this->img_disparity8U_L );
			imshow( "disparity_right", this->img_disparity8U_R );
			imshow( "disparity_filtered", this->img_filtered_disp_vis);
		}

		return filtered_disp;
	}	
}



void stereoPublisher::printCalibration()
{
	cout <<  "--------------------------------------------" << endl;
	cout << "LEFT" << endl;
	cout <<  "--------------------------------------------" << endl;

	for(int i=0; i<calib_left.D.size().width; i++)
		for(int j=0; j<calib_left.D.size().height; j++)
			cout << calib_left.D.at<double>(i,j) << " ";
		cout << endl;

	for(int i=0; i<calib_left.K.size().width; i++)
		for(int j=0; j<calib_left.K.size().height; j++)
			cout << calib_left.K.at<double>(i,j) << " ";
		cout << endl;

	for(int i=0; i<calib_left.R.size().width; i++)
		for(int j=0; j<calib_left.R.size().height; j++)
			cout << calib_left.R.at<double>(i,j) << " ";
		cout << endl;

	for(int i=0; i<calib_left.P.size().width; i++)
		for(int j=0; j<calib_left.P.size().height; j++)
			cout << calib_left.P.at<double>(i,j) << " ";
		cout << endl;

	cout <<  "--------------------------------------------" << endl;
	cout << "RIGHT" << endl;
	cout <<  "--------------------------------------------" << endl;

	for(int i=0; i<calib_right.D.size().width; i++)
		for(int j=0; j<calib_right.D.size().height; j++)
			cout << calib_right.D.at<double>(i,j) << " ";
		cout << endl;

	for(int i=0; i<calib_right.K.size().width; i++)
		for(int j=0; j<calib_right.K.size().height; j++)
			cout << calib_right.K.at<double>(i,j) << " ";
		cout << endl;

	for(int i=0; i<calib_right.R.size().width; i++)
		for(int j=0; j<calib_right.R.size().height; j++)
			cout << calib_right.R.at<double>(i,j) << " ";
		cout << endl;

	for(int i=0; i<calib_right.P.size().width; i++)
		for(int j=0; j<calib_right.P.size().height; j++)
			cout << calib_right.P.at<double>(i,j) << " ";
		cout << endl;

	cout <<  "--------------------------------------------" << endl;
	cout << "EXTRINSICS" << endl;
	cout <<  "--------------------------------------------" << endl;

	for(int i=0; i<stereo_extrinsics.R.size().width; i++)
		for(int j=0; j<stereo_extrinsics.R.size().height; j++)
			cout << stereo_extrinsics.R.at<double>(i,j) << " ";
		cout << endl;

	for(int i=0; i<stereo_extrinsics.T.size().width; i++)
		for(int j=0; j<stereo_extrinsics.T.size().height; j++)
			cout << stereo_extrinsics.T.at<double>(i,j) << " ";
		cout << endl;

	for(int i=0; i<stereo_extrinsics.Q.size().width; i++)
		for(int j=0; j<stereo_extrinsics.Q.size().height; j++)
			cout << stereo_extrinsics.Q.at<double>(i,j) << " ";
		cout << endl;

		
}


// http://www.pcl-users.org/Conversion-from-cv-Mat-to-pcl-PointCloud-td4023936.html
void stereoPublisher::createPointcloudFromRegisteredDepthImage(cv::Mat& depthImage, cv::Mat& rgbImage, Mat& intrinsics)
{
    float rgbFocalInvertedX = 1/intrinsics.at<double>(0,0);	// 1/fx
	float rgbFocalInvertedY = 1/intrinsics.at<double>(1,1);	// 1/fy

	PCloud::Ptr outputPointcloud(new PCloud);
	pcl::PointXYZRGB newPoint;
	for (int i=0;i<depthImage.rows;i++)
	{
		for (int j=0;j<depthImage.cols;j++)
		{
			float depthValue = depthImage.at<Vec3f>(i,j)[2]/10.;
			float xValue = depthImage.at<Vec3f>(i,j)[0]/10.;
			float yValue = depthImage.at<Vec3f>(i,j)[1]/10.;

			if (depthValue == depthValue)                // if depthValue is not NaN
			{
				// Find 3D position respect to rgb frame:
				newPoint.z = depthValue;
				newPoint.x = xValue;
				newPoint.y = yValue;
				//newPoint.x = ((j - intrinsics.at<double>(0,2)) * newPoint.z * rgbFocalInvertedX);
				//newPoint.y = ((i - intrinsics.at<double>(1,2)) * newPoint.z * rgbFocalInvertedY);
				newPoint.r = rgbImage.at<cv::Vec3b>(i,j)[2];
				newPoint.g = rgbImage.at<cv::Vec3b>(i,j)[1];
				newPoint.b = rgbImage.at<cv::Vec3b>(i,j)[0];
				outputPointcloud->push_back(newPoint);
			}
			else
			{
				newPoint.z = std::numeric_limits<float>::quiet_NaN();
				newPoint.x = std::numeric_limits<float>::quiet_NaN();
				newPoint.y = std::numeric_limits<float>::quiet_NaN();
				newPoint.r = std::numeric_limits<unsigned char>::quiet_NaN();
				newPoint.g = std::numeric_limits<unsigned char>::quiet_NaN();
				newPoint.b = std::numeric_limits<unsigned char>::quiet_NaN();
				outputPointcloud->push_back(newPoint);
			}
		}
	}
	
	outputPointcloud->header.frame_id = "/base";
	outputPointcloud->height = 480;
	outputPointcloud->width = 640;
	pcl_conversions::toPCL(ros::Time::now(), outputPointcloud->header.stamp);
	this->_point_cloud_pub.publish (outputPointcloud);

}


Mat stereoPublisher::computeDepth(bool visualize)
{	
	// reprojectImageTo3D

	this->img_depth = Mat::zeros(this->img_disparity_filtered.size(), CV_32FC3);

	cout << "Calculating depth" << endl;
	reprojectImageTo3D(this->img_disparity_filtered, this->img_depth, this->stereo_extrinsics.Q, true);
	cout << "Done" << endl;
	
	this->cloud->clear();
	createPointcloudFromRegisteredDepthImage(this->img_depth, this->img_left, this->calib_left.K);

	return img_depth;
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
	
	std::cout << "--(!) Stereo Publisher: waiting for images to come up..." << std::endl;
	while (sp.img_left_gray.empty() || sp.img_right_gray.empty() )
	{ 
		waitKey(1000);
		ros::spinOnce();
	}

	sp.printCalibration();

	std::cout << "Starting depth calculation!" << std::endl;
	while (waitKey(10) != 27)
	{

		sp.img_disparity_filtered = sp.computeDisparity(sp.img_left_gray, sp.img_right_gray, true);
		sp.publishDisparity(sp.img_disparity_filtered);

		sp.img_depth = sp.computeDepth(true);
		

		ros::spinOnce();
	}

	return 0;
}