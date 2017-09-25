#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/ximgproc/disparity_filter.hpp"

#include <dynamic_reconfigure/server.h>

#include <stdio.h>
#include <math.h>
#include <vector>
#include <climits>

#include "sift_detector.h"


SiftDetector::SiftDetector(int num_octaves, int num_levels)
{
	this->num_octaves = num_octaves;
	this->num_levels = num_levels;
}


SiftDetector::~SiftDetector()
{
 	

}

cv::Mat filter2D(cv::Mat)
{

}


void SiftDetector::detect(cv::Mat img)
{
	Mat gray;
	cv::cvtColor(img, gray, CV_BGR2GRAY);
	Mat scaled = gray.clone();
	
	// create the scale space of the image
	// - generate octaves by gaussian blurring
	for(int i=0; i<this->num_octaves; i++)
	{	 
		// resize for the number of octaves
		cv::resize(gray, scaled, cv::Size(), 1/(pow(2.0,i)), 1/(pow(2.0,i)));
		vector<cv::Mat> scales;
		for(int j=0; j<this->num_levels; j++)
		{
			Mat blurred = scaled.clone();
			int k_size = (2*j)+1; 	// Kernel size (odd) 1, 3, 5, 7 etc.
			cv::GaussianBlur( scaled, blurred, Size( k_size, k_size ), 0, 0 );
			scales.push_back(blurred);
		}
		this->octaves.push_back(scales);
	}


	// approximate the LoG using DoG on scale space images
	vector<vector<cv::Mat> > dog; 					// stores DoG for each scale space image
	for(int i=0; i<this->octaves.size(); i++)
	{	 
		vector<cv::Mat> diffs;
		for(int j=0; j<this->octaves[0].size()-1; j++)
		{
			cv::Mat diff = Mat::zeros(this->octaves[i][j].size(), this->octaves[i][j].type());
			diff = this->octaves[i][j] - this->octaves[i][j+1];
			diffs.push_back(diff);
		}
		dog.push_back(diffs);
	}

	
	vector<vector<cv::Mat> > extrema; 			// stores extreme values of DoG
	for(int i=0; i<dog.size(); i++)
	{	 
		vector<cv::Mat> scale_extrema;
		for(int j=0; j<dog[0].size()-2; j++)
		{
			Mat d1, d2, d3;
			// store local min/max in an array, these are keypoint candidates
			cv::Mat res = Mat::zeros(dog[i][j].size(), dog[i][j].type());
			d1 = dog[i][j];
			d2 = dog[i][j+1];
			d3 = dog[i][j+2];

			int max=0;
			int min = INT_MAX;
			// iterate through the DoG image
			for(int ii=1; ii<d1.rows-1; ii++)
			{
				for(int jj=1; jj<d1.cols-1; jj++)
				{
					// compare relative to middle scale
					int curr = d2.at<uchar>(ii, jj);
					bool isMax = true;
					bool isMin = true;

					// check for first DoG image
					for(int ki=-1; ki<1; ki++)
					{
						for(int kj=-1; kj<1; kj++)
						{
							// check for local max
							if(d1.at<uchar>(ii+ki,jj+kj) >= curr || d2.at<uchar>(ii+ki,jj+kj) > curr || d3.at<uchar>(ii+ki,jj+kj) >= curr)
								isMax = false;
							
							// check for local min
							if(d1.at<uchar>(ii+ki,jj+kj) <= curr || d2.at<uchar>(ii+ki,jj+kj) < curr || d3.at<uchar>(ii+ki,jj+kj) <= curr)
								isMin = false;
						}
						if(!isMax && !isMin)
							break;
					}
					// store a min/max value in the result mat
					if(isMax || isMin)
						res.at<uchar>(ii, jj) = 255;
					else
						res.at<uchar>(ii, jj) = 0;
				}
			}
			scale_extrema.push_back(res);
		}
		extrema.push_back(scale_extrema);
	}
	

	// reject useless features (along edges and low contrast)
	// keep only corners by calculating gradients at each pixel
	// http://aishack.in/tutorials/sift-scale-invariant-feature-transform-keypoints/
	vector<vector<cv::Mat> > corners; 			// stores strong corners
	for(int i=0; i<extrema.size(); i++)
	{	 
		vector<cv::Mat> filtered_extrema;
		for(int j=0; j<extrema[i].size(); j++)
		{
			// grab the gray image for gradient calculation
			Mat gray = octaves[i][j];
			Mat ext = extrema[i][j];
			// TODO assert gray.size == ext.size
			Mat dGray = gray.clone();
			Mat res = Mat::zeros(extrema[i][j].size(), extrema[i][j].type());

			imshow("gray", gray);
			imshow("ext", ext);
			cout << ext.type() << endl;

			// iterate through the DoG image
			for(int ii=1; ii<gray.rows-1; ii++)
			{
				for(int jj=1; jj<gray.cols-1; jj++)
				{
					//cout << ii << " " << jj << endl;
					// calculate gradient for extrema only
					if(ext.at<uchar>(ii,jj) != 0)
					{
						//cout << ext[ii][jj] << endl;
						Mat edge_kernel_x = (Mat_<int>(3,3) << 1, 0, -1, 2, 0, -2, 1, 0, -1);
						Mat edge_kernel_y = (Mat_<int>(3,3) << 1, 2, 1, 0, 0, 0, -1, -2, -1);
						// TODO assert rows == cols and sizex == sizey

						int dx = 0, dy = 0;
						// check for first DoG image
						// i2/j2 index in local region of the image
						// ki/kj index through the kernel
						for(int ki=0, i2=-edge_kernel_x.rows/2; ki<edge_kernel_x.rows; ki++, i2++)
						{
							for(int kj=0, j2=-edge_kernel_x.rows/2; kj<edge_kernel_x.rows; kj++, j2++)
							{	
								//cout << ki << "," << kj << ": " << (int)edge_kernel_x.at<int>(ki, kj) << " " << (int)edge_kernel_y.at<int>(ki, kj) << " " << (int)gray.at<uchar>(ii+i2, jj+j2) << endl;
								dx += (int)edge_kernel_x.at<int>(ki, kj) * (int)gray.at<uchar>(ii+i2, jj+j2);
								dy += (int)edge_kernel_y.at<int>(ki, kj) * (int)gray.at<uchar>(ii+i2, jj+j2);
							}
						}
						// compute products of the derivatives
						double dx2 = dx * dx;
						double dy2 = dy * dy;
						double R = dx2/dy2;
						
						// store a min/max value in the result mat
						if(R > 500)
							res.at<uchar>(ii, jj) = 255;
						else
							res.at<uchar>(ii, jj) = 0;
					}
				}
			}

			imshow("filtered", res);
			waitKey(-1);
			filtered_extrema.push_back(res);
		}

		
		corners.push_back(filtered_extrema);
	}
	
	

}


int main()
{
	Mat img = imread("/home/kyle/Dropbox/catkin_ws/src/camera_publisher/test/lenna.png");
	imshow("orig", img);
	waitKey(-1);
	SiftDetector d = SiftDetector(4, 5);

	d.detect(img);
}