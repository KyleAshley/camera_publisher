#include "feature_detect.h"


featureDetector::featureDetector(int num_octaves, int num_levels)
{


	this->num_octaves = num_octaves;
	this->num_levels = num_levels;
}


featureDetector::~featureDetector()
{
 	

}


void featureDetector::detect(cv::Mat img)
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
	vector<vector<cv::Mat> > dog; 
	for(int i=0; i<this->num_octaves; i++)
	{	 
		vector<cv::Mat> diffs;
		for(int j=0; j<this->num_levels-1; j++)
		{
			cv::Mat diff = Mat::zeros(this->octaves[i][j].size(), this->octaves[i][j].type());
			diff = this->octaves[i][j] - this->octaves[i][j+1];
			diffs.push_back(diff);
		}
		dog.push_back(diffs);
	}

	// TODO find approximate min/max among scales
	// http://aishack.in/tutorials/sift-scale-invariant-feature-transform-keypoints/
	vector<vector<cv::Mat> > extrema; 
	for(int i=0; i<this->num_octaves; i++)
	{	 
		vector<cv::Mat> scale_extrema;
		for(int j=0; j<this->num_levels-2; j++)
		{
			Mat d1, d2, d3;
			cv::Mat res = Mat::zeros(this->octaves[i][j].size(), this->octaves[i][j].type());
			d1 = this->octaves[i][j];
			d2 = this->octaves[i][j+1];
			d3 = this->octaves[i][j+2];

			int max=0;
			int min = INT_MAX;
			for(int ii=1; ii<d1.rows-1; ii++)
			{
				for(int jj=1; jj<d1.cols-1; jj++)
				{
					// compare relative to middle scale
					int curr = d2.at<uchar>(ii, jj);
					bool isMax = true;
					bool isMin = true;

					if(curr > 0)
					{
						// check for first DoG image
						for(int ki=-1; ki<1; ki++)
						{
							for(int kj=-1; kj<1; kj++)
							{
								// check for local max
								if(d1.at<uchar>(ii+ki,jj+kj) >= curr || d2.at<uchar>(ii+ki,jj+kj) > curr || d3.at<uchar>(ii+ki,jj+kj) >= curr)
								{
									isMax = false;
									//break;
								}
								
								// check for local min
								if(d1.at<uchar>(ii+ki,jj+kj) <= curr || d2.at<uchar>(ii+ki,jj+kj) < curr || d3.at<uchar>(ii+ki,jj+kj) <= curr)
								{
									isMin = false;
									//break;
								}
								
							}
							if(!isMax && !isMin)
								break;
						}
					}
					
					if(isMax || isMin)
						res.at<uchar>(ii, jj) = 255;
				}
				
			}

			imshow("extrema", res);
			waitKey(-1);

			scale_extrema.push_back(res);
		}
		extrema.push_back(scale_extrema);
	}


}


int main()
{
	Mat img = imread("/home/kyle/Dropbox/catkin_ws/src/camera_publisher/test/lenna.png");
	imshow("orig", img);
	waitKey(-1);
	featureDetector d = featureDetector(4, 5);

	d.detect(img);
}