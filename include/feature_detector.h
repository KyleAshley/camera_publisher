
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// interface class for a generic feature detector
class IFeatureDetector
{


	public:
		virtual void detect(cv::Mat img) = 0;

	private:

};

