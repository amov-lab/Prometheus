#ifndef SPIRE_ELLIPSE_HPP_
#define SPIRE_ELLIPSE_HPP_
#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>

namespace spire {

typedef std::vector<cv::Point> VP;
typedef std::vector< VP >      VVP;
typedef unsigned int uint;

#define _INFINITY 1024


int inline sgn(float val) {
	return (0.f < val) - (val < 0.f);
}


bool inline isInf(float x)
{
	union
	{
		float f;
		int	  i;
	} u;

	u.f = x;
	u.i &= 0x7fffffff;
	return !(u.i ^ 0x7f800000);
}


float inline slope(float x1, float y1, float x2, float y2)
{
	//reference slope
	float den = float(x2 - x1);
	float num = float(y2 - y1);
	if (den != 0)
	{
		return (num / den);
	}
	else
	{
		return ((num > 0) ? float(_INFINITY) : float(-_INFINITY));
	}
}

//void cvCanny2(	const void* srcarr, void* dstarr,
//				double low_thresh, double high_thresh,
//				void* dxarr, void* dyarr,
//                int aperture_size );
//
//void cvCanny3(	const void* srcarr, void* dstarr,
//				void* dxarr, void* dyarr,
//                int aperture_size );

void Canny2(cv::InputArray image, cv::OutputArray _edges,
	cv::OutputArray _sobel_x, cv::OutputArray _sobel_y,
	double threshold1, double threshold2,
	int apertureSize, bool L2gradient);

void Canny3(cv::InputArray image, cv::OutputArray _edges,
	cv::OutputArray _sobel_x, cv::OutputArray _sobel_y,
	int apertureSize, bool L2gradient);

void Canny4(cv::InputArray image, cv::OutputArray _edges,
	cv::OutputArray _sobel_x, cv::OutputArray _sobel_y,
	int apertureSize, bool L2gradient);


float inline ed2(const cv::Point& A, const cv::Point& B)
{
	return float(((B.x - A.x)*(B.x - A.x) + (B.y - A.y)*(B.y - A.y)));
}

float inline ed2f(const cv::Point2f& A, const cv::Point2f& B)
{
	return (B.x - A.x)*(B.x - A.x) + (B.y - A.y)*(B.y - A.y);
}

cv::Point2f lineCrossPoint(cv::Point2f l1p1, cv::Point2f l1p2, cv::Point2f l2p1, cv::Point2f l2p2);
void point2Mat(cv::Point2f p1, cv::Point2f p2, float mat[2][2]);

#define V2SP cv::Point2f p3,cv::Point2f p2,cv::Point2f p1,cv::Point2f p4,cv::Point2f p5,cv::Point2f p6
float value4SixPoints(V2SP);

void show_contours(cv::Mat1b& image, VVP& segments, const char* title = "contours");
void find_contours(cv::Mat1b& image, VVP& segments, int iMinLength);

void find_contours_rect(cv::Mat1b& image, VVP& segments, int iMinLength, std::vector<cv::Rect>& bboxes);
void thinning(cv::Mat1b& imgMask, uchar byF = 255, uchar byB = 0);

bool SortBottomLeft2TopRight(const cv::Point& lhs, const cv::Point& rhs);
bool SortTopLeft2BottomRight(const cv::Point& lhs, const cv::Point& rhs);

bool SortBottomLeft2TopRight2f(const cv::Point2f& lhs, const cv::Point2f& rhs);
float GetMinAnglePI(float alpha, float beta);

struct Ellipse
{
	float _xc;
	float _yc;
	float _a;
	float _b;
	float _rad;
	float _score;

    Ellipse() : _xc(0.f), _yc(0.f), _a(0.f), _b(0.f), _rad(0.f), _score(0.f) {}
    Ellipse(float xc, float yc, float a, float b, float rad, float score = 0.f) : _xc(xc), _yc(yc), _a(a), _b(b), _rad(rad), _score(score) {}
    Ellipse(const Ellipse& other) : _xc(other._xc), _yc(other._yc), _a(other._a), _b(other._b), _rad(other._rad), _score(other._score) {}

	void Draw(cv::Mat& img, const cv::Scalar& color, const int thickness)
	{
		ellipse(img, cv::Point(cvRound(_xc), cvRound(_yc)), cv::Size(cvRound(_a), cvRound(_b)), _rad * 180.0 / CV_PI, 0.0, 360.0, color, thickness);
    }

	void Draw(cv::Mat3b& img, const int thickness)
	{
		cv::Scalar color(0, cvFloor(255.f * _score), 0);
		ellipse(img, cv::Point(cvRound(_xc), cvRound(_yc)), cv::Size(cvRound(_a), cvRound(_b)), _rad * 180.0 / CV_PI, 0.0, 360.0, color, thickness);
    }

	bool operator<(const Ellipse& other) const
	{	// use for sorting
		if (_score == other._score)
		{
			float lhs_e = _b / _a;
			float rhs_e = other._b / other._a;
			if (lhs_e == rhs_e)
			{
				return false;
			}
			return lhs_e > rhs_e;
		}
		return _score > other._score;
    }
};


}
#endif // USE_OPENCV
#endif // SPIRE_ELLIPSE_HPP_
