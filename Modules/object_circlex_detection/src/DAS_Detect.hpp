#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include "opencv2/features2d/features2d.hpp"

using namespace cv;
using namespace std;

#define N 300

class DASDetect
{
public:
	int flag[N][N];
	Mat prevgray, gray, flow, cflow, frame, pre_frame, img_scale, img_temp, mask ;
	Size dsize;
	vector<Point2f> prepoint, nextpoint;
	vector<Point2f> F_prepoint, F_nextpoint;
	vector<Rect> resRect;
	vector<uchar> state;
	vector<float> err;
	double dis[N];
	int cal ;
	int width , height ;
	int rec_width;
	int Harris_num;
	int flag2;
	Rect pre_result;
	int numFrame;

	double vehicle_speed ;
	double limit_of_check ;
	double scale ; //设置缩放倍数
	int margin; //帧间隔
	double limit_dis_epi; //距离极线的距离

public:
	void init();
        bool ROI_mod(int x1, int y1);
	void preprocess();
	void optical_flow_check();
	bool stable_judge();
        Rect detectMain(Mat& m_frame,Mat& im_result,int numFrame);
	int NMS(vector<Rect>& srcRects, vector<Rect>& resRects,float thresh);
};
