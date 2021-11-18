#include "DAS_Detect.hpp"



void DASDetect::init()
{
	cal = 0;
	width = 100;
	height = 10;
	rec_width = 40;
	Harris_num = 0;
	flag2 = 0;
	//mask = Mat(Size(1, 300), CV_8UC1);
	vehicle_speed = 1;
	limit_of_check = 2120;
	scale = 1; //设置缩放倍数
	margin = 2  ;//帧间隔
	limit_dis_epi =2;  //距离极线的距离
}

bool DASDetect::ROI_mod(int x1, int y1)
{
	if (x1 >= width / 16 && x1 <= width - width / 16 && y1 >= height / 3 && y1 <= height - height / 6) return 1;
	return 0;
}


int DASDetect::NMS(vector<Rect>& srcRects, vector<Rect>& resRects,float thresh)
{
    const size_t size = srcRects.size();
    if (size == 0)
    {
        return 0;
    }

    // Sort the bounding boxes by the bottom - right y - coordinate of the bounding box
    multimap<int, size_t> idxs;
    for (size_t i = 0; i < size; ++i)
    {
        idxs.insert(std::pair<int, size_t>(srcRects[i].br().y, i));
    }


    // keep looping while some indexes still remain in the indexes list
    while (idxs.size() > 0)
    {
        // grab the last rectangle
        multimap<int, size_t>::iterator lastElem = --idxs.end();
        Rect rect1 = srcRects[(*lastElem).second];

        resRects.push_back(rect1);

        idxs.erase(lastElem);

        for ( multimap<int, size_t>::iterator pos = idxs.begin(); pos != idxs.end(); )
        {
            // grab the current rectangle
            Rect rect2 = srcRects[(*pos).second];

            float intArea = (rect1 & rect2).area();
            float unionArea = rect1.area() + rect2.area() - intArea;
            float overlap = intArea / unionArea;

            // if there is sufficient overlap, suppress the current bounding box
            if (overlap > thresh)
            {
               idxs.erase(pos);
		           pos++;
            }
            else
            {
                pos++;
            }
        }
    }
    return 1;
}


void DASDetect::preprocess()
	{
		//图像预处理
		Harris_num = 0;
		F_prepoint.clear();
		F_nextpoint.clear();
		prepoint.clear();
		nextpoint.clear();
		resRect.clear();
		height = frame.rows*scale;
		width = frame.cols*scale;
		dsize = Size(frame.cols*scale, frame.rows*scale);
		//img_scale = Mat(dsize, CV_32SC3);
		//img_temp = Mat(dsize, CV_32SC3);
		resize(frame, img_scale, dsize);
		resize(frame, img_temp, dsize);
		cvtColor(img_scale, gray, COLOR_RGB2GRAY);
		//框框大小
		rec_width = frame.cols / 15;

		if (numFrame %50 == 0) {
			pre_result = Rect(0,0,width*2/3,height*2/3);
		}

	//	cout << " cal :   " << cal << endl;
	//	cout << "行: " << img_scale.rows << "    列: " << img_scale.cols << endl;
		//equalizeHist(gray, gray); //直方图均衡
		return;
	}

void DASDetect::optical_flow_check()
	{
		int limit_edge_corner = 5;
		for (int i = 0; i < state.size(); i++)
			if (state[i] != 0)
			{

			   int dx[10] = { -1, 0, 1, -1, 0, 1, -1, 0, 1 };
			   int dy[10] = { -1, -1, -1, 0, 0, 0, 1, 1, 1 };
			   int x1 = prepoint[i].x, y1 = prepoint[i].y;
			   int x2 = nextpoint[i].x, y2 = nextpoint[i].y;

				 //特征点在太边缘去掉
			   if ((x1 < limit_edge_corner || x1 >= gray.cols - limit_edge_corner || x2 < limit_edge_corner || x2 >= gray.cols - limit_edge_corner
				|| y1 < limit_edge_corner || y1 >= gray.rows - limit_edge_corner || y2 < limit_edge_corner || y2 >= gray.rows - limit_edge_corner))
			   {
				   state[i] = 0;
				   continue;
			   }
			double sum_check = 0;
			for (int j = 0; j < 9; j++)
				sum_check += abs(prevgray.at<uchar>(y1 + dy[j], x1 + dx[j]) - gray.at<uchar>(y2 + dy[j], x2 + dx[j]));
			if (sum_check>limit_of_check) state[i] = 0;

			if (state[i])
			 {
				Harris_num++;
				F_prepoint.push_back(prepoint[i]);
				F_nextpoint.push_back(nextpoint[i]);
			 }
			}

}

bool DASDetect::stable_judge()
{
			int stable_num = 0;
			double limit_stalbe = 0.5;
			for (int i = 0; i < state.size(); i++)
				if (state[i])
				{
				if (sqrt((prepoint[i].x - nextpoint[i].x)*(prepoint[i].x - nextpoint[i].x) + (prepoint[i].y - nextpoint[i].y)*(prepoint[i].y - nextpoint[i].y)) < limit_stalbe) stable_num++;
				}
			if (stable_num*1.0 / Harris_num > 0.2) return 1;
			return 0;
		}

Rect DASDetect::detectMain(Mat& m_frame,Mat& im_result,int num_Frame)
{
			frame = m_frame.clone();
			im_result = m_frame.clone();
			numFrame = num_Frame;
			Rect result;
			//图像预处理
			preprocess();


		 if(num_Frame!=1 && prevgray.data)
		 {

			 //calcOpticalFlowPyrLK光流
			 goodFeaturesToTrack(prevgray, prepoint, 200, 0.01, 8, Mat(), 3, true, 0.04);
			 cornerSubPix(prevgray, prepoint, Size(10, 10), Size(-1, -1), TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03));
			 if(prepoint.size() == 0)
			 {
			 	   	result = Rect(0,0,0,0);
					prevgray = gray.clone();
					return result;
			 }

			 calcOpticalFlowPyrLK(prevgray, gray, prepoint, nextpoint, state, err, Size(22, 22), 5, TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.01));
			
			 optical_flow_check();
			 
			 //画出所有角点

			 for (int i = 0; i < state.size(); i++)
			 {

				 double x1 = prepoint[i].x, y1 = prepoint[i].y;
				 double x2 = nextpoint[i].x, y2 = nextpoint[i].y;
				 /*
				 if (state[i] != 0)
				 {
					 //画出所有角点
					 circle(img_scale, nextpoint[i], 3, Scalar(255, 0, 255));
					 circle(pre_frame, prepoint[i], 2, Scalar(255, 0, 255));
				 }
				 */
			 }

			 //-----------------------------计算 F-Matrix
				 Mat F;// = Mat(3, 3, CV_32FC1);
				 //F = findFundamentalMat(F_prepoint, F_nextpoint, mask, FM_RANSAC, 2, 0.99);
				 double ppp = 110;
				 //Mat L = Mat(1000, 3, CV_32FC1);
				 
				 int loop =0;
				 while (ppp > 5 && loop <30)
				 {
					 vector<Point2f> F2_prepoint, F2_nextpoint;
					 F2_prepoint.clear();
					 F2_nextpoint.clear();
					 ppp = 0;

					 if(F_prepoint.size() == 0)
					 {
					 	result = Rect(0,0,0,0);
				        prevgray = gray.clone();
					    return result;
					 }
					 F = findFundamentalMat(F_prepoint, F_nextpoint, mask, FM_RANSAC, 0.1, 0.99);

					 //cout << F << endl;
					 //computeCorrespondEpilines(F_prepoint,1,F,L);

					 for (int i = 0; i < mask.rows; i++)
					 {
						 if (mask.at<uchar>(i, 0) == 0);
						 else
						 {
							 ///circle(pre_frame, F_prepoint[i], 6, Scalar(255, 255, 0), 3);
							 double A = F.at<double>(0, 0)*F_prepoint[i].x + F.at<double>(0, 1)*F_prepoint[i].y + F.at<double>(0, 2);
							 double B = F.at<double>(1, 0)*F_prepoint[i].x + F.at<double>(1, 1)*F_prepoint[i].y + F.at<double>(1, 2);
							 double C = F.at<double>(2, 0)*F_prepoint[i].x + F.at<double>(2, 1)*F_prepoint[i].y + F.at<double>(2, 2);
							 double dd = fabs(A*F_nextpoint[i].x + B*F_nextpoint[i].y + C) / sqrt(A*A + B*B);
							 //cout << "------:" << dd << "   " << F_prepoint[i].x << "   " << F_prepoint[i].y << endl;
							 //cout << "A:  " << A << "   B: " << B << "   C:  " << C << endl;
							 ppp += dd;
							 if (dd > 0.1)
							 {
								 //circle(pre_frame, F_prepoint[i], 6, Scalar(255, 0, 0), 3);
							 }
							 else
							 {
								 F2_prepoint.push_back(F_prepoint[i]);
								 F2_nextpoint.push_back(F_nextpoint[i]);
							 }
						 }
					 }

					 F_prepoint = F2_prepoint;
					 F_nextpoint = F2_nextpoint;
					 loop++;
				 }

				 //T：异常角点集
				 vector<Point2f> T;
				 T.clear();

				 for (int i = 0; i < prepoint.size(); i++)
				 {
					 if (state[i] != 0)
					 {
						 double A = F.at<double>(0, 0)*prepoint[i].x + F.at<double>(0, 1)*prepoint[i].y + F.at<double>(0, 2);
						 double B = F.at<double>(1, 0)*prepoint[i].x + F.at<double>(1, 1)*prepoint[i].y + F.at<double>(1, 2);
						 double C = F.at<double>(2, 0)*prepoint[i].x + F.at<double>(2, 1)*prepoint[i].y + F.at<double>(2, 2);
						 double dd = fabs(A*nextpoint[i].x + B*nextpoint[i].y + C) / sqrt(A*A + B*B);

						 //画光流
						 int x1 = (int)prepoint[i].x, y1 = (int)prepoint[i].y;
						 int x2 = (int)nextpoint[i].x, y2 = (int)nextpoint[i].y;
						 //if (sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1)) < limit_flow) continue;
						 //line(img_scale, Point((int)prepoint[i].x, (int)prepoint[i].y), Point((int)nextpoint[i].x, (int)nextpoint[i].y), Scalar(255, 255, 0 ) , 2);
						 //line(pre_frame, Point((int)prepoint[i].x, (int)prepoint[i].y), Point((int)nextpoint[i].x, (int)nextpoint[i].y), Scalar(0, 255, 0 ) , 1);


						 //距离的极线阈值
						 if (dd <= limit_dis_epi) continue;
						 //cout << "dis: " << dd << endl;
						 dis[T.size()] = dd;
						 T.push_back(nextpoint[i]);

						 //画异常角点
						 //circle(img_scale, nextpoint[i], 7, Scalar(255, 255, 255),3);
						 //circle(pre_frame, prepoint[i], 3, Scalar(255, 255, 255), 2);

						 //画极线
						 if (fabs(B) < 0.0001)
						 {
							 double xx = C / A, yy = 0;
							 double xxx = C / A, yyy = gray.cols;
							 //line(pre_frame, Point(xx, yy), Point(xxx, yyy), Scalar::all(-1), 0.01);
							 flag2++;
							 continue;
						 }
						 double xx = 0, yy = -C / B;
						 double xxx = gray.cols, yyy = -(C + A*gray.cols) / B;
						 if (fabs(yy) > 12345 || fabs(yyy) > 12345)
						 {
							 yy = 0;
							 xx = -C / A;
							 yyy = gray.rows;
							 xxx = -(C + B*yyy) / A;
						 }
						 //line(img_scale, Point(xx, yy), Point(xxx, yyy), Scalar::all(-1), 0.01);
						 //line(pre_frame, Point(xx, yy), Point(xxx, yyy), Scalar::all(-1), 0.01);
					 }
				 }


					 //画框 枚举 mod
					 int tt = 10;
					 double flag_meiju[100][100];
					 memset(flag_meiju, 0, sizeof(flag_meiju));
					 for (int i = 0; i < gray.rows / tt; i++)
						 for (int j = 0; j < gray.cols / tt; j++)
						 {
						 double x1 = i*tt + tt / 2;
						 double y1 = j*tt + tt / 2;
						 for (int k = 0; k < T.size(); k++)
							 if (ROI_mod(T[k].x, T[k].y) && sqrt((T[k].x - y1)*(T[k].x - y1) + (T[k].y - x1)*(T[k].y - x1)) < tt*sqrt(2)) flag_meiju[i][j]++;//flag_meiju[i][j] += dis[k];
						 }

					 double mm = 0;
					 int mark_i = 0, mark_j = 0;
					 for (int i = 0; i < gray.rows / tt; i++)
						 for (int j = 0; j < gray.cols / tt; j++)
							 if (ROI_mod(j*tt, i*tt) && flag_meiju[i][j] > mm)
							 {
									mark_i = i;
									mark_j = j;
									mm = flag_meiju[i][j];
									if (mm < 2) continue;
									//rectangle(im_result, Point(mark_j*tt / scale - rec_width, mark_i*tt / scale + rec_width), Point(mark_j*tt / scale + rec_width, mark_i*tt / scale - rec_width), Scalar(0, 255, 255), 3);
									Rect rect(Point(mark_j*tt / scale - rec_width, mark_i*tt / scale + rec_width), Point(mark_j*tt / scale + rec_width, mark_i*tt / scale - rec_width));
									resRect.push_back(rect);
							 }


					 if (mm > 1111)
					 {
						// rectangle(im_result, Point(mark_j*tt / scale - rec_width, mark_i*tt / scale + rec_width), Point(mark_j*tt / scale + rec_width, mark_i*tt / scale - rec_width),
						              // Scalar(0, 255, 255), 3);
						 Rect rect(Point(mark_j*tt / scale - rec_width, mark_i*tt / scale + rec_width), Point(mark_j*tt / scale + rec_width, mark_i*tt / scale - rec_width));
						 resRect.push_back(rect);
					 }

					 if (resRect.size() != 0 ) 
				{
					 	vector<Rect> nms_resRect;
						NMS(resRect,nms_resRect,0.3);

					 if (nms_resRect.size() != 1)
					{
						  float min_distance = 1000;
							int num = 0;
							//非极大值抑制之后结果不唯一，选择距离上一帧最近的
							for (size_t k = 0; k < nms_resRect.size(); k++) {
								float distance = fabs(nms_resRect[k].x-pre_result.x)+fabs(nms_resRect[k].y-pre_result.y);
								if (distance < min_distance) {
									min_distance = distance;
									num = k;
								}
							}
							result = nms_resRect[num];

					}

					 else
					{
						 result = nms_resRect[0];
					}

				 //绘制ROI
				//	rectangle(frame, Point(width / 16 / scale, height * 5 / 6 / scale), Point((width - width / 16) / scale, height / 3 / scale), Scalar(255, 0, 0), 1, 0);


				}
				 else
				 {
					 result = Rect(0,0,0,0);
				 }
				 pre_result = result;
				 rectangle(im_result,result,Scalar(0,0,255),3);
	    }
	  else {
	  		
	    	result = Rect(0,0,0,0);
	      }


	prevgray = gray.clone();
	//rectangle(im_result,result,Scalar(0,0,255),3);


	return result;
}
