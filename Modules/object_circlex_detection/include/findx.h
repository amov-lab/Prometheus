#ifndef __FIND_X__
#define __FIND_X__

#include <iostream>
#include <math.h>
#include <string>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;
typedef std::vector<cv::Point> PointsVector;
typedef std::vector<PointsVector> ContoursVector;

struct PointValue
{
    Point p;
    int value;
    Point p1;
    float rad;
    float length;
};

bool SortPointValue(const PointValue &p1, const PointValue &p2)
{ //从大到小
    return p1.value > p2.value;
}

float GetLineRad(Point &p1, Point &p2)
{
    float rad;
    Point2f p = p2 - p1;
    rad = atan2(p.y, p.x);
    return rad;
}
float GetLineRad(Vec4i &vec)
{
    Point2i p1, p2;
    p1.x = vec[0];
    p1.y = vec[1];
    p2.x = vec[2];
    p2.y = vec[3];
    return GetLineRad(p1, p2);
}

float GetRad(Point &p1, Point &p2)
{
    float rad;
    Point2f p = p2 - p1;
    rad = atan2(p.y, p.x);
    if (rad < 0)
        rad = rad + CV_PI;
    return rad;
}
float GetVertRad(float rad)
{
    float r = rad - CV_PI / 2;
    if (rad < 0)
        rad = rad + CV_PI;
}

float O2PI(float rad)
{
    return rad >= 0 ? rad : (rad + CV_PI);
}

bool IsIn(float p1, float p2, float distance)
{
    float dis = abs(p2 - p1);
    if (dis < distance)
        return true;
    else
        return false;
}
float GetDistance(Point p1, Point p2)
{
    Point p = p1 - p2;
    return sqrt(p.dot(p));
}
float GetDistance(int x1, int y1, int x2, int y2)
{
    Point p1, p2;
    p1.x = x1;
    p1.y = y1;
    p2.x = x2;
    p2.y = y2;
    return GetDistance(p1, p2);
}

int Limiter(int number, int min, int max)
{
    if (number > max)
        return max;
    if (number < min)
        return min;
    return number;
}

void Limiter(Point &p, Mat &m)
{
    p.x = Limiter(p.x, 0, m.cols - 1);
    p.y = Limiter(p.y, 0, m.rows - 1);
}

Point Limiter(int x, int y, Mat &m)
{
    Point lp;
    lp.x = Limiter(x, 0, m.cols - 1);
    lp.y = Limiter(y, 0, m.rows - 1);
    return lp;
}

Point GetPoint(Point &p1, Point &p2, float length, Mat &limit)
{
    float k = GetLineRad(p1, p2);
    Point p;
    p.x = p1.x + length * cos(k);
    p.y = p1.y + length * sin(k);
    Limiter(p, limit);
    return p;
}

void int2str(const int &int_temp, string &string_temp)
{
    stringstream stream;
    stream << int_temp;
    string_temp = stream.str(); //此处也可以用 stream>>string_temp
}

void Divide(const Point &beichu, float n, Point &ret)
{
    ret.x = beichu.x / n;
    ret.y = beichu.y / n;
}

float point2Line(Point &p1, Point &lp1, Point &lp2)
{
    float a, b, c, dis;
    // 化简两点式为一般式
    // 两点式公式为(y - y1)/(x - x1) = (y2 - y1)/ (x2 - x1)
    // 化简为一般式为(y2 - y1)x + (x1 - x2)y + (x2y1 - x1y2) = 0
    // A = y2 - y1
    // B = x1 - x2
    // C = x2y1 - x1y2
    a = lp2.y - lp1.y;
    b = lp1.x - lp2.x;
    c = lp2.x * lp1.y - lp1.x * lp2.y;
    // 距离公式为d = |A*x0 + B*y0 + C|/√(A^2 + B^2)
    dis = abs(a * p1.x + b * p1.y + c) / sqrt(a * a + b * b);
    return dis;
}

int GetGray(Mat &m, Point &p, int s = 3)
{
    Point plu = Limiter(p.x - s, p.y - s, m); //点左上
    Point prd = Limiter(p.x + s, p.y + s, m); //点右下
    Range row(plu.y, prd.y);
    Range col(plu.x, prd.x);
    //    Range row(Limiter(p.y-3,0,m.rows-1),Limiter(p.y+3,0,m.rows-1));
    //    Range col(Limiter(p.x-3,0,m.cols-1),Limiter(p.x+3,0,m.cols-1));
    Mat poi = m(row, col);
    Scalar me = mean(poi);
    return me[0];
}

bool findX(Mat &img, Point &p)
{
    Mat grayImg, bwImg;
    //设置静态变量，加快计算速度
    static Mat dilateElement, transMask, transImg;
    if (dilateElement.empty())
    {
        int dilation_size = 1;
        dilateElement = getStructuringElement(MORPH_ELLIPSE,
                                              Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                              Point(dilation_size, dilation_size));
    }
    Size transSize(60, 60);
    if (transMask.empty())
    {
        transMask = Mat(transSize.height, transSize.width, CV_8UC1);
        transMask = Scalar(0);
        rectangle(transMask, Point(19, 19), Point(59, 59), Scalar(255), -1);
    }

    static vector<Point2f> trans2d;
    if (trans2d.size() == 0)
    {
        trans2d.push_back(Point2f(0, 0));
        trans2d.push_back(Point2f(59, 0));
        trans2d.push_back(Point2f(59, 59));
        trans2d.push_back(Point2f(0, 59));
    }

    vector<Vec4i> lines;
    ContoursVector linesEx, possibleLines;
    vector<PointValue> resultPoints;

    cvtColor(img, grayImg, COLOR_RGB2GRAY);
    //
    GaussianBlur(grayImg, grayImg, Size(11, 11), 0); //可能非最佳参数
    //imshow("gauss blur",grayImg);
    Canny(grayImg, bwImg, 70, 150); //可能非最佳参数
    //imshow("canny",bwImg);
    //腐蚀
    dilate(bwImg, bwImg, dilateElement);
    //imshow("canny",bwImg);
    int64 t1, t2, t3;
    t1 = getTickCount();
    HoughLinesP(bwImg, lines, 2, CV_PI / 360, 60, 30, 3); //可能非最佳参数...慢
    t2 = getTickCount();
    int disAsOne = 6;

    Point pi[2], pj[2];
    for (int i = 0; i < lines.size(); i++)
    {
        pi[0].x = lines.at(i)[0];
        pi[0].y = lines.at(i)[1];
        pi[1].x = lines.at(i)[2];
        pi[1].y = lines.at(i)[3];
        //line(img,pi[0],pi[1],Scalar(255,255,0),2);

        float rad1 = O2PI(GetLineRad(lines.at(i)));

        for (int j = i + 1; j < lines.size(); j++)
        {
            if (i == j)
                continue;
            pj[0].x = lines.at(j)[0];
            pj[0].y = lines.at(j)[1];
            pj[1].x = lines.at(j)[2];
            pj[1].y = lines.at(j)[3];
            float rad2 = O2PI(GetLineRad(lines.at(j)));
            if (IsIn(abs(rad1 - rad2), CV_PI / 2, CV_PI / 3))
            {
                float dp[4];
                dp[0] = GetDistance(pi[0], pj[0]);
                dp[1] = GetDistance(pi[0], pj[1]);
                dp[2] = GetDistance(pi[1], pj[0]);
                dp[3] = GetDistance(pi[1], pj[1]);
                for (int k = 0; k < 4; k++)
                {
                    if (dp[k] < disAsOne)
                    {
                        PointsVector line;
                        line.push_back(pi[1 - k / 2]);
                        Point pij;
                        Divide(pi[k / 2] + pj[k % 2], 2, pij);
                        line.push_back(pij);
                        line.push_back(pj[1 - k % 2]);
                        linesEx.push_back(line);
                        break;
                    }
                }
            }
        }
    }

    bool linesToRemove[linesEx.size()];
    for (int i = 0; i < linesEx.size(); i++)
    {
        for (int j = i + 1; j < linesEx.size(); j++)
        {
            if (GetDistance(linesEx.at(i).at(1), linesEx.at(j).at(1)) < disAsOne)
            {
                linesToRemove[j] = true;
            }
        }
    }
    for (int i = 0; i < linesEx.size(); i++)
    {
        if (!linesToRemove[i])
            possibleLines.push_back(linesEx.at(i));
    }
    //polylines(img,possibleLines,false,Scalar(0,0,255),2);
    vector<Point2f> fpoints;
    for (int i = 0; i < possibleLines.size(); i++)
    {
        fpoints.clear();
        Point p[10];
        p[0] = possibleLines.at(i).at(0);
        p[1] = possibleLines.at(i).at(1);
        p[2] = possibleLines.at(i).at(2);
        float len = GetDistance(p[0], p[1]);
        //cout<<len<<endl;
        len = len * 0.4;
        p[3] = GetPoint(p[1], p[0], len * 2 / 3, grayImg);
        p[4] = GetPoint(p[1], p[2], len * 2 / 3, grayImg);
        Divide((p[3] + p[4]), 2, p[5]);
        p[6] = GetPoint(p[1], p[5], len * 2 / 3 * sqrt(2), grayImg);
        p[7] = GetPoint(p[6], p[3], len, grayImg);
        p[9] = GetPoint(p[6], p[4], len, grayImg);
        p[8] = GetPoint(p[5], p[1], len * 2 / 3 * sqrt(2), grayImg);
        //         p[3]=GetPoint(p[1],p[0],40,grayImg);
        //         p[4]=GetPoint(p[1],p[2],40,grayImg);
        //         p[5]=(p[3]+p[4])/2;
        //         p[6]=GetPoint(p[1],p[5],40*sqrt(2),grayImg);
        //         p[7]=GetPoint(p[6],p[3],60,grayImg);
        //         p[8]=GetPoint(p[5],p[1],40*sqrt(2),grayImg);
        //         p[9]=GetPoint(p[6],p[4],60,grayImg);
        fpoints.push_back(p[8]);
        fpoints.push_back(p[9]);
        fpoints.push_back(p[6]);
        fpoints.push_back(p[7]);
        Mat trans = getPerspectiveTransform(fpoints, trans2d);
        warpPerspective(grayImg, transImg, trans, transSize);

        Scalar s1 = mean(transImg, transMask);
        Scalar s2 = mean(transImg, 255 - transMask);
        if (s1[0] - s2[0] > 80) ///////////////////////////
        {
            PointValue pv;
            pv.p = p[1];
            pv.value = 0;
            pv.p1 = p[5];
            pv.length = len * 1.2;
            //pv.p2=p[4];
            resultPoints.push_back(pv);
        }
        //            vector<Point> pp;
        //            pp.push_back(p[8]);
        //            pp.push_back(p[9]);
        //            pp.push_back(p[6]);
        //            pp.push_back(p[7]);
        //            if(s1[0]-s2[0]>80)
        //            {
        //                circle(img,p[1],4,Scalar(0,0,255),-1);
        //                polylines(img,pp,true,Scalar(0,255,255));
        //            }
        //            else
        //            {
        //                polylines(img,pp,true,Scalar(255,255,255));
        //            }
    }

    for (int i = 0; i < resultPoints.size(); i++)
    {
        Point i1 = resultPoints.at(i).p - resultPoints.at(i).p1;
        float radi = atan2(i1.y, i1.x);

        for (int j = i + 1; j < resultPoints.size(); j++)
        {

            //if(i==j)continue;
            Point ij = resultPoints.at(j).p - resultPoints.at(i).p;
            float radij = atan2(ij.y, ij.x);
            if (IsIn(abs(radi - radij), 0, CV_PI / 3))
            {
                Point j1 = resultPoints.at(j).p - resultPoints.at(j).p1;
                float radj = atan2(j1.y, j1.x);
                Point ji = resultPoints.at(i).p - resultPoints.at(j).p;
                float radji = atan2(ji.y, ji.x);
                if (IsIn(abs(radj - radji), 0, CV_PI / 3))
                {
                    resultPoints.at(i).value++;
                    resultPoints.at(j).value++;
                    resultPoints.at(i).rad = radi;
                    resultPoints.at(j).rad = radj;
                }
            }
        }
    }

    //    cout<<resultPoints.size()<<endl;
    for (int i = 0; i < resultPoints.size(); i++)
    {
        for (int j = i + 1; j < resultPoints.size(); j++)
        {
            if (i == j)
                continue;
            if (IsIn(
                    abs(resultPoints.at(i).rad - resultPoints.at(j).rad),
                    CV_PI,
                    CV_PI / 18))
            {
                float dij = point2Line(resultPoints.at(j).p, resultPoints.at(i).p, resultPoints.at(i).p1);
                float dji = point2Line(resultPoints.at(i).p, resultPoints.at(j).p, resultPoints.at(j).p1);
                float len = GetDistance(resultPoints.at(i).p, resultPoints.at(j).p);
                if ((dij < len / 3) && (dji < len / 3))
                {
                    Divide(resultPoints.at(i).p + resultPoints.at(j).p, 2, p);
                    float gray = GetGray(grayImg, p);
                    float back = GetGray(grayImg, p, 100);
                    if (gray < back)
                        return true;
                }
            }
        }
    }
    //     sort(resultPoints.begin(),resultPoints.end(),SortPointValue);
    //     if(resultPoints.size()>4)
    //     {
    //         resultPoints.erase(resultPoints.begin()+4,resultPoints.end());
    //     }
    t3 = getTickCount();
    //cout<<"hough:"<<(t2-t1)/getTickFrequency()<<"\tother:"<<(t3-t2)/getTickFrequency()<<endl;
    //     for(int i=0;i<resultPoints.size();i++)
    //     {
    //         if(resultPoints.at(i).value>0)
    //         {
    //             circle(img,resultPoints.at(i).p,4,Scalar(0,0,255),-1);
    //         }
    //     }
    //string ss;
    //int2str(resultPoints.size(),ss);
    //putText(img,ss,Point(400,320),FONT_HERSHEY_SIMPLEX ,12,Scalar(0,0,225),2);
    //     cout<<lines.size()/3<<"\t"<<
    //           linesEx.size()<<"\t"<<
    //           possibleLines.size()<<"\t"<<
    //           resultPoints.size()<<"\t"<<endl;
    //cout<<resultPoints.size()<<endl;
    //     switch (resultPoints.size()) {
    //     case 4:
    //         Divide(resultPoints.at(0).p+
    //            resultPoints.at(1).p+
    //            resultPoints.at(2).p+
    //            resultPoints.at(3).p,4,p);
    //         return true;
    //         break;
    //     case 3:
    //         for(int i=0;i<3;i++)
    //         {
    //             if(IsIn(
    //                         abs(resultPoints.at(i).rad-resultPoints.at((i+1)%3).rad),
    //                         CV_PI,
    //                         CV_PI/4)
    //                     )
    //             {
    //                 Divide((resultPoints.at(i).p+resultPoints.at((i+1)%3).p),2,p);
    //                 return true;
    //             }
    //         }
    //         return false;
    //         break;
    ////     case 2:
    ////         if(IsIn(abs(resultPoints.at(0).rad-resultPoints.at(1).rad),CV_PI,CV_PI/4))
    ////         {
    ////             p=(resultPoints.at(0).p+resultPoints.at(1).p)/2;
    ////             return true;
    ////         }
    ////         else
    ////         {
    ////             float radx=(resultPoints.at(0).rad+resultPoints.at(1).rad)/2;
    ////             if(abs(resultPoints.at(0).rad-resultPoints.at(1).rad)>CV_PI)
    ////             {
    ////                 radx=radx+CV_PI;
    ////             }
    ////             Point xx=(resultPoints.at(0).p+resultPoints.at(1).p)/2;
    ////             Point xdis=(resultPoints.at(0).p-resultPoints.at(1).p)/2;
    ////             float xd=sqrt(xdis.dot(xdis));
    ////             p.x=xx.x+xd*cos(radx);
    ////             p.y=xx.y+xd*sin(radx);
    ////             return true;
    ////         }
    //     default:
    //         return false;
    //         break;
    //     }
    return false;
}

#endif
