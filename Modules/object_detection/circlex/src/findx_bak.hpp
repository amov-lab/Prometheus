#ifndef __FINDX_H__
#define __FINDX_H__
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <math.h>
#include <iostream>
#include<string>
//#include"findx.h"
using namespace cv;
using namespace std;
typedef std::vector<cv::Point>    PointsVector;
typedef std::vector<PointsVector> ContoursVector;
/**
 * @brief findX
 * @param img 输入用于识别的图片
 * @param p 输出识别中心的像素坐标
 * @return 识别成功为true
 */
bool findX(cv::Mat & img, cv::Point & p);

struct LinePoint
{
    float length;
    Point point;
};

struct RightAngle
{
    Point vertex;
    float rad1;
    float rad2;
};

bool SortLinePoint(const LinePoint &p1,const LinePoint &p2)
{
    return p1.length<p2.length;
}

float GetLineRad(Point p1,Point p2)
{
    float rad;
    if(p1.x!=p2.x)
    {
        rad=atan(((float)p2.y-(float)p1.y)/((float)p2.x-(float)p1.x));
        if(rad<0)//范围定为0-pi;
        {
            rad=rad+CV_PI;
        }
    }
    else
    {
        rad=CV_PI/2;
    }
    return rad;
}

float GetLinesRad(Point p0,Point p1,Point p2)
{
    float rad=GetLineRad(p0,p1)-GetLineRad(p1,p2);
    if(rad<0)
    {
        rad=rad+CV_PI;
    }
    if(rad>CV_PI/2)
    {
        rad=CV_PI-rad;
    }
    return rad;
}

bool IsIn(float p1,float p2,float distance)
{
    float dis=abs(p2-p1);
    if(dis<distance) return true;
    else return false;
}

float GetDistance(Point p1,Point p2)
{
    Point p=p1-p2;
    return sqrt(p.dot(p));
}

bool AtMargin(Point p,Size s)
{
    if(p.x<5||p.x>(s.width-5))
        return true;
    if(p.y<5||p.y>(s.height-5))
        return true;
    return false;
}
bool findx(Mat &grayImg, Point &p);
bool findX(Mat &img, Point &p)
{
    Mat image,grayImg;
    //pyrDown(img,image,Size(img.cols/2,img.rows/2));
    img.copyTo(image);
    //转为灰度图
    cvtColor(image,grayImg,CV_BGR2GRAY);
    // smooth it, otherwise a lot of false circles may be detected
//    GaussianBlur( grayImg, grayImg, Size(9, 9), 2, 2 );
//    imshow("gauss",grayImg);
    vector<Vec3f> circles;
    //找圆
    HoughCircles(grayImg, circles, CV_HOUGH_GRADIENT,
                 2, grayImg.rows/2, 200, 200,25 );
    //截取圆的范围再找x
    Mat roiImg;
    float cx,cy,cr;
    int ran[4];
    for(int i=0;i<circles.size();i++)
    {
        cx=circles[i][0];
        cy=circles[i][1];
        cr=circles[i][2];
        ran[0]=cx-cr;
        ran[1]=cx+cr;
        ran[2]=cy-cr;
        ran[3]=cy+cr;

        ran[0]=ran[0]>0?ran[0]:0;
        ran[1]=ran[1]<(image.cols-1)?ran[1]:(image.cols-1);
        ran[2]=ran[2]>0?ran[2]:0;
        ran[3]=ran[3]<(image.rows-1)?ran[3]:(image.rows-1);

        roiImg=grayImg(Range(ran[2],ran[3]),Range(ran[0],ran[1]));
        if(findx(roiImg,p))
        {
            p.x=p.x+ran[0];
            p.y=p.y+ran[2];
            return true;
        }
        //imshow("ori",roiImg);
    }

    return findx(grayImg,p);
}
//
bool findx(Mat & grayImg,Point & p)
{

    Mat thresholdImg;

    //转为二值图
    Scalar meansca= mean(grayImg);
    int me=meansca[0]*1.4;
    me=me>140?140:me;
    //cout<<me<<endl;
    threshold(grayImg,thresholdImg,me,255,THRESH_BINARY_INV);

    //imshow("th",thresholdImg);

    //搜索所有轮廓
    ContoursVector allContours,contours;
    findContours(thresholdImg,allContours, CV_RETR_LIST,
                 CV_CHAIN_APPROX_NONE);

    //Canny(grayImg,);

    for (size_t i = 0; i < allContours.size(); i++)
    {
        if (allContours[i].size() > 50)
        {
            contours.push_back(allContours[i]);//点数太少的不要
        }
    }

    PointsVector approxCurve;
    vector<RightAngle> possibleAngle;
    for(size_t i=0;i<contours.size();i++)
    {
        approxPolyDP(contours[i], approxCurve, 10, true);//近似多边形
        int curveSize=approxCurve.size();
        //polylines(image,approxCurve,true,Scalar(0,0,255));
        if(curveSize<3||curveSize>80) continue;//边数太多的不要
        //if(!isContourConvex(approxCurve)) continue;//不是凸的也得要
        //计算多边形最长边的长度
        float maxDist=0;
        for(int j=0;j<approxCurve.size();j++)
        {
            Point side=approxCurve[j] - approxCurve[(j + 1) % curveSize];
            float squaredSideLength = side.dot(side);
            maxDist=std::max(maxDist,squaredSideLength);
        }
        if(maxDist<1000) continue;//太小的多边形不要


        //寻找直角顶点
        float rad;
        for(int j=0;j<curveSize;j++)
        {

            rad=GetLinesRad(approxCurve[j],
                            approxCurve[(j + 1) % curveSize],
                    approxCurve[(j + 2) % curveSize]);
            if(IsIn(rad,CV_PI/2,CV_PI/18))
            {
                RightAngle possible;
                possible.vertex=approxCurve[(j + 1) % curveSize];
                if(AtMargin(possible.vertex,Size(grayImg.cols,grayImg.rows)))
                    continue;//不可能在画面边缘

                possible.rad1=GetLineRad(approxCurve[j],
                                         approxCurve[(j + 1) % curveSize]);
                possible.rad2=GetLineRad(approxCurve[(j + 1) % curveSize],
                        approxCurve[(j + 2) % curveSize]);
                possibleAngle.push_back(possible);
            }
        }
    }

    //在所有可能的直角中找到X的中心
    int poCo=possibleAngle.size();
    PointsVector truePoints;
    vector<LinePoint> dists,trueDists;
    /*
     * 当点数多于4时，找到与其他点连线之和最短的点
     */
    if(poCo>=4)
    {
        for(int i=0;i<poCo;i++)
        {
            vector<float> dis;
            float dist;
            //得到与其他点的距离
            for(int j=0;j<poCo;j++)
            {
                if(j==i)continue;
                dis.push_back(GetDistance(possibleAngle.at(i).vertex,
                                                possibleAngle.at(j).vertex));
            }
            std::sort(dis.begin(),dis.end());//从小到大排序
            dist=dis[0]+dis[1]+dis[2];//最短的三个点求和
            dists.push_back(LinePoint{dist,possibleAngle.at(i).vertex});
        }
        std:sort(dists.begin(),dists.end(),SortLinePoint);//再排序
        for(int i=0;i<4;i++)
        {
            //筛选出离得最近的四个点，并且距离不应比距离最小的大太多
            if(dists.at(i).length<dists.at(0).length*5)
            {
                truePoints.push_back(dists.at(i).point);
                trueDists.push_back(dists.at(i));
            }
        }
    }
    else
    {
        for(int i=0;i<possibleAngle.size();i++)
        {
            truePoints.push_back(possibleAngle.at(i).vertex);
        }
    }
    //判断正方形
    if(truePoints.size()==4)
    {
        std::sort(trueDists.begin(),trueDists.end(),SortLinePoint);
        float avelong=0,err=0;
        for(int j=0;j<4;j++)
        {
            avelong=avelong+trueDists.at(j).length;
        }
        avelong=avelong/4;
        for(int j=0;j<4;j++)
        {
            err=abs(avelong-trueDists.at(j).length);
            //cout<<"err rate:"<<err/avelong<<"avelong"<<avelong<<endl;
            if(err/avelong>(0.03+5/avelong))//四条边长相等
            {
                truePoints.clear();
                return false;
                break;
            }
        }
        //TODO：两条对角线相等
    }
    else
    {
        truePoints.clear();
        return false;
    }
    //将四个点按顺序排列
    if(!isContourConvex(truePoints))
    {
        swap(truePoints[2],truePoints[3]);
        //polylines(image,truePoints,false,Scalar(0,255,255),2);
        if(!isContourConvex(truePoints))
        {
            swap(truePoints[1],truePoints[2]);
            //polylines(image,truePoints,false,Scalar(255,0,255),1);
            //imshow("img",image);
            if(!isContourConvex(truePoints))
                return false;
        }
    }
    //判断中心的颜色
    vector<Point2f> markerCorners,//找到的点
            squareCorners;//正方形
    int markerSize=100;
    squareCorners.push_back(Point(0,0));
    squareCorners.push_back(Point(markerSize-1,0));
    squareCorners.push_back(Point(markerSize-1,markerSize-1));
    squareCorners.push_back(Point(0,markerSize-1));
    for(int i=0;i<4;i++)
    {
        markerCorners.push_back(truePoints[i]);
    }
    Mat markerTransform=getPerspectiveTransform(markerCorners,squareCorners);


    Mat squareImg;
    warpPerspective(grayImg,squareImg,markerTransform,Size(markerSize,markerSize));
    //imshow("square",squareImg);
    Scalar meanSquare=mean(squareImg);
    if(meanSquare[0]<me)
    {//如果颜色不够白，也不行
        return false;
    }
    if(truePoints.size()>0)
    {
        Scalar center=mean(truePoints);
        p.x=center[0];
        p.y=center[1];
        return true;
    }
    else
        return false;
}


#endif
