#define USE_OPENCV
#define DETECT_MARKER_X_NET
#include <ros/ros.h>
#include <ros/package.h>
#include <chrono>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "caffe/caffe.hpp"
#include "caffe/util/spire_camera_reader.hpp"
#include "caffe/util/spire_video_writer.hpp"

#include "caffe/util/spire_cn_detector.h"
#include "DAS_Detect.hpp"
#include "findx.h"

#include <algorithm>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <time.h>

#include <derror.h>
#include "prometheus_msgs/GimbalTrackError.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <std_srvs/SetBool.h>

DERROR derrorX, derrorY;
cv::Mat image;
boost::shared_mutex mutexImageCallback_;
bool imageStatus_ = false;
boost::shared_mutex mutexImageStatus_;

using namespace caffe; // NOLINT(build/namespaces)
using namespace cv;

// todo 统一web_cam
#define PROCESS_WIDTH 640
#define PROCESS_HEIGHT 480

// 是否放弃对吊舱控制的标志为
bool is_control_gimal = false;

std::string model_file = "/root/Prometheus/Modules/object_detection/circlex/spire_caffe/examples/spire_x_classification/xnet_deploy.prototxt";
std::string trained_file = "/root/Prometheus/Modules/object_detection/circlex/spire_caffe/examples/spire_x_classification/xnet_iter_10000.caffemodel";
std::string mean_file = "";
std::string label_file = "/root/Prometheus/Modules/object_detection/circlex/spire_caffe/examples/spire_x_classification/label.txt";

spire::CNEllipseDetector cned;
// MarkerDetector MDetector;

//CameraParameters TheCameraParameters;

double _time_record;
void _tic()
{
  _time_record = (double)cv::getTickCount();
}
double _toc()
{
  double time_gap = ((double)cv::getTickCount() - _time_record) * 1000. / cv::getTickFrequency();
  return time_gap;
  // std::cout << "Cost Time: " << time_gap << " ms" << std::endl;
}

void cx_detect(Mat &srcIm, Mat3b &resultIm, vector<cv::Point> &pts, vector<float> &axis_bs);

void cx_init();
/*
void aruco_detect(Mat &srcIm, Mat3b &resultIm, vector<cv::Point> &pts);
void flow01(Mat &srcIm, Mat3b &resultIm, vector<cv::Point> &pts);
*/

float get_ros_time(ros::Time begin)
{
  ros::Time time_now = ros::Time::now();
  float currTimeSec = time_now.sec - begin.sec;
  float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
  return (currTimenSec + currTimeSec);
}

bool control_gimal_srv_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  is_control_gimal = req.data;
  std::cout << (is_control_gimal ? "circlx_gimal_control COLSE" : "circlx_gimal_control OPEN") << std::endl;
  return true;
}

void img_callback(const sensor_msgs::ImageConstPtr &msg)
{
  // std::cout << "xxxxxxxxxxxxxxxxxxa" << std::endl;
  cv_bridge::CvImagePtr cam_image;
  try
  {
    cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s ", e.what());
    return;
  }

  if (cam_image)
  {
    {
      boost::unique_lock<boost::shared_mutex> lockImageCalback(mutexImageCallback_);
      image = cam_image->image.clone();
    }
    {
      boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
      imageStatus_ = true;
    }
  }
  // std::cout << "xxxxx" << image.cols << " " << image.rows << std::endl;
}

bool getImageStatus()
{
  boost::shared_lock<boost::shared_mutex> lock(mutexImageStatus_);
  return imageStatus_;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fly_detector");
  ros::NodeHandle nh("~");

  if (nh.getParam("model_file", model_file))
  {
    ROS_INFO("model_file is %s", model_file.c_str());
  }
  else
  {
    ROS_WARN("didn't find parameter model_file");
  }

  if (nh.getParam("trained_file", trained_file))
  {
    ROS_INFO("trained_file is %s", trained_file.c_str());
  }
  else
  {
    ROS_WARN("didn't find parameter trained_file");
  }

  if (nh.getParam("label_file", label_file))
  {
    ROS_INFO("label_file is %s", label_file.c_str());
  }
  else
  {
    ROS_WARN("didn't find parameter label_file");
  }

  image_transport::ImageTransport it(nh);

  ros::Rate loopRate(40);
  ros::Publisher position_pub = nh.advertise<std_msgs::Float32MultiArray>("/prometheus/object_detection/circlex_det", 1);

  ros::Publisher diff_pub = nh.advertise<prometheus_msgs::GimbalTrackError>("/prometheus/object_detection/circelx_error", 1);

  ros::ServiceServer server = nh.advertiseService("/prometheus/circlex_gimbal_control", control_gimal_srv_callback);
  image_transport::Subscriber img_sub = it.subscribe("/prometheus/camera/rgb/image_raw", 1, img_callback);
  image_transport::Publisher result_vision_pub = it.advertise("/prometheus/camera/circlex", 1);

  // cv::VideoCapture cap(0);
  // cap.set(cv::CAP_PROP_FRAME_WIDTH, PROCESS_WIDTH);
  // cap.set(cv::CAP_PROP_FRAME_HEIGHT, PROCESS_HEIGHT);
  cx_init();

  // DASDetect flow50;
  // flow50.init();

  Mat3b resultIm;
  float cur_time;
  float last_time = 0.0;
  float dt;
  ros::Time begin_time = ros::Time::now();

  int CenterX = PROCESS_WIDTH / 2, CenterY = PROCESS_HEIGHT / 2;
  double totTime(0);
  int countsDt(0), detectionFps(0);
  bool drawReferenceCenter = true;
  vector<cv::Point> pointsDt;
  vector<float> axisbDt;

  cv::Point xp;
  // Rect _rect;
  // Point position;

  // cap >> image;
  while (ros::ok())
  {
    // NOTE 等待从相机获取图像
    while (!getImageStatus())
    {
      std::cout << "Waiting for image...." << std::endl;
      ros::spinOnce();
      loopRate.sleep();
      cv::waitKey(1000);
    }
    cur_time = get_ros_time(begin_time);
    dt = cur_time - last_time;
    if (dt > 1.0 || dt < 0.0)
    {
      dt = 0.05;
    }

    _tic();
    // cap >> image;
    pointsDt.clear();
    axisbDt.clear();
    cx_detect(image, resultIm, pointsDt, axisbDt);

    if (pointsDt.size() == 0 && findX(image, xp))
    {
      pointsDt.push_back(xp);
      image.copyTo(resultIm);
    }
    for (cv::Point xp : pointsDt)
    {
      circle(resultIm, xp, 4, Scalar(0, 0, 255), 2);
    }

    prometheus_msgs::GimbalTrackError error_pixels;
    sensor_msgs::ImagePtr tmp_sender;
    std_msgs::Float32MultiArray msg;

    if (pointsDt.size() > 0)
    {
      msg.data.push_back(1);
      msg.data.push_back(pointsDt[0].x);
      msg.data.push_back(pointsDt[0].y);
      msg.data.push_back(PROCESS_WIDTH);
      msg.data.push_back(PROCESS_HEIGHT);

      error_pixels.detected = true;
      error_pixels.x = pointsDt[0].x - PROCESS_WIDTH / 2;
      error_pixels.y = pointsDt[0].y - PROCESS_HEIGHT / 2;

      // NOTE PID误差微分
      derrorX.add_error(error_pixels.x, cur_time);
      derrorY.add_error(error_pixels.y, cur_time);
      derrorY.derror_output();
      derrorX.derror_output();

      error_pixels.velx = derrorX.Output;
      error_pixels.vely = derrorY.Output;

      error_pixels.Ix += error_pixels.x * dt;
      error_pixels.Iy += error_pixels.y * dt;

      countsDt++;
    }
    else
    {
      msg.data.push_back(0);
      msg.data.push_back(0);
      msg.data.push_back(0);
      msg.data.push_back(0);
      msg.data.push_back(0);
      error_pixels.detected = false;
    }
    position_pub.publish(msg);
    if (is_control_gimal)
    {
      error_pixels.detected = false;
    }
    diff_pub.publish(error_pixels);
    ros::spinOnce();

    // cout<<4<<endl;

    // draw reference center
    if (drawReferenceCenter)
    {
      cv::line(resultIm, cv::Point(CenterX - 40, CenterY), cv::Point(CenterX + 40, CenterY), cv::Scalar(0, 0, 255));
      cv::line(resultIm, cv::Point(CenterX, CenterY - 40), cv::Point(CenterX, CenterY + 40), cv::Scalar(0, 0, 255));
      circle(resultIm, cv::Point(CenterX, CenterY), 40, cv::Scalar(0, 0, 255), 2);
    }

    if (drawReferenceCenter)
    {
      char dfps[128], buf[128];
      // Get time now for realtime video
      tm *local;
      time_t t = time(NULL);
      // NOTE 时间, format字符串
      local = localtime(&t);
      strftime(buf, 64, "%Y-%m-%d %H:%M:%S", local);
      sprintf(dfps, "  DFPS:%d", detectionFps);
      cv::putText(resultIm, std::string(buf) + std::string(dfps), cv::Point(PROCESS_WIDTH * .05, PROCESS_HEIGHT * .05),
                  2, 1.f, cv::Scalar(255, 255, 255));
    }
    // todo bgr8
    tmp_sender = cv_bridge::CvImage(std_msgs::Header(), "rgb8", resultIm).toImageMsg();
    result_vision_pub.publish(tmp_sender);

    // cv::imshow("cam", resultIm);
    // waitKey(5);
    // if (waitKey(1) == 27)
    //   break;
    totTime += _toc();
    if (totTime >= 1000.0)
    {
      detectionFps = countsDt;
      std::cout << "DETECTION FPS:" << countsDt << std::endl;
      countsDt = 0;
      totTime = .0;
    }
    last_time = cur_time;
    loopRate.sleep();
  }

  return 0;
}
void cx_detect(Mat &srcIm, Mat3b &resultIm, vector<cv::Point> &pts, vector<float> &axis_bs)
{
  Mat1b gray;
  vector<spire::Ellipse> ellsCned;
  cvtColor(srcIm, gray, COLOR_BGR2GRAY);
  cned.Detect(gray, ellsCned);

  resultIm = srcIm.clone();
  cned.DrawDetectedEllipses(resultIm, ellsCned);

  if (ellsCned.size() > 0)
  {
    Point pt;
    pt.x = ellsCned[0]._xc;
    pt.y = ellsCned[0]._yc;
    pts.push_back(pt);
    axis_bs.push_back(ellsCned[0]._b);
  }
}

void cx_init()
{
  Size sz(PROCESS_WIDTH, PROCESS_HEIGHT);
  // Parameters Settings
  int iThrLength = 14;
  float fThrObb = 3.0f; // Discarded..
  float fThrPos = 1.0f;

  float fThrMinScore = 0.3f;
  float fMinReliability = 0.7f;
  int iNs = 16;
  float fMaxCenterDistance = sqrt(float(sz.width * sz.width + sz.height * sz.height)) * 0.05f;
  Size szPreProcessingGaussKernelSize = Size(5, 5);
  double dPreProcessingGaussSigma = 1.0;
  float fDistanceToEllipseContour = 0.1f;
  cned.SetParameters(szPreProcessingGaussKernelSize,
                     dPreProcessingGaussSigma,
                     fThrPos,
                     fMaxCenterDistance,
                     iThrLength,
                     fThrObb,
                     fDistanceToEllipseContour,
                     fThrMinScore,
                     fMinReliability,
                     iNs);
#ifdef DETECT_MARKER_X_NET
  cned.cvmat_classifier_.Init(model_file, trained_file, mean_file, label_file);
#endif
}
