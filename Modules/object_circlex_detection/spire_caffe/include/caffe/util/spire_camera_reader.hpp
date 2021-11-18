#ifndef SPIRE_CAMERA_READER_HPP_
#define SPIRE_CAMERA_READER_HPP_

#ifdef USE_OPENCV
#if OPENCV_VERSION == 3
#include <opencv2/videoio.hpp>
#else
#include <opencv2/opencv.hpp>
#endif  // OPENCV_VERSION == 3

#include <string>
#include <vector>
#include <boost/thread.hpp>

#include "caffe/internal_thread.hpp"
#include "caffe/util/blocking_queue.hpp"

// #define USE_FLYCAPTURE_CAMERA

#ifdef USE_FLYCAPTURE_CAMERA
#include <flycapture/FlyCapture2.h>
#endif
namespace caffe {

#ifdef USE_FLYCAPTURE_CAMERA
class SpireFlyCapture {
 public:
  FlyCapture2::Error error;
  FlyCapture2::Camera camera;
  FlyCapture2::CameraInfo camInfo;
  bool SetUp() {
    // Connect the camera
    error = camera.Connect( 0 );
    if ( error != FlyCapture2::PGRERROR_OK ) {
      std::cout << "Failed to connect to camera" << std::endl;
      return false;
    }
    // Get the camera info and print it out
    error = camera.GetCameraInfo( &camInfo );
    if ( error != FlyCapture2::PGRERROR_OK ) {
      std::cout << "Failed to get camera info from camera" << std::endl;
      return false;
    }
    std::cout << camInfo.vendorName << " "
              << camInfo.modelName << " "
              << camInfo.serialNumber << std::endl;
    error = camera.StartCapture();
    if ( error == FlyCapture2::PGRERROR_ISOCH_BANDWIDTH_EXCEEDED ) {
      std::cout << "Bandwidth exceeded" << std::endl;
      return false;
    } else if ( error != FlyCapture2::PGRERROR_OK ) {
      std::cout << "Failed to start image capture" << std::endl;
      return false;
    }
    return true;
  }
  void ReadNextFrame(cv::Mat& frame) {
    // Get the image
    FlyCapture2::Image rawImage;
    FlyCapture2::Error error = camera.RetrieveBuffer( &rawImage );
    if ( error != FlyCapture2::PGRERROR_OK ) {
      std::cout << "capture error" << std::endl;
      return;
    }
    // convert to rgb
    FlyCapture2::Image rgbImage;
    rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );
    // convert to OpenCV Mat
    unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();
    cv::Mat cvImage = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);
    cvImage.copyTo(frame);
  }
};
#endif

class SpireCameraReader : public InternalThread {
 public:
  explicit SpireCameraReader();
  virtual ~SpireCameraReader();

  void SetUp(int device_id = 0);
  void GetOneFrame(cv::Mat& image);

 protected:
  virtual void InternalThreadEntry();
  virtual void load_batch(cv::Mat& batch);

  cv::VideoCapture cap_;
  int device_id_;
  bool access_one;

  vector<shared_ptr<cv::Mat > > prefetch_;
  BlockingQueue<cv::Mat*> prefetch_one_test_;
  cv::Mat* prefetch_current_;
  boost::mutex io_mutex_;

#ifdef USE_FLYCAPTURE_CAMERA
  SpireFlyCapture fly_cap_;
#endif
};

}  // namespace caffe



#endif  // USE_OPENCV

#endif  // SPIRE_CAMERA_READER_HPP_

