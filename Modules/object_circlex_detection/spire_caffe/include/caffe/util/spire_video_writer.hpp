#ifndef SPIRE_VIDEO_WRITER_HPP_
#define SPIRE_VIDEO_WRITER_HPP_

#ifdef USE_OPENCV
#if OPENCV_VERSION == 3
#include <opencv2/videoio.hpp>
#else
#include <opencv2/opencv.hpp>
#endif  // OPENCV_VERSION == 3

#include <string>
#include <vector>

#include "caffe/internal_thread.hpp"
#include "caffe/util/blocking_queue.hpp"

namespace caffe {

class SpireVideoWriter : public InternalThread {
 public:
  explicit SpireVideoWriter();
  virtual ~SpireVideoWriter();

  void SetUp(const std::string& file_path, double fps, const cv::Size& size);
  void Release();
  void PutOneFrame(const cv::Mat& image);
  int WaitDependsFps();

 protected:
  virtual void InternalThreadEntry();

  cv::VideoWriter writer_;
  double fps_;
  cv::Size image_size_;
  BlockingQueue<cv::Mat*> ready_to_write_;
};

}  // namespace caffe

#endif  // USE_OPENCV

#endif  // SPIRE_VIDEO_WRITER_HPP_

