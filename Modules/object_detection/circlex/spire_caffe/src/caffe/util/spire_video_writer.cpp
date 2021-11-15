#ifdef USE_OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/thread.hpp>
#include <stdint.h>
#include <algorithm>
#include <csignal>
#include <map>
#include <string>
#include <vector>
#include <time.h>

#include "caffe/util/spire_video_writer.hpp"

namespace caffe {

SpireVideoWriter::SpireVideoWriter()
    : ready_to_write_() {
}

SpireVideoWriter::~SpireVideoWriter() {
  Release();
}

void SpireVideoWriter::Release() {
  this->StopInternalThread();
  if (writer_.isOpened()) {
    writer_.release();
  }
}

void SpireVideoWriter::SetUp(const std::string& file_path, double fps, const cv::Size& size) {
  fps_ = fps;
  image_size_ = size;
  // get now time
  time_t t;
  tm* local;
  char buf[128];

  t = time(NULL);
  local = localtime(&t);
  strftime(buf, 64, "/FlyVideo_%Y-%m-%d_%H-%M-%S.avi", local);

  if (!writer_.open(file_path+string(buf), CV_FOURCC('M','J','P','G'), fps_, image_size_)) {
    LOG(FATAL) << "Failed to write video: " << file_path;
  }
  StartInternalThread();
}

int SpireVideoWriter::WaitDependsFps() {
  return cv::waitKey(1000/fps_);
}

void SpireVideoWriter::PutOneFrame(const cv::Mat& image) {
  cv::Mat* image_put = new cv::Mat();
  cv::resize(image, *image_put, image_size_);
  ready_to_write_.push(image_put);
}

void SpireVideoWriter::InternalThreadEntry() {
  // Jario Edit Start
  try {
    while (!must_stop()) {
      cv::Mat* image_current = ready_to_write_.pop();
      writer_ << *image_current;
      delete image_current;
    }
  } catch (boost::thread_interrupted&) {
    // Interrupted exception is expected on shutdown
    if (writer_.isOpened()) {
      writer_.release();
    }
  }
  // Jario Edit End
}

}  // namespace caffe
#endif  // USE_OPENCV
