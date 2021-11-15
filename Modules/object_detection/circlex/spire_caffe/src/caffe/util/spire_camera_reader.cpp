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

#include "caffe/util/spire_camera_reader.hpp"

namespace caffe {

SpireCameraReader::SpireCameraReader()
    : prefetch_one_test_() {
#define EXCHANGE_SIZE 2
  prefetch_.resize(EXCHANGE_SIZE);
  for (int i = 0; i < EXCHANGE_SIZE; ++i) {
    prefetch_[i].reset(new cv::Mat());
  }
  access_one = true;
}

SpireCameraReader::~SpireCameraReader() {
  this->StopInternalThread();
  if (cap_.isOpened()) {
    cap_.release();
  }
}

void SpireCameraReader::SetUp(int device_id) {
// #ifdef USE_FLYCAPTURE_CAMERA
//   fly_cap_.SetUp();
// #else
  std::cout<<"..."<<device_id<<std::endl;
  if (!cap_.open(device_id)) {
    std::cout<<"..."<<device_id<<std::endl;
    LOG(FATAL) << "Failed to open webcam: " << device_id;
  }
  std::cout<<"..."<<1<<std::endl;
// #endif
  StartInternalThread();
}

void SpireCameraReader::GetOneFrame(cv::Mat& image) {
  cv::Mat* im1 = prefetch_one_test_.pop("Waiting for data");
  {
    boost::mutex::scoped_lock lock(io_mutex_);
    access_one = !access_one;
    im1->copyTo(image);
  }
}

double _time_record;
void _tic() {
  _time_record = (double)cv::getTickCount();
}
void _toc() {
  double time_gap = ((double)cv::getTickCount()-_time_record)*1000. / cv::getTickFrequency();
  std::cout << "Cost Time: " << time_gap << " ms" << std::endl;
}
void SpireCameraReader::InternalThreadEntry() {
  // Jario Edit Start
  cv::Mat current_mat;
  try {
    while (!must_stop()) {
      // _tic();
      load_batch(current_mat);
      // _toc();
      while (prefetch_one_test_.size() > 0) {
        cv::Mat* discard_batch;
        prefetch_one_test_.try_pop(&discard_batch);
      }
      {
        boost::mutex::scoped_lock lock(io_mutex_);
        if (access_one) {
          prefetch_current_ = prefetch_[0].get();
          current_mat.copyTo(*prefetch_current_);
        } else {
          prefetch_current_ = prefetch_[1].get();
          current_mat.copyTo(*prefetch_current_);
        }
        prefetch_one_test_.push(prefetch_current_);
      }
    }
  } catch (boost::thread_interrupted&) {
    // Interrupted exception is expected on shutdown
  }
  // Jario Edit End
}

void SpireCameraReader::load_batch(cv::Mat& batch) {

  // int batch_size = 1;
  // for (int item_id = 0; item_id < batch_size; ++item_id) {
    // cv::Mat cv_img;
#ifdef USE_FLYCAPTURE_CAMERA
    fly_cap_.ReadNextFrame(batch);
#else
    cap_ >> batch;
#endif

    CHECK(batch.data) << "Could not load image!";
    // Apply transformations (mirror, crop...) to the image
  // }
}

}  // namespace caffe
#endif  // USE_OPENCV
