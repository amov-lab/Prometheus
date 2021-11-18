#ifndef SPIRE_CAMERA_DATA_LAYERS_HPP_
#define SPIRE_CAMERA_DATA_LAYERS_HPP_

#ifdef USE_OPENCV
#if OPENCV_VERSION == 3
#include <opencv2/videoio.hpp>
#else
#include <opencv2/opencv.hpp>
#endif  // OPENCV_VERSION == 3

#include <string>
#include <vector>
#include <boost/thread.hpp>

#include "caffe/blob.hpp"
#include "caffe/data_transformer.hpp"
#include "caffe/internal_thread.hpp"
#include "caffe/layer.hpp"
#include "caffe/layers/base_data_layer.hpp"
#include "caffe/proto/caffe.pb.h"
#include "caffe/util/db.hpp"

namespace caffe {

template <typename Dtype>
class SpireCameraDataLayer :
    public BaseDataLayer<Dtype>, public InternalThread {
 public:
  explicit SpireCameraDataLayer(const LayerParameter& param);
  virtual ~SpireCameraDataLayer();
  // LayerSetUp: implements common data layer setup functionality, and calls
  // DataLayerSetUp to do special data layer setup for individual layer types.
  // This method may not be overridden.
  void LayerSetUp(const vector<Blob<Dtype>*>& bottom,
      const vector<Blob<Dtype>*>& top);

  virtual void Forward_cpu(const vector<Blob<Dtype>*>& bottom,
      const vector<Blob<Dtype>*>& top);
#ifndef CPU_ONLY
  virtual void Forward_gpu(const vector<Blob<Dtype>*>& bottom,
      const vector<Blob<Dtype>*>& top);
#endif

 protected:
  virtual void InternalThreadEntry();
  virtual void load_batch(Batch<Dtype>* batch);

  SpireCameraDataParameter_VideoType video_type_;
  cv::VideoCapture cap_;

  vector<int> top_shape_;
  bool access_one_;

  vector<shared_ptr<Batch<Dtype> > > prefetch_;
  BlockingQueue<Batch<Dtype>*> prefetch_one_test_;
  Batch<Dtype>* prefetch_new_;
  Batch<Dtype>* prefetch_current_;
  boost::mutex io_mutex_;

  Blob<Dtype> transformed_data_;
};

}  // namespace caffe

#endif  // USE_OPENCV

#endif  // SPIRE_CAMERA_DATA_LAYERS_HPP_

