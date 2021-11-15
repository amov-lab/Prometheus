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

#include "caffe/blob.hpp"
#include "caffe/data_transformer.hpp"
#include "caffe/internal_thread.hpp"
#include "caffe/layer.hpp"
#include "caffe/layers/spire_camera_data_layer.hpp"
#include "caffe/proto/caffe.pb.h"
#include "caffe/util/blocking_queue.hpp"

#define EXCHANGE_SIZE 2

namespace caffe {

template <typename Dtype>
SpireCameraDataLayer<Dtype>::SpireCameraDataLayer(
    const LayerParameter& param)
    : BaseDataLayer<Dtype>(param), prefetch_(EXCHANGE_SIZE), prefetch_one_test_() {
  for (int i = 0; i < EXCHANGE_SIZE; ++i) {
    prefetch_[i].reset(new Batch<Dtype>());
  }
  access_one_ = true;
}

template <typename Dtype>
SpireCameraDataLayer<Dtype>::~SpireCameraDataLayer() {
  this->StopInternalThread();
  if (cap_.isOpened()) {
    cap_.release();
  }
}

template <typename Dtype>
void SpireCameraDataLayer<Dtype>::LayerSetUp(
    const vector<Blob<Dtype>*>& bottom, const vector<Blob<Dtype>*>& top) {
  BaseDataLayer<Dtype>::LayerSetUp(bottom, top);

  const int batch_size = this->layer_param_.data_param().batch_size();
  CHECK(batch_size == 1) << "Batch size must equal one!";

  const SpireCameraDataParameter& camera_data_param =
          this->layer_param_.spire_camera_data_param();
  video_type_ = camera_data_param.video_type();

  // Read an image, and use it to initialize the top blob.
  cv::Mat cv_img;
  if (video_type_ == SpireCameraDataParameter_VideoType_WEBCAM) {
    const int device_id = camera_data_param.device_id();
    if (!cap_.open(device_id)) {
      LOG(FATAL) << "Failed to open webcam: " << device_id;
    }
    cap_ >> cv_img;
  } else {
    LOG(FATAL) << "Unknow video type!";
  }
  // Use data_transformer to infer the expected blob shape from a cv_image.
  top_shape_ = this->data_transformer_->InferBlobShape(cv_img);
  this->transformed_data_.Reshape(top_shape_);
  top_shape_[0] = batch_size;
  top[0]->Reshape(top_shape_);

  LOG(INFO) << "output data size: " << top[0]->num() << ","
      << top[0]->channels() << "," << top[0]->height() << ","
      << top[0]->width();
  // label
  if (this->output_labels_) {
    vector<int> label_shape(1, batch_size);
    top[1]->Reshape(label_shape);
  }

  // Before starting the prefetch thread, we make cpu_data and gpu_data
  // calls so that the prefetch thread does not accidentally make simultaneous
  // cudaMalloc calls when the main thread is running. In some GPUs this
  // seems to cause failures if we do not so.
  for (int i = 0; i < EXCHANGE_SIZE; ++i) {
      prefetch_[i]->data_.mutable_cpu_data();
      if (this->output_labels_) {
        prefetch_[i]->label_.mutable_cpu_data();
      }
    }
  #ifndef CPU_ONLY
    if (Caffe::mode() == Caffe::GPU) {
      for (int i = 0; i < EXCHANGE_SIZE; ++i) {
        prefetch_[i]->data_.mutable_gpu_data();
        if (this->output_labels_) {
          prefetch_[i]->label_.mutable_gpu_data();
        }
      }
    }
  #endif

  DLOG(INFO) << "Initializing prefetch";
  this->data_transformer_->InitRand();
  StartInternalThread();
  DLOG(INFO) << "Prefetch initialized.";
}

template <typename Dtype>
void SpireCameraDataLayer<Dtype>::InternalThreadEntry() {
#ifndef CPU_ONLY
  cudaStream_t stream;
  if (Caffe::mode() == Caffe::GPU) {
    CUDA_CHECK(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking));
  }
#endif

  Batch<Dtype> current_batch;
  current_batch.data_.mutable_cpu_data();
  try {
    while (!must_stop()) {
      load_batch(&current_batch);
      while (prefetch_one_test_.size() > 0) {
        Batch<Dtype>* discard_batch;
        prefetch_one_test_.try_pop(&discard_batch);
      }
      {
        boost::mutex::scoped_lock lock(io_mutex_);
        if (access_one_) {
          prefetch_new_ = prefetch_[0].get();
        } else {
          prefetch_new_ = prefetch_[1].get();
        }
        prefetch_new_->data_.ReshapeLike(current_batch.data_);
        prefetch_new_->data_.set_cpu_data(current_batch.data_.mutable_cpu_data());
#ifndef CPU_ONLY
        if (Caffe::mode() == Caffe::GPU) {
          prefetch_new_->data_.data().get()->async_gpu_push(stream);
          CUDA_CHECK(cudaStreamSynchronize(stream));
        }
#endif
        prefetch_one_test_.push(prefetch_new_);
      }
    }
  } catch (boost::thread_interrupted&) {
    // Interrupted exception is expected on shutdown
  }
#ifndef CPU_ONLY
  if (Caffe::mode() == Caffe::GPU) {
    CUDA_CHECK(cudaStreamDestroy(stream));
  }
#endif
}

// This function is called on prefetch thread
template<typename Dtype>
void SpireCameraDataLayer<Dtype>::load_batch(Batch<Dtype>* batch) {
  CHECK(batch->data_.count());
  CHECK(this->transformed_data_.count());

  // Reshape according to the first anno_datum of each batch
  // on single input batches allows for inputs of varying dimension.
  // const int batch_size = this->layer_param_.data_param().batch_size();
  top_shape_[0] = 1;
  this->transformed_data_.Reshape(top_shape_);
  // Reshape batch according to the batch_size.
  batch->data_.Reshape(top_shape_);

  Dtype* top_data = batch->data_.mutable_cpu_data();
  // Dtype* top_label = NULL;  // suppress warnings about uninitialized variables

  cv::Mat cv_img;
  cap_ >> cv_img;
  CHECK(cv_img.data) << "Could not load image!";

  // Apply transformations (mirror, crop...) to the image
  this->transformed_data_.set_cpu_data(top_data);
  this->data_transformer_->Transform(cv_img, &(this->transformed_data_));

}

template <typename Dtype>
void SpireCameraDataLayer<Dtype>::Forward_cpu(
    const vector<Blob<Dtype>*>& bottom, const vector<Blob<Dtype>*>& top) {

  prefetch_current_ = prefetch_one_test_.pop("Waiting for data");
  {
    boost::mutex::scoped_lock lock(io_mutex_);
    access_one_ = !access_one_;
    // Reshape to loaded data.
    top[0]->ReshapeLike(prefetch_current_->data_);
    top[0]->set_cpu_data(prefetch_current_->data_.mutable_cpu_data());
    if (this->output_labels_) {
      // Reshape to loaded labels.
      top[1]->ReshapeLike(prefetch_current_->label_);
      top[1]->set_cpu_data(prefetch_current_->label_.mutable_cpu_data());
    }
  }
}

#ifdef CPU_ONLY
STUB_GPU_FORWARD(BasePrefetchingDataLayer, Forward);
#endif

INSTANTIATE_CLASS(SpireCameraDataLayer);
REGISTER_LAYER_CLASS(SpireCameraData);

}  // namespace caffe
#endif  // USE_OPENCV
