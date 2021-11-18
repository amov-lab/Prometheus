#include <vector>

#include "caffe/layers/spire_camera_data_layer.hpp"

namespace caffe {

template <typename Dtype>
void SpireCameraDataLayer<Dtype>::Forward_gpu(
    const vector<Blob<Dtype>*>& bottom, const vector<Blob<Dtype>*>& top) {

  prefetch_current_ = prefetch_one_test_.pop("Waiting for data");
  {
    boost::mutex::scoped_lock lock(io_mutex_);
    access_one_ = !access_one_;
    // Reshape to loaded data.
    top[0]->ReshapeLike(prefetch_current_->data_);
    top[0]->set_gpu_data(prefetch_current_->data_.mutable_gpu_data());
    if (this->output_labels_) {
      // Reshape to loaded labels.
      top[1]->ReshapeLike(prefetch_current_->label_);
      top[1]->set_gpu_data(prefetch_current_->label_.mutable_gpu_data());
    }
  }
}

INSTANTIATE_LAYER_GPU_FORWARD(SpireCameraDataLayer);

}  // namespace caffe
