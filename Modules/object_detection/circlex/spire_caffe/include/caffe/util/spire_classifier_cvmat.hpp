#ifndef SPIRE_CLASSIFIER_CVMAT_HPP_
#define SPIRE_CLASSIFIER_CVMAT_HPP_

#ifdef USE_OPENCV
#if OPENCV_VERSION == 3
#include <opencv2/videoio.hpp>
#else
#include <opencv2/opencv.hpp>
#endif  // OPENCV_VERSION == 3

#include <string>
#include <vector>

#include "caffe/caffe.hpp"

namespace caffe {

/* Pair (label, confidence) representing a prediction. */
typedef std::pair<string, float> PredictionStr;
/* Pair (label, confidence) representing a prediction. */
typedef std::pair<int, float> PredictionInt;

class SpireCvMatClassifier {
 public:
  explicit SpireCvMatClassifier();
  virtual ~SpireCvMatClassifier();

  // fisrt you need use this function to initialize
  void Init(const string& model_file,
            const string& trained_file,
            const string& mean_file,
            const string& label_file);
  // classify a cv::Mat and get label_string and confidient
  std::vector<PredictionStr> Classify_LS(cv::Mat cvmat, int N = 1);
  // classify a cv::Mat and get label_int and confidient
  std::vector<PredictionInt> Classify_LI(cv::Mat cvmat, int N = 1);

 protected:
  void SetMean(const string& mean_file);
  std::vector<float> Predict(const cv::Mat& img);
  void WrapInputLayer(std::vector<cv::Mat>* input_channels);
  void Preprocess(const cv::Mat& img,
                    std::vector<cv::Mat>* input_channels);

  shared_ptr<Net<float> > net_;
  cv::Size input_geometry_;
  int num_channels_;
  cv::Mat mean_;
  std::vector<string> labels_;

};

}  // namespace caffe

#endif  // USE_OPENCV

#endif  // SPIRE_CLASSIFIER_CVMAT_HPP_

