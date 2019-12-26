//
// Created by fei on 19-1-22.
// modified by ming on 19-3-23
//

#ifndef STEREO_MATCH_H
#define STEREO_MATCH_H

#include "elas.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>


class StereoMatch {
public:
    StereoMatch();
    StereoMatch(Elas::parameters &param);

    cv::Mat run(const cv::Mat &left, const cv::Mat &right, const float baseline, const float fx);
    cv::Mat convertDisparityToDepth(const cv::Mat &disp, const float baseline, const float fx);

public:
    Elas::parameters mParam;
    double fx = 711.9;
    double fy = 711.7;
    double cx = 644.6;
    double cy = 361.1;
};

#endif //STEREO_MATCH_H
