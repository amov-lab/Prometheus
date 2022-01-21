//
// Created by fei on 19-1-22.
// modified by ming on 19-3-23
//

#include "stereomatch.h"


StereoMatch::StereoMatch() {
    mParam.postprocess_only_left = true;
}

StereoMatch::StereoMatch(Elas::parameters &param) {
    mParam = param;
}


cv::Mat StereoMatch::run(const cv::Mat &left, const cv::Mat &right, const float baseline, const float fx) {
    if (left.empty() || right.empty()) {
        std::cout << "Image is empty" << std::endl;
        exit(0);
    }
    if (left.channels() > 1 || right.channels() > 1) {
        std::cout << "Images must be gray image." << std::endl;
        exit(0);
    }

    Elas elas(mParam);

    int width = left.cols;
    int height = left.rows;
    const int32_t dims[3] = {width, height, width}; // bytes per line = width

    cv::Mat disparity1(height, width, CV_32F);
    cv::Mat disparity2(height, width, CV_32F);
    cv::Mat disparityMap, depthMap;

    bool valid_depth = elas.process(left.data, right.data, (float *) disparity1.data, (float *) disparity2.data, dims);
    imwrite("disparity.pgm",disparity1);

    if (valid_depth) 
    depthMap = convertDisparityToDepth(disparity1, baseline, fx);
    return disparity1; // return empty depth image if not valid
}

cv::Mat StereoMatch::convertDisparityToDepth(const cv::Mat &disparityMap, const float baseline, const float fx) {
    cv::Mat depthMap = cv::Mat(disparityMap.size(), CV_16U);
    for (int i = 0; i < disparityMap.rows; i++) {
        for (int j = 0; j < disparityMap.cols; j++) {
            double d = static_cast<double>(disparityMap.at<float>(i, j));
            depthMap.at<unsigned short>(i, j) = (baseline * fx) / d;
            // depthMap.at<unsigned short>(i, j) = d;
            // if (d < 10)
            //     depthMap.at<unsigned short>(i, j) = -1;
            // if (d>0) std::cout << "The disparity is: "<< (baseline * fx) / d<< std::endl;
            short disparity_ij = disparityMap.at<unsigned short>(i, j);
            if(disparity_ij <= 1)
                depthMap.at<unsigned short>(i, j) = 0;

            if (std::isnan(depthMap.at<unsigned short>(i, j)) || std::isinf(depthMap.at<unsigned short>(i, j)))
                depthMap.at<unsigned short>(i, j) = 0;
        }
    }

    return depthMap;
}