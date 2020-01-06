#ifndef STEREO_POINTCLOUD_H
#define STEREO_POINTCLOUD_H

#include <opencv2/core/mat.hpp>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class StereoPointCloud {
public:
    StereoPointCloud();
    void addNewPoints_disparity(const cv::Mat &disparity, const Eigen::Isometry3d &Trans);
    void addNewPoints_depth(const cv::Mat &depth, const Eigen::Isometry3d &Trans);
    PointCloud* addNewPoints_local(const cv::Mat &disparity);
public: 
    double baseline = 0.12;
    double fx = 711.9;
    double fy = 711.7;
    double cx = 644.6;
    double cy = 361.1;
    PointCloud* pointCloud = new PointCloud;
};
#endif //STEREO_POINTCLOUD_H