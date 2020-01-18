// Created by Colin Lee in 06/12/2019
#include "pointcloud.h"

StereoPointCloud::StereoPointCloud() {}
void StereoPointCloud::addNewPoints_disparity(const cv::Mat &disparity, const Eigen::Isometry3d &Trans)
{
    int width = disparity.cols;
    int height = disparity.rows;
    for ( int v=0; v<height; v+=3 )
        for ( int u=0; u<width; u+=3)
        {
            double disp = static_cast<double>(disparity.at<float>(v,u));
            double d;
            if (disp>10) d = (baseline * fx) / disp;
            else continue;            
            Eigen::Vector3d point;
            point[2] = d;
            point[1] = (v-cy)*point[2]/fy;
            point[0] = (u-cx)*point[2]/fx;
            Eigen::Matrix3d rotation_matrix;
            Eigen::AngleAxisd rotation_vector(M_PI, Eigen::Vector3d(0,0,0));
            rotation_matrix << 0.9999952601011439, 0.0026195716859150693, -0.0016179058765224553,
            0.0026230754143729816, -0.9999942111635078, 0.002167282824549874,
            -0.0016122191580073178, -0.002171516440975972, -0.9999963426261771;
            Eigen::Vector3d translation(0.0532789112208159,0.008376288183237763,-0.028686495700069835);
            Eigen::Isometry3d T_B_C = Eigen::Isometry3d::Identity();
            T_B_C.rotate(rotation_matrix);
            T_B_C.pretranslate(translation);
            // point = rotation_vector.matrix()*point;
            Eigen::Vector3d pointWorld = Trans*point;
            
            PointT p;
            p.x = pointWorld[0];
            p.y = pointWorld[1];
            p.z = pointWorld[2];
            int B = (int)((p.z/3.5)*255);
            p.r = B;
            p.g = B;
            p.b = B;
            pointCloud->points.push_back(p);
            std::cout<< "the color index is: "<< B << std::endl; 
            std::cout<< "the height is: "<< p.z << std::endl; 
            // printf("x=%f,y=%f,z=%f\n",point[0],point[1],point[2]);
        }
    pointCloud->is_dense = false;
    pcl::io::savePCDFileBinary("map.pcd",*pointCloud);
}
PointCloud* StereoPointCloud::addNewPoints_local(const cv::Mat &disparity)
{
    PointCloud* cloud_ptr = new PointCloud;
    int width = disparity.cols;
    int height = disparity.rows;
    for ( int v=0; v<height; v+=3 )
        for ( int u=0; u<width; u+=3)
        {
            PointT p;
            double disp = static_cast<double>(disparity.at<float>(v,u));
            double d = 0;
            if (disp>10) d = (baseline * fx) / disp;
            else continue;
            p.z = d;
            p.x = (u-cx)*p.z/fx;
            p.y = (v-cy)*p.z/fy;
            printf("x=%f,y=%f,z=%f\n",p.x,p.y,p.z);
            cloud_ptr->points.push_back(p);
        }
    return cloud_ptr;

    
}
