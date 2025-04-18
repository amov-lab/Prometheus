#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>		
// 配置参数
constexpr float MIN_DEPTH = 0.25;    // 最小有效深度 (米)
constexpr float MAX_DEPTH = 5.0;     // 最大有效深度 (米)
constexpr int NEIGHBOR_SIZE = 5;     // 邻域窗口尺寸 (奇数)
constexpr float DEPTH_DIFF_THRESH = 0.2; // 深度差异阈值 (米)
std::mutex mtx; // 全局互斥锁
class Depth_Pointcloud {
private:
    ros::NodeHandle nh_;
    ros::Publisher filtered_pub_,pointcloud_pub_;
    ros::Subscriber depth_sub_,camerainfo_sub_;
    // 深度图过滤结果
    sensor_msgs::ImagePtr result_msg;
    // 裁剪参数
    const cv::Rect crop_roi_{80, 40, 480, 320};
    // 相机内参
    float fx_,fy_,cx_,cy_,fx_inv,fy_inv;
    // 相机到机体系的外参矩阵
    Eigen::Matrix3f R_,R_center ;
    Eigen::Vector3f T_;
    // Time
    ros::Time now_times,last_times;
    // 滤波器参数
    double fZpass_max,fZpass_min,fYpass,fXpass_max,fXpass_min,f_voxel,f_outlier_thresh;
    int uav_id,f_outlier_meank;
public:
    Depth_Pointcloud():R_(Eigen::Matrix3f::Identity()){
        // 加载参数
        ros::NodeHandle public_nh("~");
        public_nh.param("uav_id", uav_id, 1);
        // 直通滤波器参数
        public_nh.param("fZpass_max_", fZpass_max, 1.5);
        public_nh.param("fZpass_min_", fZpass_min, -0.8);
        public_nh.param("fYpass_", fYpass, 1.8);
        public_nh.param("fXpass_max_", fXpass_max, 5.0);
        public_nh.param("fXpass_min_", fXpass_min, 0.0);
        // 体素过滤参数
        public_nh.param("f_voxel_", f_voxel, 0.04);
        // 离群点过滤参数
        public_nh.param("f_outlier_meanK", f_outlier_meank, 50);
        public_nh.param("f_outlier_thresh", f_outlier_thresh, 1.2);
        filtered_pub_ = nh_.advertise<sensor_msgs::Image>("/SU17CAM/filtered_depthF", 1);
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/uav" + std::to_string(uav_id) + "/camera/depth/points", 1);
        camerainfo_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>("/SU17CAM/camInfoF", 1,&Depth_Pointcloud::caminfoCallback, this);
        depth_sub_ = nh_.subscribe<sensor_msgs::Image>("/uav" + std::to_string(uav_id) + "/restored_depth", 1, &Depth_Pointcloud::depthCallback, this);
        // 旋转平移矩阵
        R_ << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
        R_center << 0.9945, 0, 0.1045, 0, 1, 0, -0.1045, 0, 0.9945;
        T_ << 0.15,-0.05,0.04;
    }

    void caminfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
        fx_ = msg->K[0];
        fy_ = msg->K[4];
        cx_ = msg->K[2];
        cy_ = msg->K[5];
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr points_fliter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& raw_cloud){
            // 直通滤波 Z - Y - X
        pcl::PointCloud<pcl::PointXYZ>::Ptr intermediate_cloud_z(
                                new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_through_z,pass_through_y,pass_through_x;	
            // Z 
        pass_through_z.setInputCloud(raw_cloud);				
        pass_through_z.setFilterFieldName("z");		
        pass_through_z.setFilterLimits(fZpass_min,fZpass_max);
        pass_through_z.filter(*intermediate_cloud_z);	
        ROS_INFO("intermediate_cloud_z size = %ld",intermediate_cloud_z->points.size());
            // Y
        pcl::PointCloud<pcl::PointXYZ>::Ptr intermediate_cloud_y(
                                new pcl::PointCloud<pcl::PointXYZ>);
        pass_through_y.setInputCloud(intermediate_cloud_z);				
        pass_through_y.setFilterFieldName("y");			
        pass_through_y.setFilterLimits(-fYpass,fYpass);			
        pass_through_y.filter(*intermediate_cloud_y);
        ROS_INFO("intermediate_cloud_y size = %ld",intermediate_cloud_y->points.size());
            // X
        pcl::PointCloud<pcl::PointXYZ>::Ptr intermediate_cloud_x(
                                new pcl::PointCloud<pcl::PointXYZ>);
        pass_through_x.setInputCloud(intermediate_cloud_y);				
        pass_through_x.setFilterFieldName("x");			
        pass_through_x.setFilterLimits(fXpass_min,fXpass_max);			
        pass_through_x.filter(*intermediate_cloud_x);
        ROS_INFO("intermediate_cloud_x size = %ld",intermediate_cloud_x->points.size());
            // 体素滤波
        pcl::PointCloud<pcl::PointXYZ>::Ptr intermediate_cloud_voxel(
                                new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;	
        now_times = ros::Time::now();
        voxel_filter.setInputCloud(intermediate_cloud_x);					
        voxel_filter.setLeafSize(f_voxel, f_voxel, f_voxel);//设置体素大小，单位是m，这里设置5cm的立方体
        voxel_filter.filter(*intermediate_cloud_voxel);	
        ROS_INFO("intermediate_cloud_voxel size = %ld",intermediate_cloud_voxel->points.size());
            // 离群点滤波器
        now_times = ros::Time::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
                                new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter;   
        outlier_filter.setInputCloud (intermediate_cloud_voxel);        
        outlier_filter.setMeanK (f_outlier_meank);                               //设置在进行统计时考虑的临近点个数
        outlier_filter.setStddevMulThresh (f_outlier_thresh);                      //设置判断是否为离群点的阀值，越小越严格,1-2 之间
        outlier_filter.filter (*filtered_cloud);
        ROS_INFO("intermediate_cloud_Outlier size = %ld",filtered_cloud->points.size());
        return filtered_cloud;
    }

    void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
        // 检查图像数据是否为空
        if (msg->data.empty()) {
            ROS_WARN("Received empty depth image");
            return;
        }
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::Mat cropped_depth = cv_ptr->image; 
        // 创建过滤图像
        const int offset = NEIGHBOR_SIZE / 2;
        // 并行遍历像素
        cropped_depth.forEach<uint16_t>([&](uint16_t& pixel, const int* pos) {
            const int y = pos[0], x = pos[1];
            // 跳过边界区域
            if(y < offset || y >= cropped_depth.rows - offset || 
                x < offset || x >= cropped_depth.cols - offset) return;
            // 转换深度单位
            const float depth_m = pixel / 1000.0f;
            // 数据有效性检查
            if(depth_m < MIN_DEPTH || depth_m > MAX_DEPTH) {
                //filtered_depth.at<uint16_t>(y,x) = 0;
                pixel = 0;
                return;
            }
            // 邻域深度一致性检查
            cv::Rect neighbor_roi(x - offset, y - offset, NEIGHBOR_SIZE, NEIGHBOR_SIZE);
            const cv::Mat neighborhood = cropped_depth(neighbor_roi).clone();
            // 计算邻域中值
            cv::Mat sorted;
            cv::sort(neighborhood.reshape(1, 1), sorted, cv::SORT_ASCENDING);
            const float median_depth = sorted.at<uint16_t>(sorted.total()/2) / 1000.0f;
            // 深度差异判断
            if(std::abs(depth_m - median_depth) > DEPTH_DIFF_THRESH) {
                //filtered_depth.at<uint16_t>(y,x) = 0;
                pixel = 0; // 丢弃异常点
            }
        });
        result_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_16UC1, cropped_depth).toImageMsg();
        filtered_pub_.publish(result_msg);
        depth_to_point(result_msg);
    }
    void depth_to_point(const sensor_msgs::Image::ConstPtr& msg){
        try {
            // Step 1: 转换深度图像
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(
                msg, sensor_msgs::image_encodings::TYPE_16UC1);
            // Step 2: 生成原始点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(
                        new pcl::PointCloud<pcl::PointXYZ>);
            raw_cloud->reserve(cv_ptr->image.rows * cv_ptr->image.cols);
            cv::parallel_for_(cv::Range(0, cv_ptr->image.rows), [&](const cv::Range& range) {
                for (int v = range.start; v < range.end; ++v) {
                    const uint16_t* row_ptr = cv_ptr->image.ptr<uint16_t>(v);
                    for (int u = 0; u < cv_ptr->image.cols; ++u) {
                        const uint16_t depth_value = row_ptr[u];
                        if (depth_value == 0) continue;
                        const float z = depth_value / 1000.0f;
                        const float x = (u - cx_) * z / fx_;
                        const float y = (v - cy_) * z / fy_;
                        Eigen::Vector3f p = R_center * R_ * Eigen::Vector3f(x, y, z) + T_;
                        std::lock_guard<std::mutex> lock(mtx); // 加锁
                        raw_cloud->emplace_back(p.x(), p.y(), p.z());
                    }
                }
            });
            // Step 3:过滤
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = points_fliter(raw_cloud);
            // Step 4:检查空点云
            if (filtered_cloud->empty()) {
                ROS_WARN_THROTTLE(1.0, "Filtered cloud is empty!");
                return;
            }
            // Step 5: 发布点云
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*filtered_cloud, output);
            output.header = msg->header; 
            output.header.frame_id = "uav" + std::to_string(uav_id) + "/camera_link";
            pointcloud_pub_.publish(output);
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR_STREAM("cv_bridge error: " << e.what());
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Processing error: " << e.what());
        }
    }       
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_to_cloud");
    Depth_Pointcloud dp;
    ros::spin();
    return 0;
}