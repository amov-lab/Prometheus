#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <numeric>

// 全局变量
cv::Mat depth_frame; // 深度图像矩阵
cv::Mat filtered_depth; // 用于显示深度图像

// 定义深度图最小距离和最大距离
constexpr float MIN_DEPTH = 0.2; // 最小深度 (米)
constexpr float MAX_DEPTH = 10.0; // 最大深度 (米)

// 使用均值滤波替换无效深度值，但这种方法遇到大范围的无效点时也会失效
uint16_t replaceInvalidDepth(const cv::Mat& depth, int x, int y) {
    std::vector<uint16_t> valid_depths;

    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            // 排除中心点并确保点在整个图像范围内
            if (nx >= 0 && nx < depth.cols && ny >= 0 && ny < depth.rows && !(dx == 0 && dy == 0)) {
                uint16_t neighbor_depth = depth.at<uint16_t>(ny, nx);
                float neighbor_depth_m = neighbor_depth / 1000.0;

                if (neighbor_depth_m >= MIN_DEPTH && neighbor_depth_m <= MAX_DEPTH) {
                    valid_depths.push_back(neighbor_depth);
                }
            }
        }
    }

    if (!valid_depths.empty()) {
        uint32_t sum = std::accumulate(valid_depths.begin(), valid_depths.end(), 0);
        return static_cast<uint16_t>(sum / valid_depths.size());
    }
    return 0;
}

// 深度图像回调函数
void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg, ros::Publisher& filtered_pub,ros::Publisher& filtered_pub_display) {
    try {
        // 将 ROS 图像消息转换为 OpenCV 图像
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
        depth_frame = cv_ptr->image.clone(); // 保留原始深度图像
        
        // 创建过滤后的深度图
        filtered_depth = depth_frame.clone();
        for (int y = 0; y < filtered_depth.rows; ++y) {
            for (int x = 0; x < filtered_depth.cols; ++x) {
                uint16_t depth_mm = filtered_depth.at<uint16_t>(y, x);
                float depth_m = depth_mm / 1000.0;

                // 检查深度值并替换
                if (depth_m < MIN_DEPTH || depth_m > MAX_DEPTH) {
                    filtered_depth.at<uint16_t>(y, x) = replaceInvalidDepth(filtered_depth, x, y);
                }
            }
        }
        sensor_msgs::ImagePtr filtered_msgs = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO16, depth_frame).toImageMsg();
        filtered_pub.publish(filtered_msgs);

        sensor_msgs::ImagePtr filtered_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO16, filtered_depth).toImageMsg();
        filtered_pub_display.publish(filtered_msg);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_filter_node");
    ros::NodeHandle nh;

    // 订阅深度图像话题
    ros::Publisher filtered_pub = nh.advertise<sensor_msgs::Image>("/SU17CAM/filtered_depthF", 1);
    ros::Publisher filtered_pub_display = nh.advertise<sensor_msgs::Image>("/SU17CAM/filtered_depthF_display", 1);
    ros::Subscriber depth_sub = nh.subscribe<sensor_msgs::Image>("/SU17CAM/depthF", 1,
        boost::bind(depthImageCallback, _1, boost::ref(filtered_pub),boost::ref(filtered_pub_display)));
    ros::spin();
    return 0;
}