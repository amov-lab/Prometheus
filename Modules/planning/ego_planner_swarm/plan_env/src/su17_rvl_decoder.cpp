#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include "prometheus_msgs/DepthCompressed.h"

ros::Publisher pub_decoded;

// RVL解码函数
void RVL_Decode(const std::vector<uint8_t>& compressed, cv::Mat& depth_img) {
    int16_t* data = reinterpret_cast<int16_t*>(depth_img.data);
    int16_t prev = 0;
    uint16_t u = 0;
    int shift = 0;
    size_t idx = 0;
    const size_t max_pixels = depth_img.total();

    for (const auto& byte : compressed) {
        u |= (byte & 0x7) << shift;
        shift += 3;

        // 检测终止标记或溢出
        if (!(byte & 0x8) || shift > 15) {
            if (idx >= max_pixels) {
                ROS_WARN_THROTTLE(1, "Pixel buffer overflow");
                break;
            }

            // 解码差分值
            int16_t diff = (u >> 1) ^ -(u & 1);
            data[idx] = prev + diff;
            prev = data[idx];
            idx++;

            u = 0;
            shift = 0;
        }
    }

    // 填充未解码像素
    if (idx < max_pixels) {
        ROS_WARN("Incomplete data: Filled %zu/%zu pixels", idx, max_pixels);
        std::fill(data + idx, data + max_pixels, 0);
    }
}

void compressedCallback(const prometheus_msgs::DepthCompressedConstPtr& msg) {
    try {
        // 创建矩阵的尺寸
        cv::Mat decoded_img(msg->height, msg->width, CV_16UC1);
        decoded_img.setTo(0);

        // 执行解码
        RVL_Decode(msg->data, decoded_img);

        // 转换为ROS消息
        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header;
        out_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        out_msg.image = decoded_img;
        
        pub_decoded.publish(out_msg.toImageMsg());
        
        ROS_DEBUG("Decoded %zubytes -> %dx%d image", 
                 msg->data.size(), msg->width, msg->height);
    } catch (const std::exception& e) {
        ROS_ERROR("Decoding failed: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "su17_rvl_decoder");
    ros::NodeHandle nh("~");
    int uav_id;
    bool is_debug;
    nh.param("uav_id", uav_id, 1);
    nh.param("debug",is_debug,false);
    // 配置调试输出
    if(is_debug){
        if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
            ros::console::notifyLoggerLevelsChanged();
        }
    }
    ros::Subscriber sub = nh.subscribe("/uav" + std::to_string(uav_id) + "/compressed_depth", 1, compressedCallback);
    pub_decoded = nh.advertise<sensor_msgs::Image>("/uav" + std::to_string(uav_id) + "/restored_depth", 1);
    
    ROS_INFO("RVL Decoder initialized");
    ros::spin();
    return 0;
}