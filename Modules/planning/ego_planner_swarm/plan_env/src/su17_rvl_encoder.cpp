#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include "prometheus_msgs/DepthCompressed.h"

ros::Publisher pub_compressed;
// 裁剪参数
const cv::Rect crop_roi_{80, 40, 480, 320};
// 深度图过滤结果
sensor_msgs::ImagePtr result_msg;
// 配置参数
constexpr float MIN_DEPTH = 0.25;    // 最小有效深度 (米)
constexpr float MAX_DEPTH = 5.0;     // 最大有效深度 (米)
constexpr int NEIGHBOR_SIZE = 5;     // 邻域窗口尺寸 (奇数)
constexpr float DEPTH_DIFF_THRESH = 0.2; // 深度差异阈值 (米)

// RVL编码函数
void RVL_Encode(const cv::Mat& depth_img, std::vector<uint8_t>& compressed) {
    const int16_t* data = reinterpret_cast<const int16_t*>(depth_img.data);
    int16_t prev = 0;
    const int total_pixels = depth_img.total();// 返回图像像素总数，即x*y
    compressed.reserve(total_pixels * 2);  // 预分配内存

    for (int i = 0; i < total_pixels; ++i) {
        int16_t diff = data[i] - prev;// 计算当前像素值与前一像素值的差值
        prev = data[i];
        uint16_t u = (diff << 1) ^ (diff >> 15);// 将差分值 diff 映射为非负数;若 diff ≥ 0，结果为 2 * diff（偶数）。若 diff < 0，结果为 -2 * diff - 1（奇数）。

        // 生成变长编码
        bool has_more = true;
        while (has_more) {
            uint8_t chunk = u & 0x7;// 将差分值 u 分解为多个 3 位的块（chunk），并添加终止标记
            u >>= 3;
            has_more = (u != 0);
            
            // 设置终止标记
            chunk |= has_more ? 0x8 : 0x0;
            compressed.push_back(chunk);
        }
    }
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // 转换并裁剪图像
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::Mat cropped_depth = cv_ptr->image(crop_roi_).clone(); // 拷贝一份，让内存强制对齐,不然图像会出问题   
        std::vector<uint8_t> compressed_data;
        RVL_Encode(cropped_depth, compressed_data);
        // 发布带尺寸信息的自定义消息
        prometheus_msgs::DepthCompressed compressed_msg;
        compressed_msg.header = msg->header;
        compressed_msg.height = cropped_depth.rows;
        compressed_msg.width = cropped_depth.cols;
        compressed_msg.data = compressed_data;
        pub_compressed.publish(compressed_msg);

        ROS_DEBUG("Encoded %zux%zu -> %zu bytes", 
                 compressed_msg.height, compressed_msg.width, compressed_data.size());
    } catch (const cv::Exception& e) {
        ROS_ERROR("Encoding error: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "su17_rvl_encoder");
    ros::NodeHandle nh("~");
    int uav_id = 1;
    bool is_debug;
    nh.param("uav_id", uav_id, 1);
    nh.param("debug",is_debug,false);
    // 配置调试输出
    if(is_debug){
        if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
            ros::console::notifyLoggerLevelsChanged();
        }
    }
    ros::Subscriber sub = nh.subscribe("/SU17CAM/depthF", 1, depthCallback);
    pub_compressed = nh.advertise<prometheus_msgs::DepthCompressed>("/uav"+ std::to_string(uav_id) + "/compressed_depth", 1);
    
    ROS_INFO("RVL Encoder initialized");
    ros::Rate rate(10); 
    
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
