#include "gazebo_ros_realsense.h"
#include <sensor_msgs/fill_image.h>

namespace
{
  std::string extractCameraName(const std::string& name);
  sensor_msgs::CameraInfo cameraInfo(const sensor_msgs::Image& image, float horizontal_fov);
}

namespace gazebo
{
// Register the plugin
GZ_REGISTER_MODEL_PLUGIN(GazeboRosRealsense)

GazeboRosRealsense::GazeboRosRealsense()
{
}

GazeboRosRealsense::~GazeboRosRealsense()
{
  ROS_DEBUG_STREAM_NAMED("realsense_camera", "Unloaded");
}

void GazeboRosRealsense::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  ROS_INFO("Realsense Gazebo ROS plugin loading.");

  RealSensePlugin::Load(_model, _sdf);

  this->rosnode_ = new ros::NodeHandle(this->GetHandle());

  // initialize camera_info_manager
  this->camera_info_manager_.reset(
    new camera_info_manager::CameraInfoManager(*this->rosnode_, this->GetHandle()));

  this->itnode_ = new image_transport::ImageTransport(*this->rosnode_);

  this->color_pub_ = this->itnode_->advertiseCamera("camera/color/image_raw", 2);
  this->ir1_pub_ = this->itnode_->advertiseCamera("camera/ir/image_raw", 2);
  this->ir2_pub_ = this->itnode_->advertiseCamera("camera/ir2/image_raw", 2);
  this->depth_pub_ = this->itnode_->advertiseCamera("camera/depth/image_raw", 2);
}

void GazeboRosRealsense::OnNewFrame(const rendering::CameraPtr cam,
                                    const transport::PublisherPtr pub)
{
  common::Time current_time = this->world->SimTime();

  // identify camera
  std::string camera_id = extractCameraName(cam->Name());
  const std::map<std::string, image_transport::CameraPublisher*> camera_publishers = {
    {COLOR_CAMERA_NAME, &(this->color_pub_)},
    {IRED1_CAMERA_NAME, &(this->ir1_pub_)},
    {IRED2_CAMERA_NAME, &(this->ir2_pub_)},
  };
  const auto image_pub = camera_publishers.at(camera_id);

  // copy data into image
  this->image_msg_.header.frame_id = "realsense_camera_link";
  this->image_msg_.header.stamp.sec = current_time.sec;
  this->image_msg_.header.stamp.nsec = current_time.nsec;

  // set image encoding
  const std::map<std::string, std::string> supported_image_encodings = {
    {"L_INT8", sensor_msgs::image_encodings::MONO8},
    {"RGB_INT8", sensor_msgs::image_encodings::RGB8},
  };
  const auto pixel_format = supported_image_encodings.at(cam->ImageFormat());

  // copy from simulation image to ROS msg
  fillImage(this->image_msg_,
    pixel_format,
    cam->ImageHeight(), cam->ImageWidth(),
    cam->ImageDepth() * cam->ImageWidth(),
    reinterpret_cast<const void*>(cam->ImageData()));

  // identify camera rendering
  const std::map<std::string, rendering::CameraPtr> cameras = {
    {COLOR_CAMERA_NAME, this->colorCam},
    {IRED1_CAMERA_NAME, this->ired1Cam},
    {IRED2_CAMERA_NAME, this->ired2Cam},
  };

  // publish to ROS
  auto camera_info_msg = cameraInfo(this->image_msg_, cameras.at(camera_id)->HFOV().Radian());
  image_pub->publish(this->image_msg_, camera_info_msg);
}

void GazeboRosRealsense::OnNewDepthFrame()
{
  // get current time
  common::Time current_time = this->world->SimTime();

  RealSensePlugin::OnNewDepthFrame();

  // copy data into image
  this->depth_msg_.header.frame_id = "realsense_camera_link";
  this->depth_msg_.header.stamp.sec = current_time.sec;
  this->depth_msg_.header.stamp.nsec = current_time.nsec;

  // set image encoding
  std::string pixel_format = sensor_msgs::image_encodings::TYPE_16UC1;

  // copy from simulation image to ROS msg
  fillImage(this->depth_msg_,
    pixel_format,
    this->depthCam->ImageHeight(), this->depthCam->ImageWidth(),
    2 * this->depthCam->ImageWidth(),
    reinterpret_cast<const void*>(this->depthMap.data()));

  // publish to ROS
  auto depth_info_msg = cameraInfo(this->depth_msg_, this->depthCam->HFOV().Radian());
  this->depth_pub_.publish(this->depth_msg_, depth_info_msg);
}

}

namespace
{
  std::string extractCameraName(const std::string& name)
  {
    if (name.find(COLOR_CAMERA_NAME) != std::string::npos) return COLOR_CAMERA_NAME;
    if (name.find(IRED1_CAMERA_NAME) != std::string::npos) return IRED1_CAMERA_NAME;
    if (name.find(IRED2_CAMERA_NAME) != std::string::npos) return IRED2_CAMERA_NAME;

    ROS_ERROR("Unknown camera name");
    return COLOR_CAMERA_NAME;
  }

  sensor_msgs::CameraInfo cameraInfo(const sensor_msgs::Image& image, float horizontal_fov)
  {
    sensor_msgs::CameraInfo info_msg;

    info_msg.header = image.header;
    info_msg.height = image.height;
    info_msg.width = image.width;

    float focal = 0.5 * image.width / tan(0.5 * horizontal_fov);

    info_msg.K[0] = focal;
    info_msg.K[4] = focal;
    info_msg.K[2] = info_msg.width * 0.5;
    info_msg.K[5] = info_msg.height * 0.5;
    info_msg.K[8] = 1.;

    info_msg.P[0] = info_msg.K[0];
    info_msg.P[5] = info_msg.K[4];
    info_msg.P[2] = info_msg.K[2];
    info_msg.P[6] = info_msg.K[5];
    info_msg.P[10] = info_msg.K[8];

    return info_msg;
  }
}

