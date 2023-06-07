#include "realsense_gazebo_plugin/gazebo_ros_realsense.h"
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace {
std::string extractCameraName(const std::string &name);
sensor_msgs::CameraInfo cameraInfo(const sensor_msgs::Image &image,
                                   float horizontal_fov);
}

namespace gazebo {
// Register the plugin
GZ_REGISTER_MODEL_PLUGIN(GazeboRosRealsense)

GazeboRosRealsense::GazeboRosRealsense() {}

GazeboRosRealsense::~GazeboRosRealsense() {
  ROS_DEBUG_STREAM_NAMED("realsense_camera", "Unloaded");
}

void GazeboRosRealsense::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable "
                     "to load plugin. "
                     << "Load the Gazebo system plugin "
                        "'libgazebo_ros_api_plugin.so' in the gazebo_ros "
                        "package)");
    return;
  }
  ROS_INFO("Realsense Gazebo ROS plugin loading.");

  RealSensePlugin::Load(_model, _sdf);

  this->rosnode_ = new ros::NodeHandle(this->GetHandle());

  // initialize camera_info_manager
  this->camera_info_manager_.reset(
      new camera_info_manager::CameraInfoManager(*this->rosnode_, this->GetHandle()));

  this->itnode_ = new image_transport::ImageTransport(*this->rosnode_);

  this->color_pub_ = this->itnode_->advertiseCamera(
      cameraParamsMap_[COLOR_CAMERA_NAME].topic_name, 2);
  this->ir1_pub_ = this->itnode_->advertiseCamera(
      cameraParamsMap_[IRED1_CAMERA_NAME].topic_name, 2);
  this->ir2_pub_ = this->itnode_->advertiseCamera(
      cameraParamsMap_[IRED2_CAMERA_NAME].topic_name, 2);
  this->depth_pub_ = this->itnode_->advertiseCamera(
      cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name, 2);
  if (pointCloud_)
  {
    this->pointcloud_pub_ =
        this->rosnode_->advertise<sensor_msgs::PointCloud2>(pointCloudTopic_, 2, false);
  }
}

void GazeboRosRealsense::OnNewFrame(const rendering::CameraPtr cam,
                                    const transport::PublisherPtr pub) {
  common::Time current_time = this->world->SimTime();

  // identify camera
  std::string camera_id = extractCameraName(cam->Name());
  const std::map<std::string, image_transport::CameraPublisher *>
      camera_publishers = {
          {COLOR_CAMERA_NAME, &(this->color_pub_)},
          {IRED1_CAMERA_NAME, &(this->ir1_pub_)},
          {IRED2_CAMERA_NAME, &(this->ir2_pub_)},
      };
  const auto image_pub = camera_publishers.at(camera_id);

  // copy data into image
  this->image_msg_.header.frame_id =
      this->cameraParamsMap_[camera_id].optical_frame;
  // this->image_msg_.header.stamp.sec = current_time.sec;
  // this->image_msg_.header.stamp.nsec = current_time.nsec;

  this->image_msg_.header.stamp = ros::Time::now();

  // set image encoding
  const std::map<std::string, std::string> supported_image_encodings = {
      {"RGB_INT8", sensor_msgs::image_encodings::RGB8},
      {"L_INT8", sensor_msgs::image_encodings::TYPE_8UC1}};
  const auto pixel_format = supported_image_encodings.at(cam->ImageFormat());

  // copy from simulation image to ROS msg
  fillImage(this->image_msg_, pixel_format, cam->ImageHeight(),
            cam->ImageWidth(), cam->ImageDepth() * cam->ImageWidth(),
            reinterpret_cast<const void *>(cam->ImageData()));

  // identify camera rendering
  const std::map<std::string, rendering::CameraPtr> cameras = {
      {COLOR_CAMERA_NAME, this->colorCam},
      {IRED1_CAMERA_NAME, this->ired1Cam},
      {IRED2_CAMERA_NAME, this->ired2Cam},
  };

  // publish to ROS
  auto camera_info_msg =
      cameraInfo(this->image_msg_, cameras.at(camera_id)->HFOV().Radian());
  image_pub->publish(this->image_msg_, camera_info_msg);
}

// Referenced from gazebo_plugins
// https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_openni_kinect.cpp#L302
// Fill depth information
bool GazeboRosRealsense::FillPointCloudHelper(sensor_msgs::PointCloud2 &point_cloud_msg,
                                              uint32_t rows_arg, uint32_t cols_arg,
                                              uint32_t step_arg, void *data_arg)
{
  sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  // convert to flat array shape, we need to reconvert later
  pcd_modifier.resize(rows_arg * cols_arg);
  point_cloud_msg.is_dense = true;

  sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg_, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(pointcloud_msg_, "rgb");

  float *toCopyFrom = (float *)data_arg;
  int index = 0;

  double hfov = this->depthCam->HFOV().Radian();
  double fl = ((double)this->depthCam->ImageWidth()) / (2.0 * tan(hfov / 2.0));

  // convert depth to point cloud
  for (uint32_t j = 0; j < rows_arg; j++)
  {
    double pAngle;
    if (rows_arg > 1)
      pAngle = atan2((double)j - 0.5 * (double)(rows_arg - 1), fl);
    else
      pAngle = 0.0;

    for (uint32_t i = 0; i < cols_arg; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb)
    {
      double yAngle;
      if (cols_arg > 1)
        yAngle = atan2((double)i - 0.5 * (double)(cols_arg - 1), fl);
      else
        yAngle = 0.0;

      double depth = toCopyFrom[index++];  // + 0.0*this->myParent->GetNearClip();

      if (depth > pointCloudCutOff_ && depth < pointCloudCutOffMax_)
      {
        // in optical frame
        // hardcoded rotation rpy(-M_PI/2, 0, -M_PI/2) is built-in
        // to urdf, where the *_optical_frame should have above relative
        // rotation from the physical camera *_frame
        *iter_x = depth * tan(yAngle);
        *iter_y = depth * tan(pAngle);
        *iter_z = depth;
      }
      else  // point in the unseeable range
      {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        point_cloud_msg.is_dense = false;
      }

      // put image color data for each point
      uint8_t *image_src = (uint8_t *)(&(this->image_msg_.data[0]));
      if (this->image_msg_.data.size() == rows_arg * cols_arg * 3)
      {
        // color
        iter_rgb[0] = image_src[i * 3 + j * cols_arg * 3 + 0];
        iter_rgb[1] = image_src[i * 3 + j * cols_arg * 3 + 1];
        iter_rgb[2] = image_src[i * 3 + j * cols_arg * 3 + 2];
      }
      else if (this->image_msg_.data.size() == rows_arg * cols_arg)
      {
        // mono (or bayer?  @todo; fix for bayer)
        iter_rgb[0] = image_src[i + j * cols_arg];
        iter_rgb[1] = image_src[i + j * cols_arg];
        iter_rgb[2] = image_src[i + j * cols_arg];
      }
      else
      {
        // no image
        iter_rgb[0] = 0;
        iter_rgb[1] = 0;
        iter_rgb[2] = 0;
      }
    }
  }

  // reconvert to original height and width after the flat reshape
  point_cloud_msg.height = rows_arg;
  point_cloud_msg.width = cols_arg;
  point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width;

  return true;
}

void GazeboRosRealsense::OnNewDepthFrame() {
  // get current time
  common::Time current_time = this->world->SimTime();

  RealSensePlugin::OnNewDepthFrame();

  // copy data into image
  this->depth_msg_.header.frame_id =
      this->cameraParamsMap_[DEPTH_CAMERA_NAME].optical_frame;
  ;
  // this->depth_msg_.header.stamp.sec = current_time.sec;
  // this->depth_msg_.header.stamp.nsec = current_time.nsec;
  // 适配Prometheus
  this->depth_msg_.header.stamp = ros::Time::now();

  // set image encoding
  std::string pixel_format = sensor_msgs::image_encodings::TYPE_16UC1;

  // copy from simulation image to ROS msg
  fillImage(this->depth_msg_, pixel_format, this->depthCam->ImageHeight(),
            this->depthCam->ImageWidth(), 2 * this->depthCam->ImageWidth(),
            reinterpret_cast<const void *>(this->depthMap.data()));

  // publish to ROS
  auto depth_info_msg =
      cameraInfo(this->depth_msg_, this->depthCam->HFOV().Radian());
  this->depth_pub_.publish(this->depth_msg_, depth_info_msg);

  if (pointCloud_ && this->pointcloud_pub_.getNumSubscribers() > 0)
  {
    this->pointcloud_msg_.header = this->depth_msg_.header;
    // 增加仿真时间戳，适配Prometheus
    this->pointcloud_msg_.header.stamp = ros::Time::now(); //时间戳
    // 修改Frame，适配Prometheus
    this->pointcloud_msg_.header.frame_id = "uav1/camera_link"; 
    this->pointcloud_msg_.width = this->depthCam->ImageWidth();
    this->pointcloud_msg_.height = this->depthCam->ImageHeight();
    this->pointcloud_msg_.row_step =
        this->pointcloud_msg_.point_step * this->depthCam->ImageWidth();
    FillPointCloudHelper(this->pointcloud_msg_, this->depthCam->ImageHeight(),
                         this->depthCam->ImageWidth(), 2 * this->depthCam->ImageWidth(),
                         (void *)this->depthCam->DepthData());
    this->pointcloud_pub_.publish(this->pointcloud_msg_);
  }
}
}

namespace {
std::string extractCameraName(const std::string &name) {
  if (name.find(COLOR_CAMERA_NAME) != std::string::npos)
    return COLOR_CAMERA_NAME;
  if (name.find(IRED1_CAMERA_NAME) != std::string::npos)
    return IRED1_CAMERA_NAME;
  if (name.find(IRED2_CAMERA_NAME) != std::string::npos)
    return IRED2_CAMERA_NAME;

  ROS_ERROR("Unknown camera name");
  return COLOR_CAMERA_NAME;
}

sensor_msgs::CameraInfo cameraInfo(const sensor_msgs::Image &image,
                                   float horizontal_fov) {
  sensor_msgs::CameraInfo info_msg;

  info_msg.header = image.header;
  // 适配Prometheus
  info_msg.header.stamp = ros::Time::now(); //时间戳
  info_msg.distortion_model = "plumb_bob";
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

  //    info_msg.roi.do_rectify = true;

  return info_msg;
}
}
