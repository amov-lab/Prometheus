/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include "realsense_gazebo_plugin/RealSensePlugin.h"
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>

#define DEPTH_SCALE_M 0.001

#define DEPTH_CAMERA_TOPIC "depth"
#define COLOR_CAMERA_TOPIC "color"
#define IRED1_CAMERA_TOPIC "infrared"
#define IRED2_CAMERA_TOPIC "infrared2"

using namespace gazebo;

/////////////////////////////////////////////////
RealSensePlugin::RealSensePlugin() {
  this->depthCam = nullptr;
  this->ired1Cam = nullptr;
  this->ired2Cam = nullptr;
  this->colorCam = nullptr;
  this->prefix = "";
  this->pointCloudCutOffMax_ = 5.0;
}

/////////////////////////////////////////////////
RealSensePlugin::~RealSensePlugin() {}

/////////////////////////////////////////////////
void RealSensePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  // Output the name of the model

  std::cout
      << std::endl
      << "RealSensePlugin: The realsense_camera plugin is attached to model "
      << _model->GetName() << std::endl;

  _sdf = _sdf->GetFirstElement();

  cameraParamsMap_.insert(std::make_pair(COLOR_CAMERA_NAME, CameraParams()));
  cameraParamsMap_.insert(std::make_pair(DEPTH_CAMERA_NAME, CameraParams()));
  cameraParamsMap_.insert(std::make_pair(IRED1_CAMERA_NAME, CameraParams()));
  cameraParamsMap_.insert(std::make_pair(IRED2_CAMERA_NAME, CameraParams()));

  do {
    std::string name = _sdf->GetName();
    if (name == "depthUpdateRate")
      _sdf->GetValue()->Get(depthUpdateRate_);
    else if (name == "colorUpdateRate")
      _sdf->GetValue()->Get(colorUpdateRate_);
    else if (name == "infraredUpdateRate")
      _sdf->GetValue()->Get(infraredUpdateRate_);
    else if (name == "depthTopicName")
      cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name =
          _sdf->GetValue()->GetAsString();
    else if (name == "depthCameraInfoTopicName")
      cameraParamsMap_[DEPTH_CAMERA_NAME].camera_info_topic_name =
          _sdf->GetValue()->GetAsString();
    else if (name == "colorTopicName")
      cameraParamsMap_[COLOR_CAMERA_NAME].topic_name =
          _sdf->GetValue()->GetAsString();
    else if (name == "colorCameraInfoTopicName")
      cameraParamsMap_[COLOR_CAMERA_NAME].camera_info_topic_name =
          _sdf->GetValue()->GetAsString();
    else if (name == "infrared1TopicName")
      cameraParamsMap_[IRED1_CAMERA_NAME].topic_name =
          _sdf->GetValue()->GetAsString();
    else if (name == "infrared1CameraInfoTopicName")
      cameraParamsMap_[IRED1_CAMERA_NAME].camera_info_topic_name =
          _sdf->GetValue()->GetAsString();
    else if (name == "infrared2TopicName")
      cameraParamsMap_[IRED2_CAMERA_NAME].topic_name =
          _sdf->GetValue()->GetAsString();
    else if (name == "infrared2CameraInfoTopicName")
      cameraParamsMap_[IRED2_CAMERA_NAME].camera_info_topic_name =
          _sdf->GetValue()->GetAsString();
    else if (name == "colorOpticalframeName")
      cameraParamsMap_[COLOR_CAMERA_NAME].optical_frame =
          _sdf->GetValue()->GetAsString();
    else if (name == "depthOpticalframeName")
      cameraParamsMap_[DEPTH_CAMERA_NAME].optical_frame =
          _sdf->GetValue()->GetAsString();
    else if (name == "infrared1OpticalframeName")
      cameraParamsMap_[IRED1_CAMERA_NAME].optical_frame =
          _sdf->GetValue()->GetAsString();
    else if (name == "infrared2OpticalframeName")
      cameraParamsMap_[IRED2_CAMERA_NAME].optical_frame =
          _sdf->GetValue()->GetAsString();
    else if (name == "rangeMinDepth")
      _sdf->GetValue()->Get(rangeMinDepth_);
    else if (name == "rangeMaxDepth")
      _sdf->GetValue()->Get(rangeMaxDepth_);
    else if (name == "pointCloud")
      _sdf->GetValue()->Get(pointCloud_);
    else if (name == "pointCloudTopicName")
      pointCloudTopic_ = _sdf->GetValue()->GetAsString();
    else if (name == "pointCloudCutoff")
      _sdf->GetValue()->Get(pointCloudCutOff_);
    else if (name == "pointCloudCutoffMax")
      _sdf->GetValue()->Get(pointCloudCutOffMax_);
    else if (name == "prefix")
      this->prefix = _sdf->GetValue()->GetAsString();
    else if (name == "robotNamespace")
      break;
    else
      throw std::runtime_error("Ivalid parameter for RealSensePlugin");

    _sdf = _sdf->GetNextElement();
  } while (_sdf);

  // Store a pointer to the this model
  this->rsModel = _model;

  // Store a pointer to the world
  this->world = this->rsModel->GetWorld();

  // Sensors Manager
  sensors::SensorManager *smanager = sensors::SensorManager::Instance();

  // Get Cameras Renderers
  this->depthCam = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(
                       smanager->GetSensor(prefix + DEPTH_CAMERA_NAME))
                       ->DepthCamera();

  this->ired1Cam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                       smanager->GetSensor(prefix + IRED1_CAMERA_NAME))
                       ->Camera();
  this->ired2Cam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                       smanager->GetSensor(prefix + IRED2_CAMERA_NAME))
                       ->Camera();
  this->colorCam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                       smanager->GetSensor(prefix + COLOR_CAMERA_NAME))
                       ->Camera();

  // Check if camera renderers have been found successfuly
  if (!this->depthCam) {
    std::cerr << "RealSensePlugin: Depth Camera has not been found"
              << std::endl;
    return;
  }
  if (!this->ired1Cam) {
    std::cerr << "RealSensePlugin: InfraRed Camera 1 has not been found"
              << std::endl;
    return;
  }
  if (!this->ired2Cam) {
    std::cerr << "RealSensePlugin: InfraRed Camera 2 has not been found"
              << std::endl;
    return;
  }
  if (!this->colorCam) {
    std::cerr << "RealSensePlugin: Color Camera has not been found"
              << std::endl;
    return;
  }

  // Resize Depth Map dimensions
  try {
    this->depthMap.resize(this->depthCam->ImageWidth() *
                          this->depthCam->ImageHeight());
  } catch (std::bad_alloc &e) {
    std::cerr << "RealSensePlugin: depthMap allocation failed: " << e.what()
              << std::endl;
    return;
  }

  // Setup Transport Node
  this->transportNode = transport::NodePtr(new transport::Node());
  this->transportNode->Init(this->world->Name());

  // Setup Publishers
  std::string rsTopicRoot = "~/" + this->rsModel->GetName();

  this->depthPub = this->transportNode->Advertise<msgs::ImageStamped>(
      rsTopicRoot + DEPTH_CAMERA_TOPIC, 1, depthUpdateRate_);
  this->ired1Pub = this->transportNode->Advertise<msgs::ImageStamped>(
      rsTopicRoot + IRED1_CAMERA_TOPIC, 1, infraredUpdateRate_);
  this->ired2Pub = this->transportNode->Advertise<msgs::ImageStamped>(
      rsTopicRoot + IRED2_CAMERA_TOPIC, 1, infraredUpdateRate_);
  this->colorPub = this->transportNode->Advertise<msgs::ImageStamped>(
      rsTopicRoot + COLOR_CAMERA_TOPIC, 1, colorUpdateRate_);

  // Listen to depth camera new frame event

  this->newDepthFrameConn = this->depthCam->ConnectNewDepthFrame(
      std::bind(&RealSensePlugin::OnNewDepthFrame, this));

  this->newIred1FrameConn = this->ired1Cam->ConnectNewImageFrame(std::bind(
      &RealSensePlugin::OnNewFrame, this, this->ired1Cam, this->ired1Pub));

  this->newIred2FrameConn = this->ired2Cam->ConnectNewImageFrame(std::bind(
      &RealSensePlugin::OnNewFrame, this, this->ired2Cam, this->ired2Pub));

  this->newColorFrameConn = this->colorCam->ConnectNewImageFrame(std::bind(
      &RealSensePlugin::OnNewFrame, this, this->colorCam, this->colorPub));


  // Listen to the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&RealSensePlugin::OnUpdate, this));

}

/////////////////////////////////////////////////
void RealSensePlugin::OnNewFrame(const rendering::CameraPtr cam,
                                 const transport::PublisherPtr pub) {
  msgs::ImageStamped msg;

  // Set Simulation Time
  msgs::Set(msg.mutable_time(), this->world->SimTime());

  // Set Image Dimensions
  msg.mutable_image()->set_width(cam->ImageWidth());
  msg.mutable_image()->set_height(cam->ImageHeight());

  // Set Image Pixel Format
  msg.mutable_image()->set_pixel_format(
      common::Image::ConvertPixelFormat(cam->ImageFormat()));

  // Set Image Data
  msg.mutable_image()->set_step(cam->ImageWidth() * cam->ImageDepth());
  msg.mutable_image()->set_data(cam->ImageData(),
                                cam->ImageDepth() * cam->ImageWidth() *
                                    cam->ImageHeight());

  // Publish realsense infrared stream
  pub->Publish(msg);
}

/////////////////////////////////////////////////
void RealSensePlugin::OnNewDepthFrame() {
  // Get Depth Map dimensions
  unsigned int imageSize =
      this->depthCam->ImageWidth() * this->depthCam->ImageHeight();

  // Instantiate message
  msgs::ImageStamped msg;

  // Convert Float depth data to RealSense depth data
  const float *depthDataFloat = this->depthCam->DepthData();
  for (unsigned int i = 0; i < imageSize; ++i) {
    // Check clipping and overflow
    if (depthDataFloat[i] < rangeMinDepth_ ||
        depthDataFloat[i] > rangeMaxDepth_ ||
        depthDataFloat[i] > DEPTH_SCALE_M * UINT16_MAX ||
        depthDataFloat[i] < 0) {
      this->depthMap[i] = 0;
    } else {
      this->depthMap[i] = (uint16_t)(depthDataFloat[i] / DEPTH_SCALE_M);
    }
  }

  // Pack realsense scaled depth map
  msgs::Set(msg.mutable_time(), this->world->SimTime());
  msg.mutable_image()->set_width(this->depthCam->ImageWidth());
  msg.mutable_image()->set_height(this->depthCam->ImageHeight());
  msg.mutable_image()->set_pixel_format(common::Image::L_INT16);
  msg.mutable_image()->set_step(this->depthCam->ImageWidth() *
                                this->depthCam->ImageDepth());
  msg.mutable_image()->set_data(this->depthMap.data(),
                                sizeof(*this->depthMap.data()) * imageSize);

  // Publish realsense scaled depth map
  this->depthPub->Publish(msg);
}

/////////////////////////////////////////////////
void RealSensePlugin::OnUpdate() {}
