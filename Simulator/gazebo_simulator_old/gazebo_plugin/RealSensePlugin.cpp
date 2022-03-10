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

#include "RealSensePlugin.h"
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>

#define DEPTH_PUB_FREQ_HZ 60
#define COLOR_PUB_FREQ_HZ 60
#define IRED1_PUB_FREQ_HZ 60
#define IRED2_PUB_FREQ_HZ 60

#define DEPTH_CAMERA_TOPIC "depth"
#define COLOR_CAMERA_TOPIC "color"
#define IRED1_CAMERA_TOPIC "infrared"
#define IRED2_CAMERA_TOPIC "infrared2"

#define DEPTH_NEAR_CLIP_M 0.3
#define DEPTH_FAR_CLIP_M 10.0
#define DEPTH_SCALE_M 0.001

using namespace gazebo;

/////////////////////////////////////////////////
RealSensePlugin::RealSensePlugin()
{
  this->depthCam = nullptr;
  this->ired1Cam = nullptr;
  this->ired2Cam = nullptr;
  this->colorCam = nullptr;
}

/////////////////////////////////////////////////
RealSensePlugin::~RealSensePlugin()
{
}

/////////////////////////////////////////////////
void RealSensePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Output the name of the model
  std::cout << std::endl
            << "RealSensePlugin: The realsense_camera plugin is attach to model "
            << _model->GetName() << std::endl;

  // Store a pointer to the this model
  this->rsModel = _model;

  // Store a pointer to the world
  this->world = this->rsModel->GetWorld();

  // Sensors Manager
  sensors::SensorManager *smanager = sensors::SensorManager::Instance();

  // Get Cameras Renderers
  this->depthCam =
      std::dynamic_pointer_cast<sensors::DepthCameraSensor>(
          smanager->GetSensor(DEPTH_CAMERA_NAME))->DepthCamera();
  this->ired1Cam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                                smanager->GetSensor(IRED1_CAMERA_NAME))
                                ->Camera();
  this->ired2Cam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                                smanager->GetSensor(IRED2_CAMERA_NAME))
                                ->Camera();
  this->colorCam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                                smanager->GetSensor(COLOR_CAMERA_NAME))
                                ->Camera();

  // Check if camera renderers have been found successfuly
  if (!this->depthCam)
  {
    std::cerr << "RealSensePlugin: Depth Camera has not been found"
              << std::endl;
    return;
  }
  if (!this->ired1Cam)
  {
    std::cerr << "RealSensePlugin: InfraRed Camera 1 has not been found"
              << std::endl;
    return;
  }
  if (!this->ired2Cam)
  {
    std::cerr << "RealSensePlugin: InfraRed Camera 2 has not been found"
              << std::endl;
    return;
  }
  if (!this->colorCam)
  {
    std::cerr << "RealSensePlugin: Color Camera has not been found"
              << std::endl;
    return;
  }

  // Resize Depth Map dimensions
  try
  {
    this->depthMap.resize(this->depthCam->ImageWidth() * this->depthCam->ImageHeight());
  }
  catch (std::bad_alloc &e)
  {
    std::cerr << "RealSensePlugin: depthMap allocation failed: " << e.what()
              << std::endl;
    return;
  }

  // Setup Transport Node
  this->transportNode = transport::NodePtr(new transport::Node());
  this->transportNode->Init(this->world->Name());

  // Setup Publishers
  std::string rsTopicRoot =
      "~/" + this->rsModel->GetName() + "/rs/stream/";

  this->depthPub = this->transportNode->Advertise<msgs::ImageStamped>(
          rsTopicRoot + DEPTH_CAMERA_TOPIC, 1, DEPTH_PUB_FREQ_HZ);
  this->ired1Pub = this->transportNode->Advertise<msgs::ImageStamped>(
          rsTopicRoot + IRED1_CAMERA_TOPIC, 1, DEPTH_PUB_FREQ_HZ);
  this->ired2Pub = this->transportNode->Advertise<msgs::ImageStamped>(
          rsTopicRoot + IRED2_CAMERA_TOPIC, 1, DEPTH_PUB_FREQ_HZ);
  this->colorPub = this->transportNode->Advertise<msgs::ImageStamped>(
          rsTopicRoot + COLOR_CAMERA_TOPIC, 1, DEPTH_PUB_FREQ_HZ);

  // Listen to depth camera new frame event
  this->newDepthFrameConn = this->depthCam->ConnectNewDepthFrame(
      std::bind(&RealSensePlugin::OnNewDepthFrame, this));

  this->newIred1FrameConn =
      this->ired1Cam->ConnectNewImageFrame(
          std::bind(&RealSensePlugin::OnNewFrame, this, this->ired1Cam,
                    this->ired1Pub));

  this->newIred2FrameConn =
      this->ired2Cam->ConnectNewImageFrame(
          std::bind(&RealSensePlugin::OnNewFrame, this, this->ired2Cam,
                    this->ired2Pub));

  this->newColorFrameConn =
      this->colorCam->ConnectNewImageFrame(
          std::bind(&RealSensePlugin::OnNewFrame, this, this->colorCam,
                    this->colorPub));

  // Listen to the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&RealSensePlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void RealSensePlugin::OnNewFrame(const rendering::CameraPtr cam,
                                 const transport::PublisherPtr pub)
{
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
  msg.mutable_image()->set_data(
      cam->ImageData(),
      cam->ImageDepth() * cam->ImageWidth() * cam->ImageHeight());

  // Publish realsense infrared stream
  pub->Publish(msg);
}

/////////////////////////////////////////////////
void RealSensePlugin::OnNewDepthFrame()
{
  // Get Depth Map dimensions
  unsigned int imageSize = this->depthCam->ImageWidth() *
                           this->depthCam->ImageHeight();

  // Instantiate message
  msgs::ImageStamped msg;

  // Convert Float depth data to RealSense depth data
  const float *depthDataFloat = this->depthCam->DepthData();
  for (unsigned int i = 0; i < imageSize; ++i)
  {
    // Check clipping and overflow
    if (depthDataFloat[i] < DEPTH_NEAR_CLIP_M ||
        depthDataFloat[i] > DEPTH_FAR_CLIP_M ||
        depthDataFloat[i] > DEPTH_SCALE_M * UINT16_MAX ||
        depthDataFloat[i] < 0)
    {
      this->depthMap[i] = 0;
    }
    else
    {
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
  msg.mutable_image()->set_data(
      this->depthMap.data(),
      sizeof(*this->depthMap.data()) * imageSize);

  // Publish realsense scaled depth map
  this->depthPub->Publish(msg);
}

/////////////////////////////////////////////////
void RealSensePlugin::OnUpdate()
{
}
