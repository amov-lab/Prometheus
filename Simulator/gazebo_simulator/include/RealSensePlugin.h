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

#ifndef _GZRS_PLUGIN_HH_
#define _GZRS_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>
#include <sdf/sdf.hh>

#include <string>
#include <memory>

namespace gazebo
{

  #define DEPTH_CAMERA_NAME "depth"
  #define COLOR_CAMERA_NAME "color"
  #define IRED1_CAMERA_NAME "ired1"
  #define IRED2_CAMERA_NAME "ired2"

  /// \brief A plugin that simulates Real Sense camera streams.
  class RealSensePlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: RealSensePlugin();

    /// \brief Destructor.
    public: ~RealSensePlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for the World Update event.
    public: void OnUpdate();

    /// \brief Callback that publishes a received Depth Camera Frame as an
    /// ImageStamped
    /// message.
    public: virtual void OnNewDepthFrame();

    /// \brief Callback that publishes a received Camera Frame as an
    /// ImageStamped message.
    public: virtual void OnNewFrame(const rendering::CameraPtr cam,
                                    const transport::PublisherPtr pub);

    /// \brief Pointer to the model containing the plugin.
    protected: physics::ModelPtr rsModel;

    /// \brief Pointer to the world.
    protected: physics::WorldPtr world;

    /// \brief Pointer to the Depth Camera Renderer.
    protected: rendering::DepthCameraPtr depthCam;

    /// \brief Pointer to the Color Camera Renderer.
    protected: rendering::CameraPtr colorCam;

    /// \brief Pointer to the Infrared Camera Renderer.
    protected: rendering::CameraPtr ired1Cam;

    /// \brief Pointer to the Infrared2 Camera Renderer.
    protected: rendering::CameraPtr ired2Cam;

    /// \brief Pointer to the transport Node.
    protected: transport::NodePtr transportNode;

    // \brief Store Real Sense depth map data.
    protected: std::vector<uint16_t> depthMap;

    /// \brief Pointer to the Depth Publisher.
    protected: transport::PublisherPtr depthPub;

    /// \brief Pointer to the Color Publisher.
    protected: transport::PublisherPtr colorPub;

    /// \brief Pointer to the Infrared Publisher.
    protected: transport::PublisherPtr ired1Pub;

    /// \brief Pointer to the Infrared2 Publisher.
    protected: transport::PublisherPtr ired2Pub;

    /// \brief Pointer to the Depth Camera callback connection.
    protected: event::ConnectionPtr newDepthFrameConn;

    /// \brief Pointer to the Depth Camera callback connection.
    protected: event::ConnectionPtr newIred1FrameConn;

    /// \brief Pointer to the Infrared Camera callback connection.
    protected: event::ConnectionPtr newIred2FrameConn;

    /// \brief Pointer to the Color Camera callback connection.
    protected: event::ConnectionPtr newColorFrameConn;

    /// \brief Pointer to the World Update event connection.
    protected: event::ConnectionPtr updateConnection;
  };
}
#endif
