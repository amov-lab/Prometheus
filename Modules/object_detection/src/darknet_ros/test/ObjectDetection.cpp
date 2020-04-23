/*
 * ObjectDetection.cpp
 *
 *  Created on: Jan 07, 2017
 *      Author: Marko Bjelonic
 *	 Institute: ETH Zurich, Robotic Systems Lab
 */


// Google Test
#include <gtest/gtest.h>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <actionlib/client/simple_action_client.h>

// boost
#include <boost/thread.hpp>

// OpenCV2.
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

// Actions.
#include <darknet_ros_msgs/CheckForObjectsAction.h>

typedef actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction> CheckForObjectsActionClient;
typedef std::shared_ptr<CheckForObjectsActionClient> CheckForObjectsActionClientPtr;

// c++
#include <string>
#include <cmath>

#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_ = DARKNET_FILE_PATH;
#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif

darknet_ros_msgs::BoundingBoxes boundingBoxesResults_;

/*!
 * Done-callback for CheckForObjects action client.
 * @param[in] state
 * @param[in] result
 */
void checkForObjectsResultCB(
    const actionlib::SimpleClientGoalState& state,
    const darknet_ros_msgs::CheckForObjectsResultConstPtr& result) {
  std::cout <<  "[ObjectDetectionTest] Received bounding boxes." << std::endl;

  boundingBoxesResults_ = result->bounding_boxes;
}

bool sendImageToYolo(ros::NodeHandle nh, const std::string& pathToTestImage) {
  //!Check for objects action client.
  CheckForObjectsActionClientPtr checkForObjectsActionClient;

  // Action clients.
  std::string checkForObjectsActionName;
  nh.param("/darknet_ros/camera_action", checkForObjectsActionName, std::string("/darknet_ros/check_for_objects"));
  checkForObjectsActionClient.reset(
      new CheckForObjectsActionClient(
          nh, checkForObjectsActionName,
          true));

  // Wait till action server launches.
  if(!checkForObjectsActionClient->waitForServer(ros::Duration(20.0))) {
	  std::cout << "[ObjectDetectionTest] sendImageToYolo(): checkForObjects action server has not been advertised." << std::endl;
	  return false;
  }

  // Get test image
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  cv_ptr->image = cv::imread(pathToTestImage, CV_LOAD_IMAGE_COLOR);
  cv_ptr->encoding = sensor_msgs::image_encodings::RGB8;
  sensor_msgs::ImagePtr image = cv_ptr->toImageMsg();

  // Generate goal.
  darknet_ros_msgs::CheckForObjectsGoal goal;
  goal.image = *image;

  // Send goal.
  ros::Time beginYolo = ros::Time::now();
  checkForObjectsActionClient->sendGoal(
      goal,
      boost::bind(&checkForObjectsResultCB, _1, _2),
      CheckForObjectsActionClient::SimpleActiveCallback(),
      CheckForObjectsActionClient::SimpleFeedbackCallback());

  if(!checkForObjectsActionClient->waitForResult(ros::Duration(100.0))) {
    std::cout << "[ObjectDetectionTest] sendImageToYolo(): checkForObjects action server took to long to send back result." << std::endl;
    return false;
  }
  ros::Time endYolo = ros::Time::now();
  std::cout << "[ObjectDetectionTest] Object detection for one image took " << endYolo-beginYolo << " seconds." << std::endl;
  return true;
}

TEST(ObjectDetection, DISABLED_DetectDog)
{
  srand((unsigned int) time(0));
  ros::NodeHandle nodeHandle("~");

  // Path to test image.
  std::string pathToTestImage = darknetFilePath_;
  pathToTestImage += "/data/";
  pathToTestImage += "dog";
  pathToTestImage += ".jpg";

  // Send dog image to yolo.
  ASSERT_TRUE(sendImageToYolo(nodeHandle, pathToTestImage));
  ASSERT_TRUE(sendImageToYolo(nodeHandle, pathToTestImage));

  // Evaluate if yolo was able to detect the three objects: dog, bicycle and car.
  bool detectedDog = false;
  double centerErrorDog;
  bool detectedBicycle = false;
  double centerErrorBicycle;
  bool detectedCar = false;
  double centerErrorCar;

  for(unsigned int i = 0; i < boundingBoxesResults_.bounding_boxes.size(); ++i) {
    double xPosCenter = boundingBoxesResults_.bounding_boxes.at(i).xmin +
        (boundingBoxesResults_.bounding_boxes.at(i).xmax - boundingBoxesResults_.bounding_boxes.at(i).xmin)*0.5;
    double yPosCenter = boundingBoxesResults_.bounding_boxes.at(i).ymin +
        (boundingBoxesResults_.bounding_boxes.at(i).ymax - boundingBoxesResults_.bounding_boxes.at(i).ymin)*0.5;

    if(boundingBoxesResults_.bounding_boxes.at(i).Class == "dog") {
      detectedDog = true;
      //std::cout << "centerErrorDog  " << xPosCenter << ", " <<  yPosCenter << std::endl;
      centerErrorDog = std::sqrt(std::pow(xPosCenter - 222.5, 2) + std::pow(yPosCenter - 361.5, 2));
    }
    if(boundingBoxesResults_.bounding_boxes.at(i).Class == "bicycle") {
      detectedBicycle = true;
      //std::cout << "centerErrorBicycle "  << xPosCenter << ", " <<  yPosCenter << std::endl;
      centerErrorBicycle = std::sqrt(std::pow(xPosCenter - 338.0, 2) + std::pow(yPosCenter - 289.0, 2));
    }
    if(boundingBoxesResults_.bounding_boxes.at(i).Class == "truck") {
      detectedCar = true;
      //std::cout << "centerErrorCar  " << xPosCenter << ", " <<  yPosCenter << std::endl;
      centerErrorCar = std::sqrt(std::pow(xPosCenter - 561.0, 2) + std::pow(yPosCenter - 126.5, 2));
    }
  }

  ASSERT_TRUE(detectedDog);
  EXPECT_LT(centerErrorDog, 40.0);
  ASSERT_TRUE(detectedBicycle);
  EXPECT_LT(centerErrorBicycle, 40.0);
  ASSERT_TRUE(detectedCar);
  EXPECT_LT(centerErrorCar, 40.0);
}

TEST(ObjectDetection, DetectANYmal)
{
  srand((unsigned int) time(0));
  ros::NodeHandle nodeHandle("~");

  // Path to test image.
  std::string pathToTestImage = ros::package::getPath("darknet_ros");
  pathToTestImage += "/doc/";
  pathToTestImage += "quadruped_anymal_and_person";
  pathToTestImage += ".JPG";

  // Send dog image to yolo.
  ASSERT_TRUE(sendImageToYolo(nodeHandle, pathToTestImage));
  ASSERT_TRUE(sendImageToYolo(nodeHandle, pathToTestImage));

  // Evaluate if yolo was able to detect the three objects: dog, bicycle and car.
  bool detectedPerson = false;
  double centerErrorPersonX;
  double centerErrorPersonY;

  for(unsigned int i = 0; i < boundingBoxesResults_.bounding_boxes.size(); ++i) {
    double xPosCenter = boundingBoxesResults_.bounding_boxes.at(i).xmin +
        (boundingBoxesResults_.bounding_boxes.at(i).xmax - boundingBoxesResults_.bounding_boxes.at(i).xmin)*0.5;
    double yPosCenter = boundingBoxesResults_.bounding_boxes.at(i).ymin +
        (boundingBoxesResults_.bounding_boxes.at(i).ymax - boundingBoxesResults_.bounding_boxes.at(i).ymin)*0.5;

    if(boundingBoxesResults_.bounding_boxes.at(i).Class == "person") {
      detectedPerson = true;
      centerErrorPersonX = std::sqrt(std::pow(xPosCenter - 1650.0, 2));
      centerErrorPersonY = std::sqrt(std::pow(xPosCenter - 1675.0, 2));
    }
  }

  ASSERT_TRUE(detectedPerson);
  EXPECT_LT(centerErrorPersonX, 30);
  EXPECT_LT(centerErrorPersonY, 30);
}

TEST(ObjectDetection, DISABLED_DetectPerson) {
  srand((unsigned int) time(0));
  ros::NodeHandle nodeHandle("~");

  // Path to test image.
  std::string pathToTestImage = darknetFilePath_;
  pathToTestImage += "/data/";
  pathToTestImage += "person";
  pathToTestImage += ".jpg";

  ASSERT_TRUE(sendImageToYolo(nodeHandle, pathToTestImage));
  ASSERT_TRUE(sendImageToYolo(nodeHandle, pathToTestImage));

  // Evaluate if yolo was able to detect the person.
  bool detectedPerson = false;
  double centerErrorPerson;

  for(unsigned int i = 0; i < boundingBoxesResults_.bounding_boxes.size(); ++i) {
    double xPosCenter = boundingBoxesResults_.bounding_boxes.at(i).xmin +
        (boundingBoxesResults_.bounding_boxes.at(i).xmax - boundingBoxesResults_.bounding_boxes.at(i).xmin)*0.5;
    double yPosCenter = boundingBoxesResults_.bounding_boxes.at(i).ymin +
        (boundingBoxesResults_.bounding_boxes.at(i).ymax - boundingBoxesResults_.bounding_boxes.at(i).ymin)*0.5;

    if(boundingBoxesResults_.bounding_boxes.at(i).Class == "person") {
      detectedPerson = true;
      //std::cout << "centerErrorPerson  " << xPosCenter << ", " <<  yPosCenter << std::endl;
      centerErrorPerson = std::sqrt(std::pow(xPosCenter - 228.0, 2) + std::pow(yPosCenter - 238.0, 2));
    }
  }

  ASSERT_TRUE(detectedPerson);
  EXPECT_LT(centerErrorPerson, 40.0);
}
