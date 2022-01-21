#include "ros/ros.h"
#include "OptiTrackFeedBackRigidBody.h"
#include "KeyboardEvent.h"
#include <Eigen/Eigen>
using namespace std;
using namespace Eigen;
int main(int argc, char **argv)
{
    double Control_Rate = 40;// Hz the rate

    // Initialize ros node
    ros::init(argc, argv, "OptiTrackTest");
    ros::NodeHandle n;
    // Initialize Data Recorder
    //DataRecorder Rigidbody1_recorder("OptiTestRigidbody1.txt",31,Control_Rate);
        double Rigidbody1state[16];
    /*  0 OptTrack Flag
        1 - 3 Rigidbody1 positition from OpTiFeedback (hx,hy,Altitude)
        4 - 6 Rigidbody1 filtered velocity from OpTiFeedback
        7 - 9 Rigidbody1 raw velocity from Optifeedback
        10 - 12 Rigidbody1 raw angular velocity from Optifeedback
        13 - 15 Rigidbody1 filtered angular velocity from Optifeedback
        16 - 18 Rigidbody1 euler angle from OpTiFeedback
        19 - 22 Rigidbody1 quaterion from OpTiFeedback
        23 - 31 Rigidbody1 Rotation matrix R_IB from OptiFeedback
     */
    // Initialize OptiTrack System
    OptiTrackFeedBackRigidBody Opti_RigidBody1("/vrpn_client_node/UAV/pose",n,3,3);
    OptiTrackFeedBackRigidBody Opti_RigidBody2("/vrpn_client_node/RigidBody2/pose",n,3,3);
    KeyboardEvent keyboardcontrol;
    rigidbody_state RigidBody1state;
    ros::Rate loop_rate(Control_Rate);
  while (ros::ok())
  {
      Opti_RigidBody1.RosWhileLoopRun();
      Opti_RigidBody2.RosWhileLoopRun();
      keyboardcontrol.RosWhileLoopRun();
      switch (keyboardcontrol.GetPressedKey())
      {
        
        case U_KEY_E:
        {
          //Rigidbody1_recorder.StartRecording();
        }
        case U_KEY_Q:
        {
           Opti_RigidBody1.GetOptiTrackState();
           Opti_RigidBody2.GetOptiTrackState();
           Opti_RigidBody1.GetState(RigidBody1state);
           cout<< "Rigid 1 Position: \n" << RigidBody1state.Position << endl;
           cout<< "Rigid 1 Velocity: \n" << RigidBody1state.V_I << endl;
           cout<< "Rigid 1 AngularVelocity: \n" << RigidBody1state.Omega_BI << endl;
           cout<< "Rigid 1 R_IB: \n" << RigidBody1state.R_IB << endl;
           cout<< "Rigid 1 R_BI: \n" << RigidBody1state.R_BI << endl;
           cout<< "Rigid 1 Rotation Test: \n" << RigidBody1state.R_IB * RigidBody1state.R_BI<< endl;
           cout<< "Rigid 1 Euler: \n" <<RigidBody1state.Euler*57.3<<endl;
           cout<< "Rigid 1 quaterion: \n" <<RigidBody1state.quaterion<<endl;
           //Rigidbody1_recorder.StopRecording();
           break;
        }
        case U_KEY_NONE:
        {
          break;
        }
      }
      //Rigidbody1_recorder.RecorderUpdate(Rigidbody1state);
      ros::spinOnce();// do the loop once
      loop_rate.sleep();

  }
  return 0;
}
