#include "OptiTrackFeedBackRigidBody.h"

OptiTrackFeedBackRigidBody::OptiTrackFeedBackRigidBody(const char* name,ros::NodeHandle& n,unsigned int linear_window, unsigned int angular_window)
{
    // load filter window size
    linear_velocity_window = linear_window;
    angular_velocity_window = angular_window;
    if(linear_velocity_window>max_windowsize)
    {
        ROS_INFO("Linear Velocity Window Size Overlimit, Max Value is [%d]",max_windowsize);
        ROS_INFO("Input Valude is [%d]",linear_velocity_window);
        linear_velocity_window = max_windowsize;
    }
    if(angular_velocity_window>max_windowsize)
    {
        ROS_INFO("Angular Velocity Window Size Overlimit, Max Value is [%d]",max_windowsize);
        ROS_INFO("Input Valude is [%d]",angular_velocity_window);
        angular_velocity_window = max_windowsize;
    }
    // set up subscriber to vrpn optitrack beedback
    subOptiTrack = n.subscribe(name, 1, &OptiTrackFeedBackRigidBody::OptiTrackCallback,this);
    //Initialize all velocity
    for(int i =0;i<max_windowsize;i++)
    {
        velocity_raw[i](0)=0;
        velocity_raw[i](1)=0;
        velocity_raw[i](2)=0;
        angular_velocity_raw[i](0)=0;
        angular_velocity_raw[i](1)=0;
        angular_velocity_raw[i](2)=0;
    }
    velocity_filtered(0)=0;
    velocity_filtered(1)=0;
    velocity_filtered(2)=0;
    angular_velocity_filtered(0)=0;
    angular_velocity_filtered(1)=0;
    angular_velocity_filtered(2)=0;
    //Initialize all pose
    for(int i = 0;i<2;i++)
    {
        pose[i].q0 = 1;
        pose[i].q1 = 0;
        pose[i].q2 = 0;
        pose[i].q3 = 0;
        pose[i].t = 0;
        pose[i].Position(0) = 0;
        pose[i].Position(1) = 0;
        pose[i].Position(2) = 0;
        pose[i].L<< 0,1,0,0,
                    0,0,1,0,
                    0,0,0,1;
        pose[i].R<< 0,1,0,0,
                    0,0,1,0,
                    0,0,0,1;   
        pose[1].R_IB<< 1,0,0,
                    0,1,0,
                    0,0,1;
        pose[1].R_BI<< 1,0,0,
                    0,1,0,
                    0,0,1;                    
    }
    // Initialize flag
    OptiTrackFlag = 0;
    FeedbackState = 0;
}

void OptiTrackFeedBackRigidBody::CalculateVelocityFromPose()
{

    /* Logic:
     * 1) push the current pose into buffer
     * 2) determine whether the buffer has a valid time value  (pose[0].t >0); if so calculate velocity
     * 3) if not just set the velocity_onestep as zero
     * 4) push current time, and velocity_onestep into the velocity buffer
     * 5) update the filtered velocity
    */
    // perform the Logic:
    // step (1): push the current pose into buffer
    PushPose();
    // step (2): determine whether the buffer has a valid time value  (pose[0].t >0); if so calculate velocity
    double dt = 0.0;
    Vector3d velocity_onestep;
    Vector3d angular_velocity_onestep;
  if (pose[0].t >0)// calculate only when last time stamp has been recorded.
  {
      // step (2)
      dt = pose[1].t - pose[0].t;// time step
      // calculate linear velocity
      velocity_onestep = (pose[1].Position- pose[0].Position)/dt;
      // calculate angular velocity
      Matrix3d RotationDifference = - pose[1].R_BI*pose[0].R_IB/dt;
      Veemap(RotationDifference,angular_velocity_onestep);
  }else// step (3): if not set velocity to zero
  {
      velocity_onestep(0) = 0.0;
      velocity_onestep(1) = 0.0;
      velocity_onestep(2) = 0.0;
      angular_velocity_onestep(0) = 0.0;
      angular_velocity_onestep(1) = 0.0;
      angular_velocity_onestep(2) = 0.0;
  }
  // step (4): push current time, and velocity_onestep into the velocity buffer
  PushRawVelocity(velocity_onestep,angular_velocity_onestep);
  // step (5): update filtered velocity
  MovingWindowAveraging();
}
void OptiTrackFeedBackRigidBody::PushPose()
{
    pose[0] = pose[1];// straightforward push the pose into buffer
    // update the latest pose
    double t_current = (double)OptiTrackdata.header.stamp.sec + (double)OptiTrackdata.header.stamp.nsec*0.000000001;
    pose[1].t = t_current;
    // take a special note at the order of the quaterion
    pose[1].q0 = OptiTrackdata.pose.orientation.w;
    pose[1].q1 = OptiTrackdata.pose.orientation.x;
    pose[1].q2 = OptiTrackdata.pose.orientation.y;
    pose[1].q3 = OptiTrackdata.pose.orientation.z;
    // update the auxiliary matrix
    /*
    L = [-q1 q0 q3 -q2;
         -q2 -q3 q0 q1;
         -q3 q2 -q1 q0]
    R = [-q1 q0 -q3 q2;
         -q2 q3 q0 -q1;
         -q3 -q2 q1 q0]
    R_IB = RL^T
    */
    pose[1].L(0,0) = - pose[1].q1;
    pose[1].L(1,0) = - pose[1].q2;
    pose[1].L(2,0) = - pose[1].q3;

    pose[1].L(0,1) = pose[1].q0;
    pose[1].L(1,2) = pose[1].q0;
    pose[1].L(2,3) = pose[1].q0;

    pose[1].L(0,2) = pose[1].q3;
    pose[1].L(0,3) = - pose[1].q2;
    pose[1].L(1,1) = - pose[1].q3;
    pose[1].L(1,3) = pose[1].q1;
    pose[1].L(2,1) = pose[1].q2;
    pose[1].L(2,2) = - pose[1].q1;

    pose[1].R(0,0) = - pose[1].q1;
    pose[1].R(1,0) = - pose[1].q2;
    pose[1].R(2,0) = - pose[1].q3;

    pose[1].R(0,1) = pose[1].q0;
    pose[1].R(1,2) = pose[1].q0;
    pose[1].R(2,3) = pose[1].q0;

    pose[1].R(0,2) = -pose[1].q3;
    pose[1].R(0,3) =  pose[1].q2;
    pose[1].R(1,1) =  pose[1].q3;
    pose[1].R(1,3) = -pose[1].q1;
    pose[1].R(2,1) = -pose[1].q2;
    pose[1].R(2,2) =  pose[1].q1; 

    pose[1].R_IB = pose[1].R * pose[1].L.transpose();
    pose[1].R_BI = pose[1].R_IB.transpose();
    // position is straight forward
    pose[1].Position(0) =  OptiTrackdata.pose.position.x;
    pose[1].Position(1) =  OptiTrackdata.pose.position.y;
    pose[1].Position(2) =  OptiTrackdata.pose.position.z;
}

void OptiTrackFeedBackRigidBody::PushRawVelocity(Vector3d& new_linear_velocity, Vector3d& new_angular_velocity)
{
    /* Logic:
     * a(i-1) = a(i), i = 2...windowsize
     * should fristly start from  i = 2. a(1) = a(2); a(2) = a(3);....; a(N-1) = a(N)
     * secondly a(N) = a_new
    */
   // linear velocity
    for(int i = 1;i<linear_velocity_window;i++)//first step
    {
        velocity_raw[i-1] = velocity_raw[i];
    }
    velocity_raw[linear_velocity_window-1] = new_linear_velocity;// second step update the last variable in the velocity buffer
    // angular velocity
    for(int i = 1;i<angular_velocity_window;i++)//first step
    {
        angular_velocity_raw[i-1] = angular_velocity_raw[i];
    }
    angular_velocity_raw[angular_velocity_window-1] = new_angular_velocity;// second step update the last variable in the velocity buffer   
}

void OptiTrackFeedBackRigidBody::MovingWindowAveraging()
{

    /* Logic: Average the raw velocity measurement in the
    */
    double weight_linear = (double)1/linear_velocity_window;// the weight on each velocity to be summed up.
    double weight_angular = (double)1/angular_velocity_window;// the weight on each velocity to be summed up.
    // create a temporary variable to store the summed velocity and initialize it witht the 1st buffer value
    Vector3d velocitytemp;
    Vector3d angular_velocitytemp;
    velocitytemp = weight_linear*velocity_raw[0];
    angular_velocitytemp = weight_angular*angular_velocity_raw[0];

    for(int i = 1;i<linear_velocity_window;i++)// sum starts from the second buffer value
    {
        velocitytemp += weight_linear*velocity_raw[i];
    }
    for(int i = 1;i<angular_velocity_window;i++)// sum starts from the second buffer value
    {
        angular_velocitytemp += weight_angular*angular_velocity_raw[i];
    }
    // the filtered vlocity is just the weighted summed result
    velocity_filtered = velocitytemp;
    angular_velocity_filtered = angular_velocitytemp;
}

void OptiTrackFeedBackRigidBody::GetState(rigidbody_state& state)
{
    state.time_stamp = pose[1].t;
    state.Position = pose[1].Position;
    state.V_I = velocity_filtered;
    state.Omega_BI = angular_velocity_filtered;
    Hatmap(state.Omega_BI,state.Omega_Cross);
    state.R_IB = pose[1].R_IB;
    state.R_BI = pose[1].R_BI; 
    double euler_temp[3];
    GetEulerAngleFromQuaterion_NormalConvention(euler_temp);
    state.Euler(0) = euler_temp[0];// euler angle
    state.Euler(1) = euler_temp[1];// euler angle
    state.Euler(2) = euler_temp[2];// euler angle
    state.quaterion(0) = pose[1].q0;
    state.quaterion(1) = pose[1].q1;
    state.quaterion(2) = pose[1].q2;
    state.quaterion(3) = pose[1].q3;
}
void OptiTrackFeedBackRigidBody::GetRaWVelocity(Vector3d& linear_velocity,Vector3d& angular_velocity)
{
    linear_velocity = velocity_raw[linear_velocity_window-1];// return the filtered velocity
    angular_velocity = angular_velocity_raw[angular_velocity_window-1];// return the filtered velocity
}
void  OptiTrackFeedBackRigidBody::SetZeroVelocity()
{
    for(int i =0;i<linear_velocity_window;i++)
    {
        velocity_raw[i](0)=0;
        velocity_raw[i](1)=0;
        velocity_raw[i](2)=0;
    }
    for(int i =0;i<angular_velocity_window;i++)
    {
        angular_velocity_raw[i](0)=0;
        angular_velocity_raw[i](1)=0;
        angular_velocity_raw[i](2)=0;
    }
    velocity_filtered(0)=0;
    velocity_filtered(1)=0;
    velocity_filtered(2)=0;
    angular_velocity_filtered(0) =0;
    angular_velocity_filtered(1) =0;
    angular_velocity_filtered(2) =0;
}

void OptiTrackFeedBackRigidBody::RosWhileLoopRun()
{
    if(OptiTrackFlag==1)
    {// update the velocity only when there is OptiTrack feedback
        CalculateVelocityFromPose();
        FeedbackState=1;
    }else{
        // if the optitrack measurements no longer feedback, when the pose update will stop and we only return 0 velocity
        SetZeroVelocity();
        FeedbackState=0;
    }

    OptiTrackFlag = 0;// reset the feedback flag to 0
}
int OptiTrackFeedBackRigidBody::GetOptiTrackState()
{
    if (FeedbackState==1) {
      ROS_INFO("OptiTrack:Normal");
    }else{
      ROS_INFO("OptiTrack:No FeedBack");
    }
    ROS_INFO("Linear Velocity Filter Window Size is [%d]",linear_velocity_window);
    ROS_INFO("Angular Velocity Filter Window Size is [%d]",angular_velocity_window);
    return FeedbackState;
}
void OptiTrackFeedBackRigidBody::GetEulerAngleFromQuaterion_NormalConvention(double (&eulerangle)[3])
{


    /* Normal means the following https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    */
//    eulerangle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
//    eulerangle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
//    eulerangle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));


    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (pose[1].q0 * pose[1].q1 + pose[1].q2 * pose[1].q3);
    double cosr_cosp = +1.0 - 2.0 * (pose[1].q1 * pose[1].q1 +pose[1].q2 * pose[1].q2);
    double roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (pose[1].q0 * pose[1].q2 - pose[1].q3 * pose[1].q1);
    double pitch;
    if (fabs(sinp) >= 1)
           pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
           pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (pose[1].q0 * pose[1].q3 + pose[1].q1 * pose[1].q2);
    double cosy_cosp = +1.0 - 2.0 * (pose[1].q2 * pose[1].q2 + pose[1].q3 * pose[1].q3);
    double yaw = atan2(siny_cosp, cosy_cosp);
    //double yaw  = atan2(2.0 * (dronepose[1].q3 * dronepose[1].q0 + dronepose[1].q1 * dronepose[1].q2), -1.0 + 2.0 * (dronepose[1].q0 * dronepose[1].q0 + dronepose[1].q1 * dronepose[1].q1));
    eulerangle[0] = roll;
    eulerangle[1] = pitch;
    eulerangle[2] = yaw;

}

void OptiTrackFeedBackRigidBody::GetEulerAngleFromQuaterion_OptiTrackYUpConvention(double (&eulerangle)[3])
{

    // OptiTrack gives a quaternion with q2 and q3 flipped. (and sign flipped for q3)
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (pose[1].q0 * pose[1].q1 + pose[1].q3 * pose[1].q2);
    double cosr_cosp = +1.0 - 2.0 * (pose[1].q1 * pose[1].q1 +pose[1].q3 * pose[1].q3);
    double roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (pose[1].q0 * pose[1].q3 - pose[1].q2 * pose[1].q1);
    double pitch;
    if (fabs(sinp) >= 1)
           pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
           pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (pose[1].q0 * pose[1].q2 + pose[1].q1 * pose[1].q3);
    double cosy_cosp = +1.0 - 2.0 * (pose[1].q2 * pose[1].q2 + pose[1].q3 * pose[1].q3);
    double yaw = atan2(siny_cosp, cosy_cosp);
    eulerangle[0] = roll;
    eulerangle[1] = pitch;
    eulerangle[2] = yaw;

}

void OptiTrackFeedBackRigidBody::OptiTrackCallback(const geometry_msgs::PoseStamped& msg)
{
        // must use head information to distiguish the correct 
        OptiTrackdata = msg; // update optitrack data
        OptiTrackFlag = 1;// signal a new measurement feed has been revcieved.
}

OptiTrackFeedBackRigidBody::~OptiTrackFeedBackRigidBody()
{

}

void OptiTrackFeedBackRigidBody::Veemap(Matrix3d& cross_matrix, Vector3d& vector)
{
    vector(0) = -cross_matrix(1,2);
    vector(1) = cross_matrix(0,2);
    vector(2) = -cross_matrix(0,1);
}
void OptiTrackFeedBackRigidBody::Hatmap(Vector3d& vector, Matrix3d& cross_matrix)
{
    /*

    r^x = [0 -r3 r2;
           r3 0 -r1;
          -r2 r1 0]
    */
    
    cross_matrix(0,0) = 0.0;
    cross_matrix(0,1) = - vector(2);
    cross_matrix(0,2) = vector(1);

    cross_matrix(1,0) = vector(2);
    cross_matrix(1,1) = 0.0;
    cross_matrix(1,2) = - vector(0);

    cross_matrix(2,0) = - vector(1);
    cross_matrix(2,1) = vector(0);
    cross_matrix(2,2) = 0.0;

}