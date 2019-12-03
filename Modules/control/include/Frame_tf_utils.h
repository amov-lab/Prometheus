/***************************************************************************************************************************
* Frame_tf_utils.h
*
* Author: Qyp
*
* Update Time: 2019.3.16
*
* Introduction:  utils functions for frame transform
*         1. Coby from mavros source code  /mavros/mavros/src/lib/ftf_frame_conversions.cpp
***************************************************************************************************************************/
#ifndef FRAME_TF_UTILS_H
#define FRAME_TF_UTILS_H

#include <Eigen/Eigen>
#include <math.h>

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Rotation Transform<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Transform the attitude representation from frame to frame.
// The proof for this transform can be seen
// http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/

//坐标系转换 transform_orientation_aircraft_to_baselink
Eigen::Quaterniond transform_orientation_aircraft_to_baselink(const Eigen::Quaterniond &q)
{
    // Static quaternion needed for rotating between aircraft and base_link frames
    // +PI rotation around X (Forward) axis transforms from Forward, Right, Down (aircraft)
    // Fto Forward, Left, Up (base_link) frames.
    Eigen::Vector3d aaa(M_PI, 0.0, 0.0);
    Eigen::Quaterniond AIRCRAFT_BASELINK_Q = quaternion_from_rpy(aaa);
    return q * AIRCRAFT_BASELINK_Q;
}

//坐标系转换 transform_orientation_baselink_to_aircraft
Eigen::Quaterniond transform_orientation_baselink_to_aircraft(const Eigen::Quaterniond &q)
{
    // AIRCRAFT_BASELINK_Q is the Static quaternion needed for rotating between aircraft and base_link frames
    // +PI rotation around X (Forward) axis transforms from Forward, Right, Down (aircraft)
    // Fto Forward, Left, Up (base_link) frames.
    Eigen::Vector3d aaa(M_PI, 0.0, 0.0);
    Eigen::Quaterniond AIRCRAFT_BASELINK_Q = quaternion_from_rpy(aaa);
    return q * AIRCRAFT_BASELINK_Q;
}

//坐标系转换 transform_orientation_ned_to_enu
Eigen::Quaterniond transform_orientation_ned_to_enu(const Eigen::Quaterniond &q)
{
    // NED_ENU_Q is the Static quaternion needed for rotating between ENU and NED frames
    // NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
    // ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
    Eigen::Vector3d bbb(M_PI, 0.0, M_PI_2);
    Eigen::Quaterniond NED_ENU_Q = quaternion_from_rpy(bbb);

    return NED_ENU_Q * q;
}
//坐标系转换 transform_orientation_enu_to_ned
Eigen::Quaterniond transform_orientation_enu_to_ned(const Eigen::Quaterniond &q)
{
    // NED_ENU_Q is the Static quaternion needed for rotating between ENU and NED frames
    // NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
    // ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
    Eigen::Vector3d bbb(M_PI, 0.0, M_PI_2);
    Eigen::Quaterniond NED_ENU_Q = quaternion_from_rpy(bbb);

    return NED_ENU_Q * q;
}



//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Translation Transform<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
/**
 * @brief Static vector needed for rotating between ENU and NED frames
 * +PI rotation around X (North) axis follwed by +PI/2 rotation about Z (Down)
 * gives the ENU frame.  Similarly, a +PI rotation about X (East) followed by
 * a +PI/2 roation about Z (Up) gives the NED frame.
 */
Eigen::Vector3d transform_enu_to_ned(const Eigen::Vector3d &vec)
{
    Eigen::Vector3d bbb(M_PI, 0.0, M_PI_2);
    Eigen::Quaterniond NED_ENU_Q = quaternion_from_rpy(bbb);
    Eigen::Affine3d NED_ENU_AFFINE(NED_ENU_Q);

    return NED_ENU_AFFINE * vec;
}

Eigen::Vector3d transform_ned_to_enu(const Eigen::Vector3d &vec)
{
    Eigen::Vector3d bbb(M_PI, 0.0, M_PI_2);
    Eigen::Quaterniond NED_ENU_Q = quaternion_from_rpy(bbb);
    Eigen::Affine3d NED_ENU_AFFINE(NED_ENU_Q);

    return NED_ENU_AFFINE * vec;
}

#endif
