#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <Eigen/Geometry>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>

#ifndef IIWA_14_FK_FAST_HPP
#define IIWA_14_FK_FAST_HPP

namespace IIWA_14_FK_FAST
{
    const size_t IIWA_14_NUM_ACTIVE_JOINTS = 7;
    const size_t IIWA_14_NUM_LINKS = 8;

    const std::string IIWA_14_ACTIVE_JOINT_1_NAME = "iiwa_joint_1";
    const std::string IIWA_14_ACTIVE_JOINT_2_NAME = "iiwa_joint_2";
    const std::string IIWA_14_ACTIVE_JOINT_3_NAME = "iiwa_joint_3";
    const std::string IIWA_14_ACTIVE_JOINT_4_NAME = "iiwa_joint_4";
    const std::string IIWA_14_ACTIVE_JOINT_5_NAME = "iiwa_joint_5";
    const std::string IIWA_14_ACTIVE_JOINT_6_NAME = "iiwa_joint_6";
    const std::string IIWA_14_ACTIVE_JOINT_7_NAME = "iiwa_joint_7";

    const std::string IIWA_14_LINK_1_NAME = "iiwa_link_0";
    const std::string IIWA_14_LINK_2_NAME = "iiwa_link_1";
    const std::string IIWA_14_LINK_3_NAME = "iiwa_link_2";
    const std::string IIWA_14_LINK_4_NAME = "iiwa_link_3";
    const std::string IIWA_14_LINK_5_NAME = "iiwa_link_4";
    const std::string IIWA_14_LINK_6_NAME = "iiwa_link_5";
    const std::string IIWA_14_LINK_7_NAME = "iiwa_link_6";
    const std::string IIWA_14_LINK_8_NAME = "iiwa_link_7";

    inline Eigen::Isometry3d Get_link_0_joint_1_LinkJointTransform(const double joint_val)
    {
        const Eigen::Translation3d pre_joint_translation(0.0, 0.0, 0.1575);
        const Eigen::Quaterniond pre_joint_rotation = EigenHelpers::QuaternionFromUrdfRPY(0.0, 0.0, 0.0);
        const Eigen::Isometry3d pre_joint_transform = pre_joint_translation * pre_joint_rotation;
        const Eigen::Translation3d joint_translation(0.0, 0.0, 0.0);
        const Eigen::Quaterniond joint_rotation(Eigen::AngleAxisd(joint_val, Eigen::Vector3d::UnitZ()));
        const Eigen::Isometry3d joint_transform = joint_translation * joint_rotation;
        return (pre_joint_transform * joint_transform);
    }

    inline Eigen::Isometry3d Get_link_1_joint_2_LinkJointTransform(const double joint_val)
    {
        const Eigen::Translation3d pre_joint_translation(0.0, 0.0, 0.2025);
        const Eigen::Quaterniond pre_joint_rotation = EigenHelpers::QuaternionFromUrdfRPY(M_PI_2, 0.0, M_PI);
        const Eigen::Isometry3d pre_joint_transform = pre_joint_translation * pre_joint_rotation;
        const Eigen::Translation3d joint_translation(0.0, 0.0, 0.0);
        const Eigen::Quaterniond joint_rotation(Eigen::AngleAxisd(joint_val, Eigen::Vector3d::UnitZ()));
        const Eigen::Isometry3d joint_transform = joint_translation * joint_rotation;
        return (pre_joint_transform * joint_transform);
    }

    inline Eigen::Isometry3d Get_link_2_joint_3_LinkJointTransform(const double joint_val)
    {
        const Eigen::Translation3d pre_joint_translation(0.0, 0.2045, 0.0);
        const Eigen::Quaterniond pre_joint_rotation = EigenHelpers::QuaternionFromUrdfRPY(M_PI_2, 0.0, M_PI);
        const Eigen::Isometry3d pre_joint_transform = pre_joint_translation * pre_joint_rotation;
        const Eigen::Translation3d joint_translation(0.0, 0.0, 0.0);
        const Eigen::Quaterniond joint_rotation(Eigen::AngleAxisd(joint_val, Eigen::Vector3d::UnitZ()));
        const Eigen::Isometry3d joint_transform = joint_translation * joint_rotation;
        return (pre_joint_transform * joint_transform);
    }

    inline Eigen::Isometry3d Get_link_3_joint_4_LinkJointTransform(const double joint_val)
    {
        const Eigen::Translation3d pre_joint_translation(0.0, 0.0, 0.2155);
        const Eigen::Quaterniond pre_joint_rotation = EigenHelpers::QuaternionFromUrdfRPY(M_PI_2, 0.0, 0.0);
        const Eigen::Isometry3d pre_joint_transform = pre_joint_translation * pre_joint_rotation;
        const Eigen::Translation3d joint_translation(0.0, 0.0, 0.0);
        const Eigen::Quaterniond joint_rotation(Eigen::AngleAxisd(joint_val, Eigen::Vector3d::UnitZ()));
        const Eigen::Isometry3d joint_transform = joint_translation * joint_rotation;
        return (pre_joint_transform * joint_transform);
    }

    inline Eigen::Isometry3d Get_link_4_joint_5_LinkJointTransform(const double joint_val)
    {
        const Eigen::Translation3d pre_joint_translation(0.0, 0.1845, 0.0);
        const Eigen::Quaterniond pre_joint_rotation = EigenHelpers::QuaternionFromUrdfRPY(-M_PI_2, M_PI, 0.0);
        const Eigen::Isometry3d pre_joint_transform = pre_joint_translation * pre_joint_rotation;
        const Eigen::Translation3d joint_translation(0.0, 0.0, 0.0);
        const Eigen::Quaterniond joint_rotation(Eigen::AngleAxisd(joint_val, Eigen::Vector3d::UnitZ()));
        const Eigen::Isometry3d joint_transform = joint_translation * joint_rotation;
        return (pre_joint_transform * joint_transform);
    }

    inline Eigen::Isometry3d Get_link_5_joint_6_LinkJointTransform(const double joint_val)
    {
        const Eigen::Translation3d pre_joint_translation(0.0, 0.0, 0.2155);
        const Eigen::Quaterniond pre_joint_rotation = EigenHelpers::QuaternionFromUrdfRPY(M_PI_2, 0.0, 0.0);
        const Eigen::Isometry3d pre_joint_transform = pre_joint_translation * pre_joint_rotation;
        const Eigen::Translation3d joint_translation(0.0, 0.0, 0.0);
        const Eigen::Quaterniond joint_rotation(Eigen::AngleAxisd(joint_val, Eigen::Vector3d::UnitZ()));
        const Eigen::Isometry3d joint_transform = joint_translation * joint_rotation;
        return (pre_joint_transform * joint_transform);
    }

    inline Eigen::Isometry3d Get_link_6_joint_7_LinkJointTransform(const double joint_val)
    {
        const Eigen::Translation3d pre_joint_translation(0.0, 0.081, 0.0);
        const Eigen::Quaterniond pre_joint_rotation = EigenHelpers::QuaternionFromUrdfRPY(-M_PI_2, M_PI, 0.0);
        const Eigen::Isometry3d pre_joint_transform = pre_joint_translation * pre_joint_rotation;
        const Eigen::Translation3d joint_translation(0.0, 0.0, 0.0);
        const Eigen::Quaterniond joint_rotation(Eigen::AngleAxisd(joint_val, Eigen::Vector3d::UnitZ()));
        const Eigen::Isometry3d joint_transform = joint_translation * joint_rotation;
        return (pre_joint_transform * joint_transform);
    }

    inline EigenHelpers::VectorIsometry3d GetLinkTransforms(const std::vector<double>& configuration, const Eigen::Isometry3d& base_transform=Eigen::Isometry3d::Identity())
    {
        assert(configuration.size() == IIWA_14_NUM_ACTIVE_JOINTS);
        EigenHelpers::VectorIsometry3d link_transforms(IIWA_14_NUM_LINKS);
        link_transforms[0] = base_transform;
        link_transforms[1] = link_transforms[0] * Get_link_0_joint_1_LinkJointTransform(configuration[0]);
        link_transforms[2] = link_transforms[1] * Get_link_1_joint_2_LinkJointTransform(configuration[1]);
        link_transforms[3] = link_transforms[2] * Get_link_2_joint_3_LinkJointTransform(configuration[2]);
        link_transforms[4] = link_transforms[3] * Get_link_3_joint_4_LinkJointTransform(configuration[3]);
        link_transforms[5] = link_transforms[4] * Get_link_4_joint_5_LinkJointTransform(configuration[4]);
        link_transforms[6] = link_transforms[5] * Get_link_5_joint_6_LinkJointTransform(configuration[5]);
        link_transforms[7] = link_transforms[6] * Get_link_6_joint_7_LinkJointTransform(configuration[6]);
        return link_transforms;
    }

    inline EigenHelpers::VectorIsometry3d GetLinkTransforms(const std::map<std::string, double>& configuration, const Eigen::Isometry3d& base_transform=Eigen::Isometry3d::Identity())
    {
        std::vector<double> configuration_vector(IIWA_14_NUM_ACTIVE_JOINTS);
        configuration_vector[0] = arc_helpers::RetrieveOrDefault(configuration, IIWA_14_ACTIVE_JOINT_1_NAME, 0.0);
        configuration_vector[1] = arc_helpers::RetrieveOrDefault(configuration, IIWA_14_ACTIVE_JOINT_2_NAME, 0.0);
        configuration_vector[2] = arc_helpers::RetrieveOrDefault(configuration, IIWA_14_ACTIVE_JOINT_3_NAME, 0.0);
        configuration_vector[3] = arc_helpers::RetrieveOrDefault(configuration, IIWA_14_ACTIVE_JOINT_4_NAME, 0.0);
        configuration_vector[4] = arc_helpers::RetrieveOrDefault(configuration, IIWA_14_ACTIVE_JOINT_5_NAME, 0.0);
        configuration_vector[5] = arc_helpers::RetrieveOrDefault(configuration, IIWA_14_ACTIVE_JOINT_6_NAME, 0.0);
        configuration_vector[6] = arc_helpers::RetrieveOrDefault(configuration, IIWA_14_ACTIVE_JOINT_7_NAME, 0.0);
        return GetLinkTransforms(configuration_vector, base_transform);
    }

    inline EigenHelpers::MapStringIsometry3d GetLinkTransformsMap(const std::vector<double>& configuration, const Eigen::Isometry3d& base_transform=Eigen::Isometry3d::Identity())
    {
        const EigenHelpers::VectorIsometry3d link_transforms = GetLinkTransforms(configuration, base_transform);
        EigenHelpers::MapStringIsometry3d link_transforms_map;
        link_transforms_map[IIWA_14_LINK_1_NAME] = link_transforms[0];
        link_transforms_map[IIWA_14_LINK_2_NAME] = link_transforms[1];
        link_transforms_map[IIWA_14_LINK_3_NAME] = link_transforms[2];
        link_transforms_map[IIWA_14_LINK_4_NAME] = link_transforms[3];
        link_transforms_map[IIWA_14_LINK_5_NAME] = link_transforms[4];
        link_transforms_map[IIWA_14_LINK_6_NAME] = link_transforms[5];
        link_transforms_map[IIWA_14_LINK_7_NAME] = link_transforms[6];
        link_transforms_map[IIWA_14_LINK_8_NAME] = link_transforms[7];
        return link_transforms_map;
    }

    inline EigenHelpers::MapStringIsometry3d GetLinkTransformsMap(const std::map<std::string, double>& configuration, const Eigen::Isometry3d& base_transform=Eigen::Isometry3d::Identity())
    {
        const EigenHelpers::VectorIsometry3d link_transforms = GetLinkTransforms(configuration, base_transform);
        EigenHelpers::MapStringIsometry3d link_transforms_map;
        link_transforms_map[IIWA_14_LINK_1_NAME] = link_transforms[0];
        link_transforms_map[IIWA_14_LINK_2_NAME] = link_transforms[1];
        link_transforms_map[IIWA_14_LINK_3_NAME] = link_transforms[2];
        link_transforms_map[IIWA_14_LINK_4_NAME] = link_transforms[3];
        link_transforms_map[IIWA_14_LINK_5_NAME] = link_transforms[4];
        link_transforms_map[IIWA_14_LINK_6_NAME] = link_transforms[5];
        link_transforms_map[IIWA_14_LINK_7_NAME] = link_transforms[6];
        link_transforms_map[IIWA_14_LINK_8_NAME] = link_transforms[7];
        return link_transforms_map;
    }
}

#endif // IIWA_14_FK_FAST_HPP
