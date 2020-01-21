#include <Eigen/Geometry>
#include <Eigen/Jacobi>
#include <Eigen/SVD>
#include <stdio.h>
#include <iostream>
#include <arc_utilities/pretty_print.hpp>
#include <functional>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Wrench.h>
#include <arc_utilities/eigen_helpers.hpp>

#ifndef EIGEN_HELPERS_CONVERSIONS_HPP
#define EIGEN_HELPERS_CONVERSIONS_HPP

namespace EigenHelpersConversions
{
    inline Eigen::Vector3d GeometryPointToEigenVector3d(const geometry_msgs::Point& point)
    {
        Eigen::Vector3d eigen_point(point.x, point.y, point.z);
        return eigen_point;
    }

    inline geometry_msgs::Point EigenVector3dToGeometryPoint(const Eigen::Vector3d& point)
    {
        geometry_msgs::Point geom_point;
        geom_point.x = point.x();
        geom_point.y = point.y();
        geom_point.z = point.z();
        return geom_point;
    }

    inline Eigen::Vector4d GeometryPointToEigenVector4d(const geometry_msgs::Point& point)
    {
        Eigen::Vector4d eigen_point(point.x, point.y, point.z, 1.0);
        return eigen_point;
    }

    inline geometry_msgs::Point EigenVector4dToGeometryPoint(const Eigen::Vector4d& point)
    {
        geometry_msgs::Point geom_point;
        geom_point.x = point(0);
        geom_point.y = point(1);
        geom_point.z = point(2);
        return geom_point;
    }

    inline geometry_msgs::PointStamped EigenVector3dToGeometryPointStamped(const Eigen::Vector3d& point, const std::string& frame_id)
    {
        geometry_msgs::PointStamped point_stamped;
        point_stamped.header.frame_id = frame_id;
        point_stamped.point = EigenVector3dToGeometryPoint(point);
        return point_stamped;
    }

    inline Eigen::Vector3d GeometryVector3ToEigenVector3d(const geometry_msgs::Vector3& vector)
    {
        Eigen::Vector3d eigen_vector(vector.x, vector.y, vector.z);
        return eigen_vector;
    }

    inline geometry_msgs::Vector3 EigenVector3dToGeometryVector3(const Eigen::Vector3d& vector)
    {
        geometry_msgs::Vector3 geom_vector;
        geom_vector.x = vector.x();
        geom_vector.y = vector.y();
        geom_vector.z = vector.z();
        return geom_vector;
    }

    inline Eigen::Quaterniond GeometryQuaternionToEigenQuaterniond(const geometry_msgs::Quaternion& quat)
    {
        Eigen::Quaterniond eigen_quaternion(quat.w, quat.x, quat.y, quat.z);
        return eigen_quaternion;
    }

    inline geometry_msgs::Quaternion EigenQuaterniondToGeometryQuaternion(const Eigen::Quaterniond& quat)
    {
        geometry_msgs::Quaternion geom_quaternion;
        geom_quaternion.w = quat.w();
        geom_quaternion.x = quat.x();
        geom_quaternion.y = quat.y();
        geom_quaternion.z = quat.z();
        return geom_quaternion;
    }

    inline Eigen::Isometry3d GeometryPoseToEigenIsometry3d(const geometry_msgs::Pose& pose)
    {
        const Eigen::Translation3d trans(pose.position.x, pose.position.y, pose.position.z);
        const Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        const Eigen::Isometry3d eigen_pose = trans * quat;
        return eigen_pose;
    }

    inline geometry_msgs::Pose EigenIsometry3dToGeometryPose(const Eigen::Isometry3d& transform)
    {
        const Eigen::Vector3d trans = transform.translation();
        const Eigen::Quaterniond quat(transform.rotation());
        geometry_msgs::Pose geom_pose;
        geom_pose.position.x = trans.x();
        geom_pose.position.y = trans.y();
        geom_pose.position.z = trans.z();
        geom_pose.orientation.w = quat.w();
        geom_pose.orientation.x = quat.x();
        geom_pose.orientation.y = quat.y();
        geom_pose.orientation.z = quat.z();
        return geom_pose;
    }

    inline geometry_msgs::PoseStamped EigenIsometry3dToGeometryPoseStamped(const Eigen::Isometry3d& transform, const std::string& frame_id)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = frame_id;
        pose_stamped.pose = EigenIsometry3dToGeometryPose(transform);
        return pose_stamped;
    }

    inline Eigen::Isometry3d GeometryTransformToEigenIsometry3d(const geometry_msgs::Transform& transform)
    {
        const Eigen::Translation3d trans(transform.translation.x, transform.translation.y, transform.translation.z);
        const Eigen::Quaterniond quat(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
        const Eigen::Isometry3d eigen_transform = trans * quat;
        return eigen_transform;
    }

    inline geometry_msgs::Transform EigenIsometry3dToGeometryTransform(const Eigen::Isometry3d& transform)
    {
        const Eigen::Vector3d trans = transform.translation();
        const Eigen::Quaterniond quat(transform.rotation());
        geometry_msgs::Transform geom_transform;
        geom_transform.translation.x = trans.x();
        geom_transform.translation.y = trans.y();
        geom_transform.translation.z = trans.z();
        geom_transform.rotation.w = quat.w();
        geom_transform.rotation.x = quat.x();
        geom_transform.rotation.y = quat.y();
        geom_transform.rotation.z = quat.z();
        return geom_transform;
    }

    inline Eigen::Matrix3Xd VectorGeometryPointToEigenMatrix3Xd(const std::vector<geometry_msgs::Point>& vector_geom)
    {
        Eigen::Matrix3Xd eigen_matrix = Eigen::MatrixXd(3, vector_geom.size());
        for (size_t idx = 0; idx < vector_geom.size(); idx++)
        {
            eigen_matrix.block<3,1>(0, idx) = GeometryPointToEigenVector3d(vector_geom[idx]);
        }
        return eigen_matrix;
    }

    inline std::vector<geometry_msgs::Point> EigenMatrix3XdToVectorGeometryPoint(const Eigen::Matrix3Xd& eigen_matrix)
    {
        std::vector<geometry_msgs::Point> vector_geom(eigen_matrix.cols());
        for (size_t idx = 0; idx < vector_geom.size(); idx++)
        {
            vector_geom[idx] = EigenVector3dToGeometryPoint(eigen_matrix.block<3,1>(0,idx));
        }
        return vector_geom;
    }

    inline std::vector<geometry_msgs::Point> VectorEigenVector3dToVectorGeometryPoint(const EigenHelpers::VectorVector3d& vector_eigen)
    {
        std::vector<geometry_msgs::Point> vector_geom(vector_eigen.size());
        for (size_t idx = 0; idx < vector_eigen.size(); idx++)
        {
            vector_geom[idx] = EigenVector3dToGeometryPoint(vector_eigen[idx]);
        }
        return vector_geom;
    }

    inline Eigen::Matrix3Xd VectorEigenVector3dToEigenMatrix3Xd(const EigenHelpers::VectorVector3d& vector_eigen)
    {
        Eigen::Matrix3Xd eigen_matrix(3, vector_eigen.size());
        for (size_t idx = 0; idx < vector_eigen.size(); idx++)
        {
            eigen_matrix.col(idx) = vector_eigen[idx];
        }
        return eigen_matrix;
    }

    inline EigenHelpers::VectorVector3d VectorGeometryPointToVectorEigenVector3d(const std::vector<geometry_msgs::Point>& vector_geom)
    {
        EigenHelpers::VectorVector3d vector_eigen(vector_geom.size());
        for (size_t idx = 0; idx < vector_geom.size(); idx++)
        {
            vector_eigen[idx] = GeometryPointToEigenVector3d(vector_geom[idx]);
        }
        return vector_eigen;
    }

    inline EigenHelpers::VectorVector3d VectorGeometryVector3ToEigenVector3d(const std::vector<geometry_msgs::Vector3>& vector_geom)
    {
        EigenHelpers::VectorVector3d vector_eigen(vector_geom.size());
        for (size_t idx = 0; idx < vector_geom.size(); idx++)
        {
            vector_eigen[idx] = GeometryVector3ToEigenVector3d(vector_geom[idx]);
        }
        return vector_eigen;
    }

    inline EigenHelpers::VectorIsometry3d VectorGeometryPoseToVectorIsometry3d(const std::vector<geometry_msgs::Pose>& vector_geom)
    {
        EigenHelpers::VectorIsometry3d vector_eigen(vector_geom.size());
        for (size_t idx = 0; idx < vector_geom.size(); idx++)
        {
            vector_eigen[idx] = GeometryPoseToEigenIsometry3d(vector_geom[idx]);
        }
        return vector_eigen;
    }

    inline EigenHelpers::VectorIsometry3d VectorGeometryPoseToVectorIsometry3d(const std::vector<geometry_msgs::Transform>& vector_geom)
    {
        EigenHelpers::VectorIsometry3d vector_eigen(vector_geom.size());
        for (size_t idx = 0; idx < vector_geom.size(); idx++)
        {
            vector_eigen[idx] = GeometryTransformToEigenIsometry3d(vector_geom[idx]);
        }
        return vector_eigen;
    }

    inline std::vector<geometry_msgs::Pose> VectorIsometry3dToVectorGeometryPose(const EigenHelpers::VectorIsometry3d& vector_eigen)
    {
        std::vector<geometry_msgs::Pose> vector_geom(vector_eigen.size());
        for (size_t idx = 0; idx < vector_eigen.size(); idx++)
        {
            vector_geom[idx] = EigenIsometry3dToGeometryPose(vector_eigen[idx]);
        }
        return vector_geom;
    }

    inline std::vector<geometry_msgs::Transform> VectorIsometry3dToVectorGeometryTransform(const EigenHelpers::VectorIsometry3d& vector_eigen)
    {
        std::vector<geometry_msgs::Transform> vector_geom(vector_eigen.size());
        for (size_t idx = 0; idx < vector_eigen.size(); idx++)
        {
            vector_geom[idx] = EigenIsometry3dToGeometryTransform(vector_eigen[idx]);
        }
        return vector_geom;
    }

    // Convert wrench (force and torque) ROS message to Eigen typed data
    inline std::pair<Eigen::Vector3d, Eigen::Vector3d> GeometryWrenchToEigenPairVector(const geometry_msgs::Wrench& wrench)
    {
        const Eigen::Vector3d eigen_force(wrench.force.x, wrench.force.y, wrench.force.z);
        const Eigen::Vector3d eigen_torque(wrench.torque.x, wrench.torque.y, wrench.torque.z);
        const std::pair<Eigen::Vector3d, Eigen::Vector3d> eigen_wrench = std::make_pair(eigen_force, eigen_torque);
        return eigen_wrench;
    }

    template<typename data_type, int LENGTH>
    inline Eigen::Matrix<data_type, Eigen::Dynamic, 1> VectorEigenVectorToEigenVectorX(const std::vector<Eigen::Matrix<data_type, LENGTH, 1>>& vector_eigen_input)
    {
        assert(vector_eigen_input.size() > 0);

        Eigen::Matrix<data_type, Eigen::Dynamic, 1> eigen_result;
        eigen_result.resize((ssize_t)vector_eigen_input.size() * vector_eigen_input[0].rows());

        for (size_t idx = 0; idx < vector_eigen_input.size(); idx++)
        {
            eigen_result.segment((ssize_t)idx * LENGTH, LENGTH) = vector_eigen_input[idx];
        }

        return eigen_result;
    }

    template<typename data_type, int LENGTH>
    inline std::vector<Eigen::Matrix<data_type, LENGTH, 1>, Eigen::aligned_allocator<Eigen::Matrix<data_type, LENGTH, 1>>> EigenVectorXToVectorEigenVector(const Eigen::Matrix<data_type, Eigen::Dynamic, 1>& eigen_input)
    {
        assert(eigen_input.rows() % LENGTH == 0);
        size_t num_vectors = eigen_input.rows() / LENGTH;

        std::vector<Eigen::Matrix<data_type, LENGTH, 1>, Eigen::aligned_allocator<Eigen::Matrix<data_type, LENGTH, 1>>> vector_eigen_output(num_vectors);

        for (size_t idx = 0; idx < num_vectors; idx++)
        {
            vector_eigen_output[idx] = eigen_input.segment<LENGTH>((ssize_t)idx * LENGTH);
        }

        return vector_eigen_output;
    }
}

#endif // EIGEN_HELPERS_CONVERSIONS_HPP
