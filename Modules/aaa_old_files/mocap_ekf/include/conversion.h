#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

Quaterniond euler2quaternion(Vector3d euler);
Matrix3d quaternion2mat(Quaterniond q);
Vector3d mat2euler(Matrix3d m);
Quaterniond mat2quaternion(Matrix3d m);
Matrix3d euler2mat(Vector3d euler);
Vector3d quaternion2euler(Quaterniond q);
double quaternion2yaw(Quaterniond q);
Matrix3d w_Euler2Body(Vector3d q);
Matrix3d w_Body2Euler(Vector3d q);
