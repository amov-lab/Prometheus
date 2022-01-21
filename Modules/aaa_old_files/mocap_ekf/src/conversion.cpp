#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

/**
Euler angle defination: zyx
Rotation matrix: C_body2ned
**/
Quaterniond euler2quaternion(Vector3d euler)
{
  double cr = cos(euler(0)/2);
  double sr = sin(euler(0)/2);
  double cp = cos(euler(1)/2);
  double sp = sin(euler(1)/2);
  double cy = cos(euler(2)/2);
  double sy = sin(euler(2)/2);
  Quaterniond q;
  q.w() = cr*cp*cy + sr*sp*sy;
  q.x() = sr*cp*cy - cr*sp*sy;
  q.y() = cr*sp*cy + sr*cp*sy;
  q.z() = cr*cp*sy - sr*sp*cy;
  return q; 
}


Matrix3d quaternion2mat(Quaterniond q)
{
  Matrix3d m;
  double a = q.w(), b = q.x(), c = q.y(), d = q.z();
  m << a*a + b*b - c*c - d*d, 2*(b*c - a*d), 2*(b*d+a*c),
       2*(b*c+a*d), a*a - b*b + c*c - d*d, 2*(c*d - a*b),
       2*(b*d - a*c), 2*(c*d+a*b), a*a-b*b - c*c + d*d;
  return m;
}

Vector3d mat2euler(Matrix3d m)
{ 
  double r = atan2(m(2, 1), m(2, 2));
  double p = asin(-m(2, 0));
  double y = atan2(m(1, 0), m(0, 0));
  Vector3d rpy(r, p, y);
  return rpy;
}

Quaterniond mat2quaternion(Matrix3d m)
{
  //return euler2quaternion(mat2euler(m));
  Quaterniond q;
  double a, b, c, d;
  a = sqrt(1 + m(0, 0) + m(1, 1) + m(2, 2))/2;
  b = (m(2, 1) - m(1, 2))/(4*a);
  c = (m(0, 2) - m(2, 0))/(4*a);
  d = (m(1, 0) - m(0, 1))/(4*a);
  q.w() = a; q.x() = b; q.y() = c; q.z() = d;
  return q;
}

//ZYX
Matrix3d euler2mat(Vector3d euler)
{
  double cr = cos(euler(0));
  double sr = sin(euler(0));
  double cp = cos(euler(1));
  double sp = sin(euler(1));
  double cy = cos(euler(2));
  double sy = sin(euler(2));
  Matrix3d m;
  m << cp*cy,  -cr*sy + sr*sp*cy, sr*sy + cr*sp*cy, 
       cp*sy,  cr*cy + sr*sp*sy,  -sr*cy + cr*sp*sy, 
       -sp,    sr*cp,             cr*cp;
  return m;
}

Vector3d quaternion2euler(Quaterniond q)
{
  return mat2euler(quaternion2mat(q));
}

double quaternion2yaw(Quaterniond q)
{
  double a = q.w(), b = q.x(), c = q.y(), d = q.z();
  double y = atan2(2*(b*c+a*d), (a*a + b*b - c*c - d*d));
  return y;
}

//Euler motion equation
//ZYX Euler angles velocity to body frame wx wy wz  w_phi,  w_theta,  w_psi     q: roll pitch yaw  (phi theta psi)
Matrix3d w_Euler2Body(Vector3d q)
{
    double cr = cos( q(0));
    double sr = sin( q(0));
    double cp = cos( q(1));
    double sp = sin( q(1));
    double cy = cos( q(2));
    double sy = sin( q(2));

    Matrix3d G;
    G << 1.,    0.,       -sp,
         0.,    cr,   cp * sr,
         0.,   -sr,   cp * cr;
    
    // Vector3d w(0,0,0);
    // w = G * qdot;
    return G;
} 
Matrix3d w_Body2Euler(Vector3d q)
{
    double cr = cos( q(0));
    double sr = sin( q(0));
    double cp = cos( q(1)) + 0.00000001;
    double sp = sin( q(1));
    double cy = cos( q(2));
    double sy = sin( q(2));

    Matrix3d G_inv;
    G_inv << 1., sp*sr/cp, cr*sp/cp,
             0.,       cr,      -sr,
             0.,    sr/cp,    cr/cp;

    // Vector3d qdot(0,0,0);
    // qdot = G_inv * w;
    return G_inv;
}
//ZXY Euler angles velocity to body frame wx wy wz  w_phi,  w_theta,  w_psi
// Vector3d Euler2Body(Vector3d q, Vector3d qdot)
// {
//     Vector3d w(0,0,0);
//     Matrix3d G;
//     G << cos(q.y),   0.,  -cos(q.x)*sin(q.y),
//          0.,         1.,   sin(q.x),
//          sin(q.y),   0.,   cos(q.x)*cos(q.y);
//     w = G * qdot;
//     return w;
// } 
// Vector3d Body2Euler(Vector3d q, Vector3d w)
// {
//     Vector3d qdot(0,0,0);
//     Matrix3d G_inv;
//     G_inv << cos(q.y),   0.,    sin(q.y),
//              (sin(q.x)*sin(q.y))/(cos(q.x)+0.00000001), 1., -(cos(q.y)*sin(q.x))/(cos(q.x)+0.00000001),
//              -sin(q.y)/(cos(q.x)+0.00000001), 0., cos(q.y)/(cos(q.x)+0.00000001);
//     qdot = G_inv * w;
//     return qdot;
// }
