/**
 * This module calculates the Forward Kinematics of the da Vinci Research Kit Robot.
 */

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>
// using namespace Eigen;

// Some helpful constants
const double pi = 3.14;
const double degToRad = pi/180;
const double radToDeg = 180/pi;

// Calculate the Transformation Matrix
Eigen::Matrix4f tdh(double theta, double d, double a, double alpha)
{
  Eigen::Matrix4f dh_matrix;     // typedef Matrix<float, 4, 4> Matrix4f;

  dh_matrix << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta),
               sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
               0,           sin(alpha),             cos(alpha),            d,
               0,           0,                      0,                     1;

  // modified_dh_matrix << cos(theta),             -sin(theta),             0,              a,
  //              sin(theta)*cos(alpha),  cos(theta)*cos(alpha),   -sin(alpha),    -d*sin(alpha),
  //              sin(theta)*sin(alpha),  cos(theta)*sin(alpha),   cos(alpha),     d*cos(alpha),
  //              0,                      0,                       0,              1;
               
  return dh_matrix;
}


void dVRK_fk(const Eigen::Matrix<double, 7, 4>& psm_config)
{
  Eigen::Matrix4f T_psm;
  T_psm<< 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;

 for(const int& i : {0, 1, 2, 3, 4, 5, 6}) {
   T_psm *= tdh(psm_config(i, 0) * degToRad, psm_config(i, 1), psm_config(i, 2), psm_config(i, 3) * degToRad);
 }
  
 std::cout<<"PSM Transformation: \n"<<T_psm<<std::endl;

  // Eigen::Matrix3f rotation_mat = T_psm.block<3,3>(0,0);    // Extract a block of matrix of size 3*3 from index [0,0] of T
  // Eigen::Quaternionf quats(rotation_mat);
  // Eigen::Matrix<double, 7, 1> endEffectorPose_frame1 {
  //                                                     T_psm(0,3),
  //                                                     T_psm(1,3),
  //                                                     T_psm(2,3),
  //                                                     quats.x(),
  //                                                     quats.y(),
  //                                                     quats.z(),
  //                                                     quats.w()
  //                                                   };
  // std::cout << "[OUTPUT] Final Pose of the end effector: \n" << endEffectorPose_frame1 << std::endl;
}



int main(int argc, char **argv)
{
  Eigen::Matrix<double, 7, 4> psm_config;
  psm_config<<pi/2, 0, 0, pi/2,
             -pi/2, 0, 0, -pi/2,
              0, -0.4318, 0, pi/2,
              0, 0.4162, 0, 0,
              -pi/2, 0, 0, -pi/2,
              -pi/2, 0, 0.0091, -pi/2,
              0, 0.0102, 0, -pi/2;

  dVRK_fk(psm_config);
  return 0;
}
