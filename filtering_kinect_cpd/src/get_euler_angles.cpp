#include <iostream>
#include <string>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>


int main(int argc, char** argv)
{
  float roll, pitch, yaw;

  // Eigen::Affine3f mat_r;
  Eigen::Matrix3f mat_r;
  // mat_r (0, 0) = -0.213718;
  // mat_r (0, 1) = 0.473845;
  // mat_r (0, 2) = -0.854318;
  // mat_r (1, 0) = -0.421344;
  // mat_r (1, 1) = 0.744281;
  // mat_r (1, 2) = 0.51822;
  // mat_r (2, 0) = 0.881389;
  // mat_r (2, 1) = 0.470709;
  // mat_r (2, 2) = 0.0405874;

  mat_r (0, 0) = -0.6397590;
  mat_r (0, 1) = 0.5559349;
  mat_r (0, 2) = -0.5307508;
  mat_r (1, 0) = 0.6164725;
  mat_r (1, 1) = -0.0412794;
  mat_r (1, 2) = -0.7863256;
  mat_r (2, 0) = -0.4590450;
  mat_r (2, 1) = -0.8302264;
  mat_r (2, 2) = 0.9295245;

  // cout << "\n Input rotation :\n\n" << mat_r << endl;

  Eigen::Matrix3f matrix;
  matrix = mat_r.matrix().block(0,0,3,3);

  Eigen::Vector3f eu = mat_r.eulerAngles(2,1,0);

  cout << "\n Get euler angles \n\n" << eu << endl;

  // pcl::getEulerAngles(mat_r, roll, pitch, yaw);
  // cout << "\n The output Euler angles (using getEulerAngles function) are :\n\n" << "roll : " << roll << " ,pitch : " << pitch << " ,yaw : " << yaw << endl;

  return 0;
}
