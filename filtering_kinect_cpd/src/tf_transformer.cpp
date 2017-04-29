#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>


void
Make_rotationMatrix (Eigen::Matrix4d & mat_t, double x, double y, double z, double phi, double theta, double psi)
{
  // 回転行列：R
  mat_t (0, 0) = cos(phi) * cos(theta);
  mat_t (0, 1) = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);
  mat_t (0, 2) = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);
  mat_t (1, 0) = sin(phi) * cos(theta);
  mat_t (1, 1) = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
  mat_t (1, 2) = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
  mat_t (2, 0) = -sin(theta);
  mat_t (2, 1) = cos(theta) * sin(psi);
  mat_t (2, 2) = cos(theta) * cos(psi);

  // 並進ベクトル：t
  mat_t (0, 3) = x;
  mat_t (1, 3) = y;
  mat_t (2, 3) = z;

  mat_t (3, 0) = 0;
  mat_t (3, 1) = 0;
  mat_t (3, 2) = 0;
  mat_t (3, 3) = 1;
}

void
Initial (Eigen::Matrix4d & mat_init) // 実験結果から得られた変換行列 axis変換なし
{
  // 回転行列：R
  mat_init (0, 0) = 0;//0.99998;
  mat_init (0, 1) = 0;//0.00082;
  mat_init (0, 2) = 0;//0.00595;
  mat_init (1, 0) = 0;//-0.00061;
  mat_init (1, 1) = 0;//0.99936;
  mat_init (1, 2) = 0;//-0.03579;
  mat_init (2, 0) = 0;//-0.00597;
  mat_init (2, 1) = 0;//0.03579;
  mat_init (2, 2) = 0;//0.99934;

  // 並進ベクトル：t
  mat_init (0, 3) = 0.8794;
  mat_init (1, 3) = -0.6445;
  mat_init (2, 3) = 0.5804;

  mat_init (3, 0) = 0;
  mat_init (3, 1) = 0;
  mat_init (3, 2) = 0;
  mat_init (3, 3) = 1;
}



int main(int argc, char** argv)
{
  Eigen::Matrix4d mat_a, mat_b, transformation_kinect, kinect_third, initial;

  //Make_rotationMatrix(mat_a, -2, 0, 0.9, 0, M_PI/6, 0);
  Make_rotationMatrix(mat_a, 0, 0, 0.5, 0, 0, 0);
  //cout << "\n matrix a \n\n" << mat_a << endl;
  // kinect_third を kinect 座標系へ変換
  //Make_rotationMatrix(mat_b, 0, 0, 0, -M_PI/2, M_PI/2, 0);
  //cout << "\n matrix b \n\n" << mat_b << endl;
  // ワールド座標系から kinect 座標系への変換
  //transformation_kinect = mat_a * mat_b;
  //cout << "\n transformation_kinect \n\n" << transformation_kinect << endl;
  // 逆変換して表示
  //kinect_third = transformation_kinect.inverse();
  //kinect_third = mat_a.inverse();
  //cout << "\n kinect_third \n\n" << kinect_third << endl;

  //Initial(initial);
  
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile(argv[1], *cloud_in);

  pcl::transformPointCloud (*cloud_in, *cloud_out, mat_a);

  // //tf::TransformListener tflistener;
  // tf::StampedTransform kinect_third;
  // tf_.lookupTransform("/base_link", "/kinect_third", ros::Time(0), kinect_third);

  // //tflistener.transformPointCloud();

  // pcl_ros::transformPointCloud(cloud_in, cloud_out, kinect_third);

  pcl::io::savePCDFileBinary("translation.pcd", *cloud_out);

  return(0);

}
