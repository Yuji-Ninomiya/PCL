#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>


int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rotated (new pcl::PointCloud<pcl::PointXYZ>);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  pcl::io::loadPCDFile(argv[1], *cloud);

  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

  double phi = -0.0362 ; // roll ロール
  double theta = -0.4907 ; // pich ピッチ
  double psi = 5.8214 ; // yaw ヨー

  // 回転行列：R
  transformation_matrix (0, 0) = cos(phi) * cos(theta);
  transformation_matrix (0, 1) = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);
  transformation_matrix (0, 2) = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);
  transformation_matrix (1, 0) = sin(phi) * cos(theta);
  transformation_matrix (1, 1) = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
  transformation_matrix (1, 2) = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
  transformation_matrix (2, 0) = -sin(theta);
  transformation_matrix (2, 1) = cos(theta) * sin(psi);
  transformation_matrix (2, 2) = cos(theta) * cos(psi);

  // A translation on X,Y,Z axis：t
  transformation_matrix (0, 3) = 0.8466;
  transformation_matrix (1, 3) = -0.7767;
  transformation_matrix (2, 3) = -0.6661;

  pcl::transformPointCloud (*cloud, *cloud_rotated, transformation_matrix);

  std::stringstream Filename;
  std::string name;
  name = argv[1];
  name.erase(name.length()-4);
  Filename << name << "_r.pcd";

  pcl::io::savePCDFileBinary(Filename.str(),*cloud_rotated);
  std::cerr << "Cloud after filtering: " << std::endl;

  return(0);
}
