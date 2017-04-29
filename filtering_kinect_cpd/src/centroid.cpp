#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>

void centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  Eigen::Vector4f xyz_centroid;
  pcl::compute3DCentroid(*cloud, xyz_centroid); //重心の計算

  for(size_t i = 0; i < cloud->points.size(); i++)
  {
    cloud->points[i].x = cloud->points[i].x - xyz_centroid[0]; //X座標の移動
    cloud->points[i].y = cloud->points[i].y - xyz_centroid[1]; //Y座標の移動
    cloud->points[i].z = cloud->points[i].z - xyz_centroid[2]; //Z座標の移動
  }
}

int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile(argv[1], *cloud);

  centroid(cloud); //重心計算＋移動の関数に点群を渡す

  std::stringstream Filename;
  std::string name;
  name = argv[1];
  name.erase(name.length()-4);
  Filename << name << "_C.pcd";

  pcl::io::savePCDFileBinary(Filename.str(),*cloud);
  
}
