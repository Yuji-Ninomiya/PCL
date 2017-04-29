#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>


pcl::PointCloud<pcl::PointXYZ> rotation_x(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double theta) //rotate point cloud by X axis
{
   Eigen::Matrix4f rot_x;
    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    cloud_out = *cloud;
    double theta_x = theta * 3.1415926 / 180.0;//角度の変換
    rot_x(0,0) = 1.0;
    rot_x(1,0) = 0.0;
    rot_x(2,0) = 0.0;
    rot_x(3,0) = 0.0;
    rot_x(0,1) = 0.0;
    rot_x(1,1) = cos(theta_x);
    rot_x(2,1) = -sin(theta_x);
    rot_x(3,1) = 0.0;
    rot_x(0,2) = 0.0;
    rot_x(1,2) = sin(theta_x);
    rot_x(2,2) = cos(theta_x);
    rot_x(3,2) = 0.0;
    rot_x(0,3) = 0.0;
    rot_x(1,3) = 0.0;
    rot_x(2,3) = 0.0;
    rot_x(3,3) = 1.0;

    for(size_t i = 0; i < cloud->points.size(); ++i){
        cloud_out.points[i].x = rot_x(0,0) * cloud->points[i].x + rot_x(0,1) * cloud->points[i].y + rot_x(0,2) * cloud->points[i].z + rot_x(0,3) * 1;
        cloud_out.points[i].y = rot_x(1,0) * cloud->points[i].x + rot_x(1,1) * cloud->points[i].y + rot_x(1,2) * cloud->points[i].z + rot_x(1,3) * 1;
        cloud_out.points[i].z = rot_x(2,0) * cloud->points[i].x + rot_x(2,1) * cloud->points[i].y + rot_x(2,2) * cloud->points[i].z + rot_x(2,3) * 1;
    }

    return cloud_out;
}


pcl::PointCloud<pcl::PointXYZ> rotation_y(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double theta) //rotate point cloud by Y axis
{
   Eigen::Matrix4f rot_y;
    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    cloud_out = *cloud;
    double theta_y = theta * 3.1415926 / 180.0;//角度の変換
    rot_y(0,0) = cos(theta_y);
    rot_y(1,0) = 0.0;
    rot_y(2,0) = sin(theta_y);
    rot_y(3,0) = 0.0;
    rot_y(0,1) = 0.0;
    rot_y(1,1) = 1.0;
    rot_y(2,1) = 0.0;
    rot_y(3,1) = 0.0;
    rot_y(0,2) = -sin(theta_y);
    rot_y(1,2) = 0.0;
    rot_y(2,2) = cos(theta_y);
    rot_y(3,2) = 0.0;
    rot_y(0,3) = 0.0;
    rot_y(1,3) = 0.0;
    rot_y(2,3) = 0.0;
    rot_y(3,3) = 1.0;

    for(size_t i = 0; i < cloud->points.size(); ++i){
        cloud_out.points[i].x = rot_y(0,0) * cloud->points[i].x + rot_y(0,1) * cloud->points[i].y + rot_y(0,2) * cloud->points[i].z + rot_y(0,3) * 1;
        cloud_out.points[i].y = rot_y(1,0) * cloud->points[i].x + rot_y(1,1) * cloud->points[i].y + rot_y(1,2) * cloud->points[i].z + rot_y(1,3) * 1;
        cloud_out.points[i].z = rot_y(2,0) * cloud->points[i].x + rot_y(2,1) * cloud->points[i].y + rot_y(2,2) * cloud->points[i].z + rot_y(2,3) * 1;
    }

    return cloud_out;
}


pcl::PointCloud<pcl::PointXYZ> rotation_z(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double theta)///rotate point cloud by Z axiz
{
    Eigen::Matrix4f rot_z;
    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    cloud_out = *cloud;
    double theta_z = theta * 3.1415926 / 180.0;//角度の変換
    rot_z(0,0) = cos(theta_z);
    rot_z(1,0) = -sin(theta_z);
    rot_z(2,0) = 0.0;
    rot_z(3,0) = 0.0;
    rot_z(0,1) = sin(theta_z);
    rot_z(1,1) = cos(theta_z);
    rot_z(2,1) = 0.0;
    rot_z(3,1) = 0.0;
    rot_z(0,2) = 0.0;
    rot_z(1,2) = 0.0;
    rot_z(2,2) = 1.0;
    rot_z(3,2) = 0.0;
    rot_z(0,3) = 0.0;
    rot_z(1,3) = 0.0;
    rot_z(2,3) = 0.0;
    rot_z(3,3) = 1.0;

    for(size_t i = 0; i < cloud->points.size(); ++i){
        cloud_out.points[i].x = rot_z(0,0) * cloud->points[i].x + rot_z(0,1) * cloud->points[i].y + rot_z(0,2) * cloud->points[i].z + rot_z(0,3) * 1;
        cloud_out.points[i].y = rot_z(1,0) * cloud->points[i].x + rot_z(1,1) * cloud->points[i].y + rot_z(1,2) * cloud->points[i].z + rot_z(1,3) * 1;
        cloud_out.points[i].z = rot_z(2,0) * cloud->points[i].x + rot_z(2,1) * cloud->points[i].y + rot_z(2,2) * cloud->points[i].z + rot_z(2,3) * 1;
    }

    return cloud_out;
}



int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile(argv[1],*cloud);

  //*rotated_cloud = rotation_x(cloud, 180.0); //回転
  // *rotated_cloud = rotation_y(cloud, -90.0); //回転
  *rotated_cloud = rotation_z(cloud, 90.0); //回転

  std::stringstream Filename;
  std::string name;
  name = argv[1];
  name.erase(name.length()-4);
  Filename << name << "_z.pcd";

  pcl::io::savePCDFileBinary(Filename.str(), *rotated_cloud);

  return 0;
}
