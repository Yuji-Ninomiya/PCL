#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void
keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void* nothing)
{
  if (event.getKeySym () == "space" && event.keyDown ())
    next_iteration = true;
}

int
main (int argc, char** argv)
{
  // The point clouds we will be using
  PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud    //PointCloudT <= pcl::PointCloud<pcl::PointXYZ>
  PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
  PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud

  // Checking program arguments
  if (argc < 2)
  {
    printf ("Usage :\n");
    printf ("\t\t%s file.pcl number_of_ICP_iterations\n", argv[0]);
    PCL_ERROR ("Provide one pcl file.\n");
    return (-1);
  }

  int iterations = 1 ;  // Default number of ICP iterations

  if (argc > 2)
  {
    // If the user passed the number of iteration as an argument
    iterations = atoi (argv[2]);
    if (iterations < 1)
    {
      PCL_ERROR ("Number of initial iterations must be >= 1\n");
      return (-1);
    }
  }

  pcl::console::TicToc time;
  time.tic ();

  PointCloudT::Ptr cloud_o (new PointCloudT);  //追加したやつ

  //モデルデータの読み込み
  pcl::PCDReader reader;
  reader.read<pcl::PointXYZ> ("sia5_dhand_15000p.pcd", *cloud_in);

  //取得したPCDファイルを読み込む
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud_o) == -1)  //icp -> o
  {
    PCL_ERROR ("Couldn't read file.pcd \n", argv[1]);
    return (-1);
  }


  std::cout << "\nLoaded file " << argv[1] << " (" <<cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;

  // Defining a rotation matrix and translation vector
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();


  double phi = 0.0362 ; // roll ロール
  double theta = -4.4907 ; // pich ピッチ
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
  transformation_matrix (0, 3) = 0.1466;
  transformation_matrix (1, 3) = -0.3067;
  transformation_matrix (2, 3) = -0.0661;

  // Display in terminal the transformation matrix
  std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
  print4x4Matrix (transformation_matrix);

  // // Executing the transformation
  pcl::transformPointCloud (*cloud_o, *cloud_icp, transformation_matrix);
  //pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);
  *cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use

  // The Iterative Closest Point algorithm  :パラメータセット
  time.tic ();
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setTransformationEpsilon (1e-8);  //ネットより引用した値　要検証
  //icp.setMaxCorrespondenceDistance (0.05);  // 許容される点群との距離
  icp.setMaximumIterations (iterations); // 最大反復回数
  icp.setInputSource (cloud_icp);
  icp.setInputTarget (cloud_in);  //モデルデータ
  // PointCloudT::Ptr cloud_new (new PointCloudT);  //追加
  icp.align (*cloud_icp);// icp -> new
  //icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
  std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;


  //fprintf(stderr, "Check\n"); // デバッカー

  if (icp.hasConverged ())  //繰り返しのたびに表示されるやつ
  {
    std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
    transformation_matrix = icp.getFinalTransformation ().cast<double>();
    print4x4Matrix (transformation_matrix);
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
    return (-1);
  }






  // Visualization    *** ウィンドウに関する記述 ***
  pcl::visualization::PCLVisualizer viewer ("ICP demo");
  // Create two verticaly separated viewports
  int v1 (0);
  int v2 (1);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

  // The color we will be using
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                             (int) 255 * txt_gray_lvl);
  viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
  viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

  // Transformed point cloud is green
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
  viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

  // ICP aligned point cloud is red
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
  viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2); //icp -> new

  // Adding text descriptions in each viewport
  viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
  viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

  std::stringstream ss;
  ss << iterations;
  std::string iterations_cnt = "ICP iterations = " + ss.str ();
  viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

  // Set background color
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

  // Set camera position and orientation <-- ウィンドウの視点の位置座標
  //viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  //viewer.setCameraPosition (0.0466, -0.6767, 0.5661, -0.0362, 0.3093, 2.0462);
  viewer.setCameraPosition (0.8466, -0.6767, 0.5661, -0.0362, -0.3093, 2.0462);  //kinectのワールド座標
  viewer.setSize (1280, 1024);  // Visualiser window size

  // Register keyboard callback :
  viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);



  // Display the visualiser
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();

    // The user pressed "space" :
    if (next_iteration)
    {
      // The Iterative Closest Point algorithm
      time.tic ();
      icp.align (*cloud_icp);
      std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;

      if (icp.hasConverged ())
      {
        printf ("\033[11A");  // Go up 11 lines in terminal output.
        printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
        std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
        print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose

        ss.str ("");
        ss << iterations;
        std::string iterations_cnt = "ICP iterations = " + ss.str ();
        viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
        viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");  // icp -> new
      }
      else
      {
        PCL_ERROR ("\nICP has not converged.\n");
        return (-1);
      }
    }
    next_iteration = false;
  }
  return (0);
}
