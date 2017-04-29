#include <iostream>
#include <string>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

//#include <math/YawPitchRoll.h>

// 取得データをワールド座標系に変換するプログラム

float roll_f_b, pitch_f_b, yaw_f_b, roll_f, pitch_f, yaw_f, roll, pitch, yaw, roll_re, pitch_re, yaw_re;

void
ICP_TransformationMatrix (Eigen::Matrix4d & mat_icp) // 実験結果から得られた変換行列
{
  // 回転行列：R
  mat_icp (0, 0) = -0.3325114846;//-0.442594826;
  mat_icp (0, 1) = 0.1493363678;//-0.885768294;
  mat_icp (0, 2) = 0.9312337041;//-0.139914393;
  mat_icp (1, 0) = 0.7315735817;//0.845540047;
  mat_icp (1, 1) = 0.6640095711;//-0.464181095;
  mat_icp (1, 2) = 0.1547363847;//0.263909549;
  mat_icp (2, 0) = -0.5952228308;//-0.298702389;
  mat_icp (2, 1) = 0.7326936722;//-0.001501144;
  mat_icp (2, 2) = -0.3300207555;//0.95436877;

  // 並進ベクトル：t
  mat_icp (0, 3) = 0.9432457685;//-0.159714058;
  mat_icp (1, 3) = 0.5138898492;//-0.064111352;
  mat_icp (2, 3) = 0.8791828156;//0.877298057;

  mat_icp (3, 0) = 0;
  mat_icp (3, 1) = 0;
  mat_icp (3, 2) = 0;
  mat_icp (3, 3) = 1;
}

void
Initial (Eigen::Matrix4d & mat_init) // 初期値
{
  // 回転行列：R
  mat_init (0, 0) = -0.43517;
  mat_init (0, 1) = 0.83555;
  mat_init (0, 2) = -0.335466;
  mat_init (1, 0) = -0.880982;
  mat_init (1, 1) = -0.472;
  mat_init (1, 2) = -0.0329651;
  mat_init (2, 0) = -0.185884;
  mat_init (2, 1) = 0.281196;
  mat_init (2, 2) = 0.941475;

  // 並進ベクトル：t
  mat_init (0, 3) = 1.11585;
  mat_init (1, 3) = 0.489664;
  mat_init (2, 3) = -0.201735;

  mat_init (3, 0) = 0;
  mat_init (3, 1) = 0;
  mat_init (3, 2) = 0;
  mat_init (3, 3) = 1;
}

void
RE_initial (Eigen::Matrix4d & mat_init) // 初期値の逆行列
{
  // 回転行列：R
  mat_init (0, 0) = 0.99998;
  mat_init (0, 1) = -0.00061;
  mat_init (0, 2) = -0.00597;
  mat_init (1, 0) = 0.00082;
  mat_init (1, 1) = 0.99936;
  mat_init (1, 2) = 0.03579;
  mat_init (2, 0) = 0.00595;
  mat_init (2, 1) = -0.03579;
  mat_init (2, 2) = 0.99934;

  // 並進ベクトル：t
  mat_init (0, 3) = -0.87631;
  mat_init (1, 3) = 0.62259;
  mat_init (2, 3) = -0.60831;

  mat_init (3, 0) = 0;
  mat_init (3, 1) = 0;
  mat_init (3, 2) = 0;
  mat_init (3, 3) = 1;
}

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


int main(int argc, char **argv)
{
  // // kinect_first の 真値算出
  // Eigen::Matrix4d mat_first;
  // Make_rotationMatrix(mat_first, 0.8794, -0.6445, 0.5804, -0.035, 0.342, 2.051);

  // Eigen::Matrix4d kinect_first;
  // Make_rotationMatrix(kinect_first,0, 0, 0, -M_PI/2, M_PI/2, 0);

  // Eigen::Matrix4d transformation_first;
  // transformation_first = mat_first * kinect_first;
  // cout << "\n kinect first from world \n\n" << transformation_first << endl;
  
  // // kinect_first から base_link への変換を考える
  // // Eigen::Matrix4d inverse_first;
  // // inverse_first = transformation_first.inverse();
  // // cout << "\n kinect_first to motoman\n\n" << inverse_first << endl;

  // Eigen::Matrix4d inverse_first;
  // inverse_first = mat_first.inverse();
  // cout << "\n kinect_first to motoman\n\n" << inverse_first << endl;

  // Eigen::Matrix3d rotation_base;
  // rotation_base = inverse_first.block(0, 0, 3, 3);
  // Eigen::Affine3f base_answer(rotation_base.cast<float>());

  // // kinect_first から base_link 座標系への回転行列のパラメータを表示
  // pcl::getEulerAngles(base_answer, roll_f_b, pitch_f_b, yaw_f_b);
  // cout << "\n The output Euler angles (using getEulerAngles function) are :\n\n" << "roll_f_b : " << roll_f_b << " ,pitch_f_b : " << pitch_f_b << " ,yaw_f_b : " << yaw_f_b << endl;


  // // kinect_first のベースリンク座標系における真値
  // Eigen::Matrix3d rotation_first;
  // rotation_first = transformation_first.block(0, 0, 3, 3);
  // Eigen::Affine3f first_answer(rotation_first.cast<float>());

  // // 角度を表示
  // pcl::getEulerAngles(first_answer, roll_f, pitch_f, yaw_f);
  // cout << "\n The output Euler angles (using getEulerAngles function) are :\n\n" << "roll_f : " << roll_f << " ,pitch_f : " << pitch_f << " ,yaw_f : " << yaw_f << endl;



  //  // 初期値を与える場合
  Eigen::Matrix4d initial;
  // // Make_rotationMatrix(initial, 0.8466, -0.7767, 0.6661, -0.0362, -0.4907, 5.8214);
  // // Make_rotationMatrix(initial, 0.8794, -0.6445, 0.5804, -0.035, 0.3421, 2.0509);
  Make_rotationMatrix(initial, 0.8794, -0.6445, 0.5804, -0.035, 0.3421, 2.0509);
  cout << "\n initial matrix\n\n" << initial << endl;

  // 初期値の逆行列
  Eigen::Matrix4d transformation_initial;
  //RE_initial (transformation_initial);
  transformation_initial = initial.inverse();
  cout << "\n inverse of initial position matrix\n\n" << transformation_initial << endl;

  Eigen::Matrix3d rotation_base;
  rotation_base = transformation_initial.block(0, 0, 3, 3);
  Eigen::Affine3f base_answer(rotation_base.cast<float>());

  // kinect_first から base_link 座標系への回転行列のパラメータを表示
  pcl::getEulerAngles(base_answer, roll_f_b, pitch_f_b, yaw_f_b);
  cout << "\n The output Euler angles (using getEulerAngles function) are :\n\n" << "roll_f_b : " << roll_f_b << " ,pitch_f_b : " << pitch_f_b << " ,yaw_f_b : " << yaw_f_b << endl;




  // // モデルデータ
  // // ワールドから見た kinect_third へ変換
  // Eigen::Matrix4d mat_a;
  // Make_rotationMatrix(mat_a, -2, 0, 0.9, 0, M_PI/6, 0);
  // cout << "\n matrix a \n\n" << mat_a << endl;

  // // kinect_third を kinect 座標系へ変換
  // Eigen::Matrix4d mat_b;
  // Make_rotationMatrix(mat_b, 0, 0, 0, -M_PI/2, M_PI/2, 0);
  // cout << "\n matrix b \n\n" << mat_b << endl;

  // // ワールド座標系から kinect 座標系への変換
  // Eigen::Matrix4d transformation_kinect;
  // transformation_kinect = mat_a * mat_b;
  // cout << "\n transformation_kinect \n\n" << transformation_kinect << endl;

  // // 逆変換して表示
  // Eigen::Matrix4d kinect_third;
  // kinect_third = transformation_kinect.inverse();
  // cout << "\n kinect_third \n\n" << kinect_third << endl;

  // ICP アルゴリズムの結果  取得データからモデルデータへの変換行列を表示
  Eigen::Matrix4d icp_matrix;
  ICP_TransformationMatrix(icp_matrix);
  cout << "\n icp transformation_matrix \n\n" << icp_matrix << endl;

  // // 初期値入力
  // Eigen::Matrix4d init_mat;
  // Initial(init_mat);
  // cout << "\n initial matrix \n\n" << init_mat << endl;

  // 計算
  Eigen::Matrix4d estimate;
  estimate = icp_matrix * transformation_initial;
  cout << "\n Estimation \n\n" << estimate << endl;

  // kinect_first から　base_link へのicp経由での回転
  Eigen::Matrix3d rotation_final;
  rotation_final = estimate.block(0, 0, 3, 3);
  Eigen::Affine3f transformation_answer(rotation_final.cast<float>());

  //角度を表示
  pcl::getEulerAngles(transformation_answer, roll, pitch, yaw);
  cout << "\n The output Euler angles (using getEulerAngles function) are :\n\n" << "roll : " << roll << " ,pitch : " << pitch << " ,yaw : " << yaw << endl;

  // // 逆変換して表示
  // Eigen::Matrix4d inversed_answer;
  // inversed_answer = estimate.inverse();
  // cout << "\n inverse the final transformation \n\n" << inversed_answer << endl;


  // オイラー角出すやつ
  // boost::tie(yaw, pitch,roll) = matrixToYawPitchRoll(rotation_final);
  // cout << "\n Euler angles" << tie << endl;
  // // kinect_first から　base_link へのicp経由での回転
  // Eigen::Matrix3d rotation_final2;
  // rotation_final2 = inversed_answer.block(0, 0, 3, 3);
  // Eigen::Affine3f retransformation_answer(rotation_final2.cast<float>());

  // // 角度を表示
  // pcl::getEulerAngles(retransformation_answer, roll_re, pitch_re, yaw_re);
  // cout << "\n The output Euler angles (using getEulerAngles function) are :\n\n" << "roll_re : " << roll_re << " ,pitch_re : " << pitch_re << " ,yaw_re : " << yaw_re << endl;

  
  // // 取得データをワールド座標に変換
  // Eigen::Matrix4d transformation_matrix;
  // transformation_matrix = transformation_initial * icp_matrix * kinect_third;   // 初期値をプラス
  // cout << "\n final transformation matrix \n\n" << transformation_matrix << endl;

  // // 逆変換して表示
  // Eigen::Matrix4d inversed_answer;
  // inversed_answer = transformation_matrix.inverse();
  // cout << "\n inverse the final transformation \n\n" << inversed_answer << endl;


  
  // // kinect_first から　base_link へのicp経由での回転
  // Eigen::Matrix3d rotation_final;
  // rotation_final = transformation_matrix.block(0, 0, 3, 3);
  // Eigen::Affine3f transformation_answer(rotation_final.cast<float>());

  // // 角度を表示
  // pcl::getEulerAngles(transformation_answer, roll, pitch, yaw);
  // cout << "\n The output Euler angles (using getEulerAngles function) are :\n\n" << "roll : " << roll << " ,pitch : " << pitch << " ,yaw : " << yaw << endl;



  // // //base_link から kinect_first へのicp経由での回転の抽出
  // // Eigen::Matrix3d rotation_icp;
  // // rotation_icp = inversed_answer.block(0, 0, 3, 3);
  // // Eigen::Affine3f icp_inverse_answer(rotation_icp.cast<float>());

  // // // 角度を表示
  // // pcl::getEulerAngles(icp_inverse_answer, roll_re, pitch_re, yaw_re);
  // // cout << "\n The output Euler angles (using getEulerAngles function) are :\n\n" << "roll_re : " << roll_re << " ,pitch_re : " << pitch_re << " ,yaw_re : " << yaw_re << endl;



  // // 3rd to 1st
  // Eigen::Matrix4d first_to_third;
  // first_to_third = kinect_third * transformation_first;
  // cout << "\n Kinect 3rd to 1st \n\n" << first_to_third << endl;

  // // 引用 もともと上のやつ
  // Eigen::Matrix3d rotation_icp;
  // rotation_icp = first_to_third.block(0, 0, 3, 3);
  // Eigen::Affine3f icp_inverse_answer(rotation_icp.cast<float>());

  // // 角度を表示
  // pcl::getEulerAngles(icp_inverse_answer, roll_re, pitch_re, yaw_re);
  // cout << "\n The output Euler angles (using getEulerAngles function) are :\n\n" << "roll_re : " << roll_re << " ,pitch_re : " << pitch_re << " ,yaw_re : " << yaw_re << endl;

  return 0;

}
