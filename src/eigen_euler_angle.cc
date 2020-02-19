#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>

Eigen::Matrix3d Roll(double w) {
  Eigen::Matrix3d out;
  out << 1, 0, 0, 0, cos(w), sin(w), 0, -sin(w), cos(w);
  return out;
}
Eigen::Matrix3d Pitch(double w) {
  Eigen::Matrix3d out;
  out << cos(w), 0, -sin(w), 0, 1, 0, sin(w), 0, cos(w);
  return out;
}
Eigen::Matrix3d Yaw(double w) {
  Eigen::Matrix3d out;
  out << cos(w), sin(w), 0, -sin(w), cos(w), 0, 0, 0, 1;
  return out;
}

int main(int argc, char** argv) {
  Eigen::Matrix3d Rdc;
  Rdc << 0.0063913, -0.592978, 0.805193, -0.99989, -0.0145938, -0.00281079,
      0.0134176, -0.805086, -0.593006;

  // roll pitch yaw, Eigen只支持Intrinsic Rotation
  Eigen::Vector3d euler_angle = Rdc.eulerAngles(2, 1, 0);

  Eigen::AngleAxisd roll(euler_angle(2), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch(euler_angle(1), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw(euler_angle(0), Eigen::Vector3d::UnitZ());

  Eigen::Matrix3d Rdc_intrinsic = yaw.matrix() * pitch.matrix() * roll.matrix();

  std::cout << "Target Matrix和Output Matrix的数据应该完全一致, "
               "以确定eulerAngles的工作原理."
            << std::endl;
  std::cout << "Intrinsic Euler Angle: " << euler_angle.transpose()
            << std::endl;
  std::cout << "Target Matrix: \n" << Rdc << std::endl;
  std::cout << "Intrinsic Matrix: \n" << Rdc_intrinsic << std::endl;

  Eigen::Matrix3d Rcd_extrinsic =
  Roll(euler_angle(2)) * Pitch(euler_angle(1)) * Yaw(euler_angle(0));

  std::cout << "Extrinsic Matrix: \n" << Rcd_extrinsic.inverse() << std::endl;

  yaw.angle() = M_PI / 2;
  std::cout
      << "\n*******************************************************\n"
      << "下面是Eigen::AngleAxisd计算出的转轴为z轴, 转角90度时候的旋转矩阵\n"
      << "参考State Estimation for robotics, 6.2.2节, p189页.\n"
      << "说明该算法是在坐标内发生旋转点/向量计算, 而不是旋转坐标系."
      << std::endl;
  std::cout << "R_z\n" << yaw.matrix() << std::endl;
}