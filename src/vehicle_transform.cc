#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

// 绕x轴旋转
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
  ros::init(argc, argv, "my_tf_broadcaster");

  ros::NodeHandle ph;
  ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("path", 10, true);
  ros::Publisher ground_pose_pub =
      ph.advertise<geometry_msgs::PoseStamped>("ground", 10, true);
  ros::Publisher body_pose_pub =
      ph.advertise<geometry_msgs::PoseStamped>("body", 10, true);
  ros::Publisher front_pose_pub =
      ph.advertise<geometry_msgs::PoseStamped>("front", 10, true);
  ros::Publisher front_camera_pose_pub =
      ph.advertise<geometry_msgs::PoseStamped>("front_camera", 10, true);

  ros::Publisher target_camera_pose_pub =
      ph.advertise<geometry_msgs::PoseStamped>("target_camera", 10, true);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  // nav_msgs::Path path;
  // path.header.stamp = current_time;
  // path.header.frame_id = "vehicle";

  Eigen::AngleAxisd yaw(M_PI / 2, Eigen::Vector3d::UnitZ());
  std::cout << "AngleAxisd: \n " << yaw.matrix() << std::endl;
  std::cout << "Matrix: \n " << Yaw(M_PI / 2) << std::endl;
  ros::Rate loop_rate(10);
  int i = 0;
  while (ros::ok()) {
    current_time = ros::Time::now();
    // 定义静态坐标系body
    geometry_msgs::PoseStamped body_pose;
    body_pose.pose.position.x = 0;
    body_pose.pose.position.y = 0;
    body_pose.pose.position.z = 0;

    body_pose.pose.orientation.x = 0;
    body_pose.pose.orientation.y = 0;
    body_pose.pose.orientation.z = 0;
    body_pose.pose.orientation.w = 1;

    current_time = ros::Time::now();
    body_pose.header.stamp = current_time;
    body_pose.header.frame_id = "vehicle";

    Eigen::Matrix3d R_ground_body;
    // Eigen::AngleAxisd test(-M_PI / 2, Eigen::Vector3d::UnitZ());
    // R_ground_body = test.matrix();
    R_ground_body << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    // 描述旋转的时候通常是由静态坐标(world)系到运动坐标系(body),
    // 此时描述的结果是T_body_world, 但是ROS中表示位姿则是以T_world_body表示,
    // 此时两个参数之间相差一个逆
    Eigen::Matrix3d R_body_ground = R_ground_body.inverse();
    Eigen::Quaterniond q_bg(R_body_ground);
    // 定义地面坐标系ground
    geometry_msgs::PoseStamped ground_pose;
    ground_pose = body_pose;

    // 为了可视化使用
    ground_pose.pose.position.y = -1;

    ground_pose.pose.orientation.x = q_bg.x();
    ground_pose.pose.orientation.y = q_bg.y();
    ground_pose.pose.orientation.z = q_bg.z();
    ground_pose.pose.orientation.w = q_bg.w();

    // 标定下的camera系
    Eigen::Matrix3d R_front_ground;
    // front
    // Eigen::Vector3d rot(0.30780044, -1.5569354, -0.0010287743);

    // rear
    Eigen::Vector3d rot(0.20749481, 1.5724747, -0.0018055739);

    // Eigen::AngleAxisd roll(-rot(2), Eigen::Vector3d::UnitX());
    // Eigen::AngleAxisd pitch(rot(0), Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd yaw(rot(1), Eigen::Vector3d::UnitZ());

    R_front_ground = Roll(-rot(2)) * Pitch(rot(0)) * Yaw(-rot(1));
    // R_front_ground = Yaw(0) * Pitch(rot(0)) * Roll(0);
    Eigen::Matrix3d R_ground_front = R_front_ground.inverse();
    Eigen::Quaterniond q_bf(R_body_ground * R_ground_front);
    geometry_msgs::PoseStamped front_pose;
    front_pose = body_pose;
    front_pose.pose.position.y = -4;
    front_pose.pose.orientation.x = q_bf.x();
    front_pose.pose.orientation.y = q_bf.y();
    front_pose.pose.orientation.z = q_bf.z();
    front_pose.pose.orientation.w = q_bf.w();

    Eigen::Matrix3d R_fcamera_front;
    R_fcamera_front << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    Eigen::Matrix3d R_front_fcamera = R_fcamera_front.inverse();
    Eigen::Matrix3d R_body_front_camera =
        R_body_ground * R_ground_front * R_front_fcamera;
    Eigen::Quaterniond q_body_front_camera(R_body_front_camera);
    geometry_msgs::PoseStamped front_camera_pose;
    front_camera_pose = body_pose;
    front_camera_pose.pose.position.y = 3;
    front_camera_pose.pose.orientation.x = q_body_front_camera.x();
    front_camera_pose.pose.orientation.y = q_body_front_camera.y();
    front_camera_pose.pose.orientation.z = q_body_front_camera.z();
    front_camera_pose.pose.orientation.w = q_body_front_camera.w();
    // std::cout << "Now\n" << R_body_front_camera << std::endl;

    // front camera
    front_camera_pose_pub.publish(front_camera_pose);
    front_pose_pub.publish(front_pose);

    body_pose_pub.publish(body_pose);
    ground_pose_pub.publish(ground_pose);

    // path.poses.push_back(body_pose);
    // path_pub.publish(path);

    // 目标坐标系target camera
    geometry_msgs::PoseStamped target_camera_pose;
    target_camera_pose = body_pose;
    target_camera_pose.pose.position.y = 3;
    Eigen::Matrix3d R_body_target_camera;
    // R_body_target_camera << 0.0063913, -0.592978, 0.805193, -0.99989,
    //     -0.0145938, -0.00281079, 0.0134176, -0.805086, -0.593006;
    R_body_target_camera << 0.039668676, 0.54972672, -0.83440185, 0.99869508,
        0.0050803246, 0.050826512, 0.032179657, -0.83532894, -0.54880744;

    // std::cout << "Target\n" << R_body_target_camera << std::endl;
    Eigen::Quaterniond q_b_tc(R_body_target_camera);

    target_camera_pose.pose.orientation.x = q_b_tc.x();
    target_camera_pose.pose.orientation.y = q_b_tc.y();
    target_camera_pose.pose.orientation.z = q_b_tc.z();
    target_camera_pose.pose.orientation.w = q_b_tc.w();

    target_camera_pose_pub.publish(target_camera_pose);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
};