//
// Created by kk on 2026/11/10.
//
#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

bool HeroChassisController::init(hardware_interface::EffortJointInterface* hw,
                                ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
  //读关节名。yaml里4个joints，顺序与下面一致
  controller_nh.getParam("joints", joint_names_);

  // take handle、读pid参数
  for (int i = 0; i < WheelN; ++i) {
    joints_[i] = hw->getHandle(joint_names_[i]);

    ros::NodeHandle pid_nh(controller_nh, "pid/" + joint_names_[i]);
    pids_[i].init(pid_nh);

    joints_[i].setCommand(0.0);  // 清零力矩
  }
  double wheel_base = 0.4;   //前后轴距
  double wheel_track = 0.4;  //左右轮距
  controller_nh.param("wheel_radius", wheel_radius_, 0.07625);
  controller_nh.param("wheel_base", wheel_base, 0.4);
  controller_nh.param("wheel_track", wheel_track, 0.4);

  //逆运动学里是轮子到中心的距离
  half_wheel_base_ = wheel_base / 2.0;
  half_wheel_track_ = wheel_track / 2.0;

  // /cmd_vel 在根命名空间，用 root_nh
  cmd_vel_sub_ = root_nh.subscribe("/cmd_vel", 1, &HeroChassisController::CmdVelCallback, this);

  return true;
}

void HeroChassisController::starting(const ros::Time& /*time*/) {
  for (int i = 0; i < WheelN; ++i) pids_[i].reset();  // 清积分和上次的输出
}

//逆运动学：底盘速度 -> 四个轮子的转速
void HeroChassisController::CmdVelCallback(const geometry_msgs::Twist& msg) {
  const double vx = msg.linear.x;   //前
  const double vy = msg.linear.y;   //左
  const double wz = msg.angular.z;  //逆时针转

  //麦轮公式常数：两个方向离中心距离和
  const double k = half_wheel_base_ + half_wheel_track_;

  target_velocity_[0] = (vx - vy - k * wz) / wheel_radius_;  //LF
  target_velocity_[1] = (vx + vy + k * wz) / wheel_radius_;  //RF
  target_velocity_[2] = (vx + vy - k * wz) / wheel_radius_;  //LB
  target_velocity_[3] = (vx - vy + k * wz) / wheel_radius_;  //RB
}

void HeroChassisController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  for (int i = 0; i < WheelN; ++i) {
    const double current = joints_[i].getVelocity();     // 实际转速
    const double error = target_velocity_[i] - current;  // 目标 − 实际

    const double effort = pids_[i].computeCommand(error, period);

    joints_[i].setCommand(effort);
  }
}

}  // namespace hero_chassis_controller

// 告诉 pluginlib 这个类存在
PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,controller_interface::ControllerBase)
