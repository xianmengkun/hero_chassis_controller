//
// Created by kk on 2026/11/10.
//
#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

bool HeroChassisController::init(hardware_interface::EffortJointInterface* hw,
                                ros::NodeHandle& /*root_nh*/, ros::NodeHandle& controller_nh) {
  //读关节名。yaml里4个joints，顺序与下面一致
  controller_nh.getParam("joints", joint_names_);

  // take handle、读pid参数
  for (int i = 0; i < WheelN; ++i) {
    joints_[i] = hw->getHandle(joint_names_[i]);

    ros::NodeHandle pid_nh(controller_nh, "pid/" + joint_names_[i]);
    pids_[i].init(pid_nh);

    joints_[i].setCommand(0.0);  // 清零力矩
  }

  XmlRpc::XmlRpcValue v;
  if (controller_nh.getParam("target_velocity", v) &&
      v.getType() == XmlRpc::XmlRpcValue::TypeArray && v.size() == WheelN) {
    for (int i = 0; i < WheelN; ++i) {

      target_velocity_[i] = (v[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
                                ? static_cast<double>(static_cast<int>(v[i]))
                                : static_cast<double>(v[i]);
    }
  }

  return true;
}
void HeroChassisController::starting(const ros::Time& /*time*/) {
  for (int i = 0; i < WheelN; ++i) pids_[i].reset();
}// 清积分和上次的输出

void HeroChassisController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  for (int i = 0; i < WheelN; ++i) {
    const double current = joints_[i].getVelocity();       // 实际转速
    const double error = target_velocity_[i] - current;    // 目标 − 实际


    const double effort = pids_[i].computeCommand(error, period);

    joints_[i].setCommand(effort);
  }
}

}  // namespace hero_chassis_controller

// 告诉 pluginlib 这个类存在
PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,controller_interface::ControllerBase)
