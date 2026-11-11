//
// Created by kk on 2026/11/10.
//
#pragma once
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <XmlRpcValue.h>
#include <ros/node_handle.h>
#include <string>
#include <vector>

namespace hero_chassis_controller {

//继承 Controller<EffortJointInterface>：
class HeroChassisController
    : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  //只调用一次
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

  // 清pid积分
  void starting(const ros::Time& time) override;

  //每个控制周期调用：算误差，力矩，下发
  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  static constexpr int WheelN = 4;

  //读速度，写力矩
  hardware_interface::JointHandle joints_[WheelN];

  control_toolbox::Pid pids_[WheelN];

  //各自转速，rad单位
  double target_velocity_[WheelN]{};

  //关节名，顺序要和数组对应
  std::vector<std::string> joint_names_;
};

}  // namespace hero_chassis_controller
