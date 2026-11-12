//
// Created by kk on 2026/11/10.
//
// Part of the hero_chassis_controller package. BSD license, see LICENSE.
//
#pragma once

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>
#include <vector>

namespace hero_chassis_controller {

// Mecanum wheel chassis: PID per wheel, kinematics and odometry
class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  // Loads parameters, joint handles and PIDs
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

  // Resets PIDs and odometry
  void starting(const ros::Time& time) override;

  // Runs the PIDs, then updates the odometry
  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  // Wheel count
  static constexpr int kWheelNum = 4;

  // Inverse kinematics: chassis velocity to wheel velocities
  void CmdVelCallback(const geometry_msgs::Twist& msg);

  // Forward kinematics and integration, publishes /odom and tf
  void UpdateOdometry(const ros::Time& time, const ros::Duration& period);

  // Joint handles, order LF, RF, LB, RB
  hardware_interface::JointHandle joints_[kWheelNum];

  // PID per wheel
  control_toolbox::Pid pids_[kWheelNum];

  // Target wheel velocities, rad/s
  double target_velocity_[kWheelNum]{};

  // Joint names, same order as above
  std::vector<std::string> joint_names_;

  // /cmd_vel subscriber
  ros::Subscriber cmd_vel_sub_;

  // Chassis geometry, m
  double wheel_radius_{0.07625};
  double half_wheel_base_{0.2};   // half wheelbase
  double half_wheel_track_{0.2};  // half track

  // Odometry state in the odom frame
  ros::Publisher odom_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};

  // Last /odom publish time, limits the rate
  ros::Time last_odom_time_;
};

}  // namespace hero_chassis_controller
