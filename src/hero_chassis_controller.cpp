//
// Created by kk on 2026/11/10.
//
// Part of the hero_chassis_controller package. BSD license, see LICENSE.
//
#include "hero_chassis_controller/hero_chassis_controller.h"
#include <cmath>
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

namespace {
// Odom publish period, s
constexpr double kOdomPublishPeriod = 0.02;
}  // namespace

bool HeroChassisController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh,
                                 ros::NodeHandle& controller_nh) {
  controller_nh.getParam("joints", joint_names_);

  for (int i = 0; i < kWheelNum; ++i) {
    joints_[i] = hw->getHandle(joint_names_[i]);

    ros::NodeHandle pid_nh(controller_nh, "pid/" + joint_names_[i]);
    pids_[i].init(pid_nh);

    joints_[i].setCommand(0.0);
  }

  double wheel_base = 0.4;   // wheelbase
  double wheel_track = 0.4;  // track
  controller_nh.param("wheel_radius", wheel_radius_, 0.07625);
  controller_nh.param("wheel_base", wheel_base, 0.4);
  controller_nh.param("wheel_track", wheel_track, 0.4);

  half_wheel_base_ = wheel_base / 2.0;
  half_wheel_track_ = wheel_track / 2.0;

  std::string velocity_mode = "chassis";
  controller_nh.param<std::string>("velocity_mode", velocity_mode, "chassis");
  controller_nh.param<std::string>("world_frame", world_frame_, "odom");
  use_world_frame_ = (velocity_mode == "global");

  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

  cmd_vel_sub_ = root_nh.subscribe("/cmd_vel", 1, &HeroChassisController::CmdVelCallback, this);

  odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("/odom", 50);

  return true;
}

void HeroChassisController::starting(const ros::Time& /*time*/) {
  for (int i = 0; i < kWheelNum; ++i) pids_[i].reset();

  for (int i = 0; i < kWheelNum; ++i) target_velocity_[i] = 0.0;

  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;
  last_odom_time_ = ros::Time::now();
}

// Inverse kinematics: chassis velocity to wheel velocities
void HeroChassisController::CmdVelCallback(const geometry_msgs::Twist& msg) {
  double vx = msg.linear.x;         // forward
  double vy = msg.linear.y;         // left
  const double wz = msg.angular.z;  // counter-clockwise

  // Global mode: the command is in the world frame, rotate it into base_link
  if (use_world_frame_) {
    if (!tf_buffer_.canTransform("base_link", world_frame_, ros::Time(0))) return;

    const geometry_msgs::TransformStamped world_to_base =
        tf_buffer_.lookupTransform("base_link", world_frame_, ros::Time(0));

    tf2::Quaternion q(world_to_base.transform.rotation.x, world_to_base.transform.rotation.y,
                      world_to_base.transform.rotation.z, world_to_base.transform.rotation.w);
    tf2::Vector3 v(vx, vy, 0.0);
    v = tf2::quatRotate(q, v);

    vx = v.x();
    vy = v.y();
  }

  // Mecanum constant: half wheelbase + half track
  const double k = half_wheel_base_ + half_wheel_track_;

  // Order: LF, RF, LB, RB
  target_velocity_[0] = (vx - vy - k * wz) / wheel_radius_;  // LF
  target_velocity_[1] = (vx + vy + k * wz) / wheel_radius_;  // RF
  target_velocity_[2] = (vx + vy - k * wz) / wheel_radius_;  // LB
  target_velocity_[3] = (vx - vy + k * wz) / wheel_radius_;  // RB
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) {
  for (int i = 0; i < kWheelNum; ++i) {
    const double current = joints_[i].getVelocity();
    const double error = target_velocity_[i] - current;

    const double effort = pids_[i].computeCommand(error, period);

    joints_[i].setCommand(effort);
  }

  UpdateOdometry(time, period);
}

// Forward kinematics and integration, from wheel velocities to pose
void HeroChassisController::UpdateOdometry(const ros::Time& time, const ros::Duration& period) {
  double wheel_velocity[kWheelNum];
  for (int i = 0; i < kWheelNum; ++i) {
    wheel_velocity[i] = joints_[i].getVelocity();
  }

  const double k = half_wheel_base_ + half_wheel_track_;

  // Order: LF, RF, LB, RB
  const double vx =
      wheel_radius_ / kWheelNum * (wheel_velocity[0] + wheel_velocity[1] + wheel_velocity[2] + wheel_velocity[3]);
  const double vy =
      wheel_radius_ / kWheelNum * (-wheel_velocity[0] + wheel_velocity[1] + wheel_velocity[2] - wheel_velocity[3]);
  const double wz = wheel_radius_ / (kWheelNum * k) *
                    (-wheel_velocity[0] + wheel_velocity[1] - wheel_velocity[2] + wheel_velocity[3]);

  // Body frame velocity, rotate into odom then integrate
  const double dt = period.toSec();
  x_ += (vx * cos(theta_) - vy * sin(theta_)) * dt;
  y_ += (vx * sin(theta_) + vy * cos(theta_)) * dt;
  theta_ += wz * dt;

  if ((time - last_odom_time_).toSec() < kOdomPublishPeriod) return;
  last_odom_time_ = time;

  geometry_msgs::Quaternion q;
  q.z = sin(theta_ / 2.0);
  q.w = cos(theta_ / 2.0);

  nav_msgs::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.orientation = q;
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = wz;
  odom_pub_.publish(odom);

  // Broadcast odom -> base_link
  geometry_msgs::TransformStamped tf;
  tf.header.stamp = time;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "base_link";
  tf.transform.translation.x = x_;
  tf.transform.translation.y = y_;
  tf.transform.rotation = q;
  tf_broadcaster_.sendTransform(tf);
}

}  // namespace hero_chassis_controller

// Register with pluginlib
PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
