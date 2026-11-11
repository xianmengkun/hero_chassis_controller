# hero_chassis_controller

## 简介

ROS1下RoboMaster英雄机器人底盘控制器插件。四个麦克纳姆轮各自拥有一个独立的
PID 控制器，把目标轮速（rad/s）经 `EffortJointInterface` 换算成关节力矩指令（N·m）。

控制器由 `controller_manager` 通过 pluginlib 加载，仿真时与 Gazebo 共用同一个 `gzserver`
进程。订阅 `/cmd_vel`，用逆运动学把底盘速度换算成四个轮子的期望转速。

## 编译

```bash
cd ~/catkin_ws
catkin build hero_chassis_controller
source devel/setup.bash
```

## 运行

带 Gazebo 的完整仿真：

```bash
roslaunch hero_chassis_controller run_simulation_and_controller.launch
rosservice call /gazebo/unpause_physics "{}"   # 解除暂停
```

已经有 ros_control 系统在跑时，只加载控制器：

```bash
roslaunch hero_chassis_controller load_controller.launch
```

控制器起来后，另开一个终端发速度指令（线速度 m/s，角速度 rad/s）：

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.4}}"    # 前进
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {y: 0.4}}"    # 左移
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{angular: {z: 1.0}}"   # 逆时针转
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{}"                    # 停车
```

## 配置文件

- `config/controllers.yaml` —— 控制器类型、四个关节名、底盘尺寸，以及每个轮子一组 PID 参数。

## launch 文件

- `launch/run_simulation_and_controller.launch` —— 启动带 hero 模型的 Gazebo，并加载
  `hero_chassis_controller` 与 `joint_state_controller`。
- `launch/load_controller.launch` —— 只加载控制器，不启动仿真器。

## 控制器

### 逆运动学

订阅 `/cmd_vel`（`geometry_msgs/Twist`），把它当作 `base_link` 坐标系下的速度指令，
换算成四个轮子的目标转速：

```
k = (wheel_base + wheel_track) / 2

v_left_front  = (vx - vy - k * wz) / wheel_radius
v_right_front = (vx + vy + k * wz) / wheel_radius
v_left_back   = (vx + vy - k * wz) / wheel_radius
v_right_back  = (vx - vy + k * wz) / wheel_radius
```

| 参数 | 含义 | 默认值 |
|---|---|---|
| `wheel_radius` | 轮子半径 | 0.07625 |
| `wheel_base` | 前后轴距 | 0.4 |
| `wheel_track` | 左右轮距 | 0.4 |

坐标系与 URDF 的 `base_link` 一致：x 向前、y 向左、z 向上，`angular.z` 逆时针为正。

实测（`wz=1.0` 时四轮转速 `[-5.27, -5.08, +5.23, +5.35]`，理论值 ±5.25）。

## 调参

```bash
rosrun rqt_reconfigure rqt_reconfigure   # 拖动 p/i/d 滑块，观察 /joint_states 的速度
rosrun rqt_plot rqt_plot /joint_states/velocity[0]
```

## 声明
- 本英雄底盘控制器由本人及ai编写，ai起到辅助，纠错，教学作用，并未完全使用ai直接生成。
- qwq佛祖保佑