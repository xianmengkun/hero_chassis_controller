# hero_chassis_controller

## 简介

ROS1下RoboMaster英雄机器人底盘控制器插件。四个麦克纳姆轮各自拥有一个独立的
PID 控制器，把目标轮速（rad/s）经 `EffortJointInterface` 换算成关节力矩指令（N·m）。

控制器由 `controller_manager` 通过 pluginlib 加载，仿真时与 Gazebo 共用同一个 `gzserver`
进程。

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

## 配置文件

- `config/controllers.yaml` —— 控制器类型、四个关节名、目标轮速，以及每个轮子一组 PID 参数。

## launch 文件

- `launch/run_simulation_and_controller.launch` —— 启动带 hero 模型的 Gazebo，并加载
  `hero_chassis_controller` 与 `joint_state_controller`。
- `launch/load_controller.launch` —— 只加载控制器，不启动仿真器。

## 控制器

### hero_chassis_controller

## 调参

```bash
rosrun rqt_reconfigure rqt_reconfigure   # 拖动 p/i/d 滑块，观察 /joint_states 的速度
rosrun rqt_plot rqt_plot /joint_states/velocity[0]
```

## 声明
- 本英雄底盘控制器由本人及ai编写，ai起到辅助，纠错，教学作用，并未完全使用ai直接生成。
- qwq佛祖保佑