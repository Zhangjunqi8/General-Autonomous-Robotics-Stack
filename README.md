# hanmole_navigation

`hanmole_navigation` 是瀚墨机器人的导航包，对外提供统一的导航入口：

- `nav_mode:=ekf_odom` 时装配 Nav2 + EKF 导航链
- `nav_mode:=wheel_odom` 时装配基于 `/odom` 的简化版 `NavigateToPose`
- 导航可视化与参数资产

`launch/bringup.launch.py` 是本包唯一对外文档化的导航聚合入口。它负责持有导航参数真值、地图路径接口和 Nav2 场景装配；`nav_bringup.launch.py` 仅作为包内实现细节保留。

## 定位

本包是 **导航聚合入口 + Nav2 资产包**，同时保留 wheel odom 模式下的轻量导航实现。

它应当持有：

- Nav2 参数文件
- 地图文件

- 行为树配置
- RViz 配置
- 导航入口 launch

## 当前机器人版本支持

### 公共 AGV 速度契约

- 对外唯一公开速度命令 topic：`/cmd_vel`
- 消息类型：`geometry_msgs/msg/TwistStamped`
- `header.stamp` 由生产者提供，供底盘控制器按消息时间做超时判断
- `header.frame_id` 仅允许 `""` 或 `base_footprint`

### `v0_1`

- 底盘类型：麦克纳姆轮
- 底盘控制器：本地 `hanmole_controllers/MecanumDriveController`
- 控制器直接订阅 `TwistStamped /cmd_vel`
- 控制器只发布原始轮式里程计 `/odom`，不发布 `odom -> base_footprint` TF
- `/imu/data` 由传感器层统一提供，`robot_localization` 融合 `/odom + /imu/data` 输出 `/odometry/filtered`

### `v0_2`

- 底盘类型：四转四驱
- 底盘控制器：`hanmole_controllers/SwerveDriverController`
- 直接订阅 `TwistStamped /cmd_vel`
- 发布原始轮式里程计 `/odom`，不发布 `odom -> base_footprint` TF
- `/imu/data` 由传感器层统一提供，`robot_localization` 融合 `/odom + /imu/data` 输出 `/odometry/filtered`

## 与其他包的边界

- `hanmole_navigation` 不实现底盘控制算法
- `hanmole_navigation` 不直接访问 `hanmole_sdk`
- `hanmole_navigation` 不负责建图
- `hanmole_navigation` 不负责多机器人协同

职责分工如下：

- `hanmole_bringup`：系统级场景装配，只 include 本包 `launch/bringup.launch.py`
- `hanmole_navigation`：导航参数、地图、行为树、可视化、导航场景装配，以及 `wheel_odom` 模式导航入口
- `hanmole_slam`：建图与地图保存
- `hanmole_multi_robot`：多机器人协同
- `hanmole_controllers`：底盘执行控制器，发布 `/odom`
- `hanmole_hardware` / `hanmole_bringup`：底盘控制器启动与硬件装配
- `robot_localization`：融合 `/odom + /imu/data`，输出 `/odometry/filtered` 和唯一的 `odom -> base_footprint` TF

## Public Bringup 参数真值

以下参数由 `launch/bringup.launch.py` 对外拥有：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `robot_version` | `v0_2` | 机器人版本，如 `v0_1`、`v0_2` |
| `use_sim_time` | `false` | 是否使用仿真时间 |
| `nav_mode` | `ekf_odom` | 支持 `ekf_odom` 或 `wheel_odom` |
| `map_yaml_file` | `""` | `ekf_odom` 模式下 Nav2 使用的静态地图 yaml |
| `nav_params_file` | `""` | Nav2 参数覆盖文件；为空时使用包内默认参数 |
| `base_controller_name` | `base_controller` | 底盘控制器名称 |

## 运行

构建并加载环境：

```bash
colcon build --packages-select hanmole_navigation
source install/setup.bash
```

直接启动公共导航入口：

```bash
ros2 launch hanmole_navigation bringup.launch.py \
  robot_version:=v0_2 \
  nav_mode:=ekf_odom \
  map_yaml_file:=/abs/path/to/map.yaml
```

直接启用基于 `/odom` 的简化版导航：

```bash
ros2 launch hanmole_navigation bringup.launch.py \
  robot_version:=v0_1 \
  nav_mode:=wheel_odom
```
