# hanmole_navigation

`hanmole_navigation` 是瀚墨机器人的导航包，对外提供统一的导航入口：

- `nav_mode:=ekf_odom` 时装配 Nav2 + EKF 导航链
- `nav_mode:=wheel_odom` 时装配使用纯轮式里程计 `/odom` 的 Nav2 导航链
- 导航可视化与参数资产

`launch/bringup.launch.py` 是本包唯一对外文档化的导航聚合入口。它负责持有导航参数真值、地图路径接口和 Nav2 场景装配。

## 定位

本包是 **导航聚合入口 + Nav2 资产包**。

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
- `hanmole_navigation`：导航参数、地图、行为树、可视化和导航场景装配
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
| `nav_map_yaml_file` | `""` | Nav2 / localization 使用的静态地图 yaml；当前两种 `nav_mode` 都要求非空 |
| `nav_params_file` | `""` | Nav2 参数覆盖文件；为空时使用包内默认参数 |
| `base_controller_name` | `base_controller` | 底盘控制器名称 |
| `use_composition` | `False` | 是否以组件容器方式启动 Nav2 节点 |

## 构建与测试

以下命令以当前源码中的 `launch/bringup.launch.py`、`nav2_target_gateway_node`、
`target_catalog_publisher_node` 和 `scripts/random_target_nav_loop.py` 为准。
install 空间中如果残留历史可执行，不作为本文档的有效入口。

构建并加载环境：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to hanmole_navigation --symlink-install
source install/setup.bash
```

运行本包测试：

```bash
colcon test --packages-select hanmole_navigation
colcon test-result --verbose
```

## 启动导航

直接启动公共导航入口，启用 `ekf_odom` 模式：

```bash
ros2 launch hanmole_navigation bringup.launch.py \
  robot_version:=v0_2 \
  nav_mode:=ekf_odom \
  nav_map_yaml_file:=/abs/path/to/map.yaml
```

直接启动公共导航入口，启用 `wheel_odom` 模式：

```bash
ros2 launch hanmole_navigation bringup.launch.py \
  robot_version:=v0_1 \
  nav_mode:=wheel_odom \
  nav_map_yaml_file:=/abs/path/to/map.yaml
```

如果需要更容易调试的非组合模式：

```bash
ros2 launch hanmole_navigation bringup.launch.py \
  robot_version:=v0_2 \
  nav_mode:=ekf_odom \
  nav_map_yaml_file:=/abs/path/to/map.yaml \
  use_composition:=False
```

## 常用命令

检查导航相关接口是否已上线：

```bash
ros2 node list | rg 'nav2_target_gateway|target_catalog_publisher'
ros2 service list | rg '/hanmole/agv/'
ros2 action list | rg '/navigate_to_pose'
```

查看可用目标点与导航状态：

```bash
ros2 topic echo /hanmole/agv/pose_list
ros2 topic echo /nav_status
ros2 topic echo /hanmole_navigation/target_state
ros2 topic echo /hanmole/agv/nav_feedback
```

### 通过本包包装层接口调用

`nav2_target_gateway_node` 对外提供“命名目标点导航”包装层。
其他服务如果不想自己维护目标点坐标，优先调用下面这组 service：

发布初始位姿：

```bash
ros2 service call /hanmole/agv/set_initial_pose std_srvs/srv/Trigger "{}"
```

通过命名目标点发起导航：

```bash
ros2 service call /hanmole/agv/navigate_to_pose \
  hanmole_msgs/srv/SetString \
  "{data: '矿泉水区'}"
```

切换到另一个命名目标点：

```bash
ros2 service call /hanmole/agv/navigate_to_pose \
  hanmole_msgs/srv/SetString \
  "{data: '收银台'}"
```

取消当前导航：

```bash
ros2 service call /hanmole/agv/navigate_cancel std_srvs/srv/Trigger "{}"
```

### 直接调用原生 Nav2 action

上面的 `/hanmole/agv/navigate_to_pose` 本质上只是对底层
`/navigate_to_pose` action 的一层“目标名 -> Pose”包装。
如果调用方已经知道精确位姿，可以直接调用原生 Nav2 action。

直接发送目标，并在终端打印 feedback 与最终结果。下面示例使用当前
`config/target_map.yaml` 中 `HOME` 的位姿：

```bash
ros2 action send_goal -f /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: -0.1176, y: 0.1475, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.0195987451, w: 0.9998079261}}}, behavior_tree: ''}"
```

直接监听底层 action status：

```bash
ros2 topic echo /navigate_to_pose/_action/status
```

当前 ROS 2 CLI 只提供 `ros2 action send_goal`，没有单独的
`ros2 action cancel` 子命令。若调用方需要直接使用原生 Nav2 action
并支持取消，可用下面的最小 `rclpy` 示例：

```bash
python3 - <<'PY'
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose


class Nav2ActionExample(Node):
    def __init__(self) -> None:
        super().__init__('nav2_action_example')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.goal_handle = None

    def feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        distance_remaining = getattr(feedback, 'distance_remaining', None)
        navigation_time = getattr(feedback, 'navigation_time', None)
        if distance_remaining is None:
            self.get_logger().info('feedback received')
            return
        nav_time_sec = 0.0
        if navigation_time is not None:
            nav_time_sec = float(navigation_time.sec) + float(navigation_time.nanosec) / 1e9
        self.get_logger().info(
            f'distance_remaining={distance_remaining:.3f}m navigation_time={nav_time_sec:.1f}s'
        )

    def run(self) -> None:
        if not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('/navigate_to_pose action server unavailable')

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = -0.1176
        goal.pose.pose.position.y = 0.1475
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = -0.0195987451
        goal.pose.pose.orientation.w = 0.9998079261

        send_future = self.client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_future)
        self.goal_handle = send_future.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            raise RuntimeError('goal rejected by /navigate_to_pose')

        self.get_logger().info('goal accepted, wait 3 seconds then cancel')
        deadline = time.time() + 3.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        cancel_future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)
        cancel_response = cancel_future.result()
        if cancel_response is None or len(cancel_response.goals_canceling) == 0:
            raise RuntimeError('cancel request rejected')

        self.get_logger().info('cancel request accepted, wait for final result')
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self.get_logger().info(
            f'final status={result.status} error_code={result.result.error_code}'
        )


def main() -> None:
    rclpy.init()
    node = Nav2ActionExample()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PY
```

运行随机目标循环脚本：

```bash
ros2 run hanmole_navigation random_target_nav_loop.py \
  --targets HOME 矿泉水区 补货备货仓 收银台
```

查看随机目标循环脚本参数：

```bash
ros2 run hanmole_navigation random_target_nav_loop.py --help
```

## Startup Orchestration

- `nav_mode=ekf_odom` 时，启动编排等待 `/scan` 和 `/odometry/filtered`
- `nav_mode=wheel_odom` 时，启动编排等待 `/scan` 和 `/odom`
- localization 与 navigation 都不是开机即自动激活，而是由 `nav2_target_gateway_node` 分阶段调用 lifecycle manager
- 自动流程顺序为：固定延迟 3 秒 -> 等输入话题新鲜 -> 等 `odom -> base_footprint` -> 启动 localization -> 自动发布 `/initialpose` -> 等 `map -> base_footprint` -> 启动 navigation
- 在导航栈进入 ready 前，`/hanmole/agv/navigate_to_pose` 会返回 `navigation stack not ready`
