# 固定路线复现导航 README

本文档说明 `hanmole_navigation` 中的固定路线录制/复现功能。该功能不使用 Nav2 的全局规划器和局部控制器，而是用于 AGV 式固定路线：人工录制一条完整路线，在路线中标记若干目标点，之后沿这条路线复现，到指定目标点停止。

安全规则：`footprint` 外扩 `safety_margin_m` 范围内检测到障碍物就强制停车；障碍物消失并连续安全一段时间后，继续沿原路线复现。不做绕障、不做重新规划。

## 1. 核心概念

推荐只录一条完整主路线，例如：

```text
原点 O -> A -> B -> C -> D -> E
```

保存到 `routes.yaml` 后，路线名可以叫 `main_route`。`A/B/C/D/E` 不是独立路线，而是 `main_route` 里面的目标点。

目标点坐标全部相对于录制原点，不是相对于上一个目标点。例如到了 A 后再去 B，B 仍然是相对于原点 O 的路线索引和坐标。

## 2. 正反向复现

同一条路线现在支持按目标点前后切换：

```bash
# 从当前点去 B
ros2 service call /hanmole_navigation/follow_route hanmole_msgs/srv/FollowRoute "{route_name: 'main_route', target_name: 'B', wait_result: false}"

# 从 B 回 A
ros2 service call /hanmole_navigation/follow_route hanmole_msgs/srv/FollowRoute "{route_name: 'main_route', target_name: 'A', wait_result: false}"
```

判断规则：

- 目标点索引大于当前位置最近索引：`FORWARD`，按录制方向复现。
- 目标点索引小于当前位置最近索引：`REVERSE`，按路线索引反向复现。

`REVERSE` 不是倒车。它只是路线索引反向，机器人会尽量朝运动方向正着走；接近目标点后，再对齐该目标点录制时保存的 yaw。

## 3. 涉及文件

- `/home/hanmole/ros2_ws/src/hanmole_navigation/launch/route_navigation.launch.py`：启动 EKF、路线复现节点和路线录制节点。
- `/home/hanmole/ros2_ws/src/hanmole_navigation/config/route_navigation.yaml`：路线录制、复现、速度、安全停车、反向复现参数。
- `/home/hanmole/ros2_ws/src/hanmole_navigation/config/routes.yaml`：保存录制出来的路线点和目标点。
- `/home/hanmole/ros2_ws/src/hanmole_navigation/src/route_recorder_node.cpp`：录制路线，标记目标点，写入 `routes.yaml`。
- `/home/hanmole/ros2_ws/src/hanmole_navigation/src/route_follower_node.cpp`：读取 `routes.yaml`，复现路线，到目标点停止，处理安全停车。
- `/home/hanmole/ros2_ws/src/hanmole_msgs/srv/RecordRoute.srv`：开始录制路线服务。
- `/home/hanmole/ros2_ws/src/hanmole_msgs/srv/SaveRoute.srv`：保存路线服务。
- `/home/hanmole/ros2_ws/src/hanmole_msgs/srv/MarkRouteTarget.srv`：录制过程中标记目标点服务。
- `/home/hanmole/ros2_ws/src/hanmole_msgs/srv/FollowRoute.srv`：复现路线服务，支持指定 `target_name`。

## 4. 编译

改了 `hanmole_navigation` 的 C++ 代码后，需要重新编译：

```bash
cd /home/hanmole/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select hanmole_navigation --symlink-install
source install/setup.bash
```

## 5. 启动前检查

先启动 motion、sensor，并确保下面话题正常：

```bash
ros2 topic echo /odom --once
ros2 topic echo /imu/data --once
ros2 topic echo /scan --once
ros2 topic info /cmd_vel -v
```

`route_navigation.launch.py` 会启动 EKF，所以启动后应有：

```bash
ros2 topic echo /odometry/filtered --once
```

注意：复现节点会直接发布 `/cmd_vel`，类型是 `geometry_msgs/msg/TwistStamped`。复现时不要让 VR、teleop、Nav2 controller 同时发布 `/cmd_vel`，否则底盘会收到互相冲突的速度。

## 6. 启动路线功能

```bash
cd /home/hanmole/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=004
ros2 launch hanmole_navigation route_navigation.launch.py
```

该 launch 会启动：

- `ekf_filter_node`
- `route_follower_node`
- `route_recorder_node`

如果不想由它启动 EKF：

```bash
ros2 launch hanmole_navigation route_navigation.launch.py use_ekf:=false
```

## 7. 录制完整路线并标记目标点

### 设置录制原点

当前 `route_recorder` 也配置为读取 `/odom`：

```yaml
route_recorder:
  ros__parameters:
    odom_topic: "/odom"
```

把机器人放在路线原点 O，车头方向摆正，并确认 `/odom` 已有数据：

```bash
ros2 topic echo /odom --once
ros2 service call /hanmole_navigation/route_recorder/set_origin_here std_srvs/srv/Trigger "{}"
```

### 开始录制

```bash
ros2 service call /hanmole_navigation/route_recorder/start_record hanmole_msgs/srv/RecordRoute "{route_name: 'main_route'}"
```

默认采样规则：

- 移动超过 `record_spacing_m`，默认 `0.05m`，追加一个路线点。
- yaw 变化超过 `record_yaw_spacing_deg`，默认 `3°`，追加一个路线点。

### 标记目标点

到达 A 点时：

```bash
ros2 service call /hanmole_navigation/route_recorder/mark_target hanmole_msgs/srv/MarkRouteTarget "{target_name: 'A'}"
```

到达 B/C/D/E 时同理：

```bash
ros2 service call /hanmole_navigation/route_recorder/mark_target hanmole_msgs/srv/MarkRouteTarget "{target_name: 'B'}"
ros2 service call /hanmole_navigation/route_recorder/mark_target hanmole_msgs/srv/MarkRouteTarget "{target_name: 'C'}"
ros2 service call /hanmole_navigation/route_recorder/mark_target hanmole_msgs/srv/MarkRouteTarget "{target_name: 'D'}"
ros2 service call /hanmole_navigation/route_recorder/mark_target hanmole_msgs/srv/MarkRouteTarget "{target_name: 'E'}"
```

`mark_target` 会强制把当前位姿写成一个路线点，并把目标名绑定到这个点的 `index`。

### 保存路线

```bash
ros2 service call /hanmole_navigation/route_recorder/save_route hanmole_msgs/srv/SaveRoute "{route_name: 'main_route'}"
```

查看保存结果：

```bash
cat /home/hanmole/ros2_ws/src/hanmole_navigation/config/routes.yaml
```

## 8. 复现到指定目标点

当前默认复现模式是 `primitive_plan`，录制节点和复现节点都使用 `/odom`：

```yaml
route_follower:
  ros__parameters:
    follow_mode: "primitive_plan"
    odom_topic: "/odom"
route_recorder:
  ros__parameters:
    odom_topic: "/odom"
```

该模式不会连续追踪录制时的每一个路线点，而是只读取目标点的 `x/y/yaw`，自动规划一组动作原语：

```text
原地旋转 90° -> 直行/后退 -> 横移 -> 原地旋转 90° -> ...
```

规划时会自动枚举 `0° / 90° / 180° / -90°` 几个朝向，优先让长距离走直线，减少横移。每个动作仍然用 `/odom` 闭环判断是否到位，不按固定时间开环运行。

### 启动

因为当前配置已经切到 `/odom`，建议不启动 EKF：

```bash
cd /home/hanmole/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=004
ros2 launch hanmole_navigation route_navigation.launch.py use_ekf:=false
```

启动后确认只有路线复现节点在发布 `/cmd_vel`，不要让 VR、teleop、Nav2 controller 同时发速度：

```bash
ros2 topic info /cmd_vel -v
```

### 设置原点

复现前必须把机器人放回录制原点 O，并设置 follower 原点：

```bash
ros2 service call /hanmole_navigation/set_origin_here std_srvs/srv/Trigger "{}"
```

注意：录制原点和复现原点的位置、车头方向要尽量一致。`primitive_plan` 要求当前 yaw 和目标 yaw 接近 `90°` 的整数倍，默认允许 `15°` 内吸附到最近的 `0° / 90° / 180° / -90°`。

### 执行到目标点

去 A：

```bash
ros2 service call /hanmole_navigation/follow_route hanmole_msgs/srv/FollowRoute "{route_name: 'main_route', target_name: 'A', wait_result: false}"
```

去 B：

```bash
ros2 service call /hanmole_navigation/follow_route hanmole_msgs/srv/FollowRoute "{route_name: 'main_route', target_name: 'B', wait_result: false}"
```

从 B 回 A 仍然使用同一条路线：

```bash
ros2 service call /hanmole_navigation/follow_route hanmole_msgs/srv/FollowRoute "{route_name: 'main_route', target_name: 'A', wait_result: false}"
```

如果 `target_name` 留空，会走到路线最后一个点：

```bash
ros2 service call /hanmole_navigation/follow_route hanmole_msgs/srv/FollowRoute "{route_name: 'main_route', target_name: '', wait_result: false}"
```

### 切回旧路线点复现

如需恢复原来的连续路线点跟踪，把 `/home/hanmole/ros2_ws/src/hanmole_navigation/config/route_navigation.yaml` 改成：

```yaml
follow_mode: "route_points"
```

然后重启 `route_navigation.launch.py` 即可。改 YAML 不需要重新编译。

## 9. 状态查看和取消

查看复现状态：

```bash
ros2 topic echo /hanmole_navigation/route_state
```

状态字段含义：

- `route`：当前路线名。
- `target`：当前目标点名，空字符串表示整条路线终点。
- `state`：`FOLLOW_ROUTE`、`BLOCKED`、`SUCCEEDED`、`FAILED`、`CANCELED`。
- `mode`：当前复现模式，`primitive_plan` 或 `route_points`。
- `direction`：`FORWARD` 或 `REVERSE`。
- `index`：当前最近路线点索引。
- `start_index`：本次开始复现时的路线点索引。
- `goal_index`：本次目标点索引。
- `primitive_step`：`primitive_plan` 模式下当前动作原语序号。
- `primitive_total`：`primitive_plan` 模式下本次规划出的动作原语总数。
- `blocked`：安全区是否有障碍物。
- `blocked_points`：进入安全区的雷达点数量。

查看录制状态：

```bash
ros2 topic echo /hanmole_navigation/route_record_state
```

取消复现：

```bash
ros2 service call /hanmole_navigation/cancel_route std_srvs/srv/Trigger "{}"
```

取消录制：

```bash
ros2 service call /hanmole_navigation/route_recorder/cancel_record std_srvs/srv/Trigger "{}"
```

## 10. 安全停车规则

安全停车由 `route_follower_node` 读取 `/scan` 实现。

当前 footprint：

```yaml
[[0.24, 0.17], [0.24, -0.17], [-0.24, -0.17], [-0.24, 0.17]]
```

当前安全外扩：

```yaml
safety_margin_m: 0.05
```

所以当前停车检测矩形大约是：

- x 半长：`0.24 + 0.05 = 0.29m`
- y 半宽：`0.17 + 0.05 = 0.22m`

默认至少 `3` 个 scan 点进入安全区才判定阻塞：

```yaml
safety_min_points: 3
```

障碍物消失后，需要连续安全 `1.0s` 才继续：

```yaml
safety_clear_duration_sec: 1.0
```

## 11. 反向复现参数

相关参数在：

```text
/home/hanmole/ros2_ws/src/hanmole_navigation/config/route_navigation.yaml
```

```yaml
reverse_face_motion_direction: true
final_heading_align_distance_m: 0.15
max_route_start_distance_m: 0.50
require_final_yaw: true
```

含义：

- `reverse_face_motion_direction`：反向索引复现时，车头朝运动方向，正着走。
- `final_heading_align_distance_m`：距离目标点小于该值后，切换到目标点保存的 yaw 做最终对齐。
- `max_route_start_distance_m`：当前位置离路线最近点超过该距离时，拒绝复现，避免乱跑。
- `require_final_yaw`：是否要求最终 yaw 也到达容差内才算完成。

## 12. 常见问题

### Target not found

说明 `routes.yaml` 的该路线下没有这个目标点。检查：

```bash
cat /home/hanmole/ros2_ws/src/hanmole_navigation/config/routes.yaml
```

### Current pose is too far from route

说明机器人当前位置离录制路线太远。请确认机器人在录制路线附近，或者调大：

```yaml
max_route_start_distance_m: 0.50
```

### 一执行就 BLOCKED

可能原因：

- 安全区内确实有障碍物。
- `/scan` 坐标系不是 `base_footprint`，当前第一版默认 scan 点已经在 `base_footprint`。
- 雷达扫到了机器人自身。
- `footprint` 或 `safety_margin_m` 配置不合适。

检查：

```bash
ros2 topic echo /hanmole_navigation/route_state
ros2 topic echo /scan --once
```

### 机器人不动

检查：

```bash
ros2 topic echo /cmd_vel --once
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 topic info /cmd_vel -v
```

如果 `/cmd_vel` 有多个发布者，先停掉 VR、teleop、Nav2 controller，只保留路线复现节点。

### 复现偏差大

可能原因：

- 复现原点和录制原点位置或车头方向不一致。
- `/odom` 漂移。
- 轮速里程计标定不准。
- 麦轮横移打滑。
- 路线太长但没有中途外部校正。

建议先录短路线低速测试，确认原点摆放一致，再逐步增加路线长度。后续如需提高可靠性，可以在 A/B/C/D/E 加 AprilTag、二维码、磁条或定位桩做站点校正。


## 13. 动作原语复现模式

当前默认启用动作原语复现模式：

```yaml
follow_mode: "primitive_plan"
primitive_allow_backward: false
primitive_snap_yaw_to_90deg: true
primitive_yaw_snap_tolerance_deg: 15.0
primitive_strafe_cost: 2.5
primitive_rotate_90_cost_m: 0.4
primitive_backward_cost: 2.0
primitive_min_segment_m: 0.03
primitive_max_strafe_m: 0.05
primitive_xy_tolerance_m: 0.03
primitive_yaw_tolerance_deg: 3.0
primitive_move_yaw_gate_deg: 8.0
primitive_kx: 0.9
primitive_ky: 0.55
primitive_kyaw_hold: 1.0
primitive_kyaw_rotate: 1.0
max_vx_mps: 0.48
max_vy_mps: 0.256
max_wz_radps: 0.88
max_ax_mps2: 0.72
max_ay_mps2: 0.48
max_awz_radps2: 1.60
```

规划逻辑：

- 从目标点读取 `x/y/yaw`，不追踪录制出来的中间路径点。
- 自动枚举机器人可用朝向：`0° / 90° / 180° / -90°`。
- 自动比较多种组合，例如“先转向再直行再横移”和“先横移再直行”。
- `primitive_strafe_cost` 越大，越倾向于少横移、多直行。
- `primitive_allow_backward: false` 时，不允许规划后退动作；如果目标必须后退才能到达，会选择转向后直行。
- `primitive_snap_yaw_to_90deg: true` 时，目标 yaw 会在容差内吸附到最近的 90°整数倍。

执行逻辑：

- `ROTATE_90`：只发布 `angular.z`，原地转到下一个 90°朝向。
- `MOVE_X`：只发布 `linear.x`，同时用小 `angular.z` 保持当前目标 yaw。
- `MOVE_Y`：只发布 `linear.y`，同时用小 `angular.z` 保持当前目标 yaw。
- 到达每个动作目标后会先发 0 速度，再进入下一步。
- 遇到安全区障碍物仍然进入 `BLOCKED` 并停车。

调试时可以查看实际输出：

```bash
ros2 topic echo /cmd_vel
```

正常情况下，旋转时只有 `angular.z` 非零；平移时只有 `linear.x` 或 `linear.y` 为主，`angular.z` 只用于小幅保持朝向。
## NavigateToNamedTarget action 调试接口

当前 navigation 侧保留原来的 `/hanmole_navigation/follow_route` service，同时新增 action：

```bash
ros2 action list -t
ros2 action info /hanmole_navigation/navigate_to_target -t
```

vision/motion 暂时不用改；手动调试时只需要发送 `routes.yaml` 里的目标点名字，例如 `A`、`B`：

```bash
ros2 action send_goal /hanmole_navigation/navigate_to_target hanmole_msgs/action/NavigateToNamedTarget "{target_name: 'A', reload_routes: true}" --feedback
```

执行前仍然需要先设置 follower 原点：

```bash
ros2 service call /hanmole_navigation/set_origin_here std_srvs/srv/Trigger "{}"
```

action 状态风格与 Nav2 类似：

- `GOAL_ACCEPTED`：目标已接收。
- `GOAL_EXECUTING`：正在执行当前 primitive plan。
- `GOAL_BLOCKED`：scan 安全区阻塞，停车等待。
- `GOAL_SUCCEEDED`：到达目标点。
- `GOAL_FAILED`：规划或执行失败。
- `GOAL_CANCELED`：目标被取消。

默认路线名来自 `route_navigation.yaml` 的 `default_route_name`，当前为 `main_route`。也就是说 action goal 只发 `target_name`，不需要发 `route_name`。
