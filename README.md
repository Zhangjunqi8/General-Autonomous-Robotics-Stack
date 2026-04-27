# hanmole_navigation

`hanmole_navigation` 是瀚墨机器人的导航包，基于 Nav2 提供：

- 地图定位
- 路径规划
- 局部控制
- 导航可视化

## 定位

本包是 **Nav2 资产与适配包**，不是第二层系统级 bringup。

它应当持有：

- Nav2 参数文件
- 地图文件

- 行为树配置
- RViz 配置
- 与底盘版本相关的少量导航适配节点
- 导航入口 launch

系统级启动统一由 `hanmole_bringup` 负责，`hanmole_bringup` 应直接 include 官方 Nav2 bringup，并装载本包中的参数与资源。

## 当前机器人版本支持

### `v0_1`

- 底盘类型：麦克纳姆轮
- 底盘控制器：社区 `mecanum_drive_controller/MecanumDriveController`
- Jazzy 下需将 `/cmd_vel` 的 `geometry_msgs/msg/Twist` 适配为 `<controller>/reference` 的 `geometry_msgs/msg/TwistStamped`

因此，`v0_1` 的导航最小链路为：

```text
Nav2 -> /cmd_vel -> hanmole_navigation adapter -> /base_controller/reference
```

### `v0_2`

- 底盘类型：四转四驱
- 底盘控制器：`hanmole_controllers/SwerveDriverController`

`v0_2` 不需要复用麦克纳姆适配逻辑。

## 与其他包的边界

- `hanmole_navigation` 不实现底盘控制算法
- `hanmole_navigation` 不直接访问 `hanmole_sdk`
- `hanmole_navigation` 不负责建图
- `hanmole_navigation` 不负责多机器人协同

职责分工如下：

- `hanmole_bringup`：系统级 launch orchestration
- `hanmole_navigation`：Nav2 参数、地图、行为树、可视化、导航适配
- `hanmole_slam`：建图与地图保存
- `hanmole_multi_robot`：多机器人协同
- `hanmole_controllers`：底盘执行控制器

## 迁移方向

本包已经承接原 `hanmole_nav_bringup` 的导航资产。后续应继续完善以下内容：

- `v0_1 / v0_2` 的 Nav2 参数
- 地图资产
- 行为树 XML
- RViz 配置
- `v0_1` 的 `/cmd_vel` 适配节点

当前 `v0_1 / v0_2` 的 Nav2 参数与 `v0_1` 的 `/cmd_vel` 适配已经迁入本包。导航主线应只保留 `hanmole_navigation`，不再保留第二层 Nav2 包装。
