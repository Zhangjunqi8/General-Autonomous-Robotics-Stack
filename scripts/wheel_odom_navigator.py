#!/usr/bin/env python3
"""基于原始轮式里程计 `/odom` 的简化版 NavigateToPose 实现。"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


@dataclass
class Pose2D:
    """平面位姿。"""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def normalize_angle(angle: float) -> float:
    """将角度归一化到 `[-pi, pi]`。"""

    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """将四元数转换为平面 yaw。"""

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """根据 yaw 构造四元数。"""

    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def compose_pose(origin: Pose2D, local_pose: Pose2D) -> Pose2D:
    """将局部原始轮式里程计位姿映射到导航全局位姿。"""

    cos_yaw = math.cos(origin.yaw)
    sin_yaw = math.sin(origin.yaw)
    return Pose2D(
        x=origin.x + cos_yaw * local_pose.x - sin_yaw * local_pose.y,
        y=origin.y + sin_yaw * local_pose.x + cos_yaw * local_pose.y,
        yaw=normalize_angle(origin.yaw + local_pose.yaw),
    )


def invert_compose_pose(target_global: Pose2D, local_pose: Pose2D) -> Pose2D:
    """根据当前局部原始轮式里程计位姿反推全局原点。"""

    cos_yaw = math.cos(target_global.yaw)
    sin_yaw = math.sin(target_global.yaw)
    return Pose2D(
        x=target_global.x - cos_yaw * local_pose.x + sin_yaw * local_pose.y,
        y=target_global.y - sin_yaw * local_pose.x - cos_yaw * local_pose.y,
        yaw=normalize_angle(target_global.yaw - local_pose.yaw),
    )


class WheelOdomNavigator(Node):
    """在 `wheel_odom` 模式下提供基于 `/odom` 的 Nav2 风格 NavigateToPose。"""

    def __init__(self) -> None:
        super().__init__("hanmole_wheel_odom_navigator")

        callback_group = ReentrantCallbackGroup()

        self._odom_topic = self.declare_parameter(
            "odom_topic",
            "/odom",
        ).value
        self._cmd_vel_topic = self.declare_parameter(
            "cmd_vel_topic",
            "/cmd_vel",
        ).value
        self._initial_pose_topic = self.declare_parameter(
            "initial_pose_topic",
            "/initialpose",
        ).value
        self._action_name = self.declare_parameter(
            "action_name",
            "navigate_to_pose",
        ).value
        self._command_frame_id = self.declare_parameter(
            "command_frame_id",
            "base_footprint",
        ).value
        self._global_frame_id = self.declare_parameter(
            "global_frame_id",
            "map",
        ).value
        self._reset_odometry_service = self.declare_parameter(
            "reset_odometry_service",
            "/hanmole/motion/agv/reset_odometry",
        ).value
        self._xy_goal_tolerance = float(
            self.declare_parameter("xy_goal_tolerance", 0.12).value
        )
        self._yaw_goal_tolerance = float(
            self.declare_parameter("yaw_goal_tolerance", 0.15).value
        )
        self._max_linear_speed = abs(
            float(self.declare_parameter("linear_speed_mps", 0.2).value)
        )
        self._min_linear_speed = abs(
            float(self.declare_parameter("min_linear_speed_mps", 0.05).value)
        )
        self._max_angular_speed = abs(
            float(self.declare_parameter("angular_speed_radps", 0.35).value)
        )
        self._min_angular_speed = abs(
            float(self.declare_parameter("min_angular_speed_radps", 0.08).value)
        )
        self._control_rate_hz = float(
            self.declare_parameter("control_rate_hz", 20.0).value
        )
        self._timeout_sec = float(
            self.declare_parameter("timeout_sec", 60.0).value
        )
        self._stop_republish_count = int(
            self.declare_parameter("stop_republish_count", 5).value
        )

        self._origin_pose = Pose2D()
        self._initial_pose: Optional[Pose2D] = None
        self._latest_local_pose: Optional[Pose2D] = None
        self._pose_lock = threading.Lock()

        self._cmd_pub = self.create_publisher(
            TwistStamped,
            self._cmd_vel_topic,
            10,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            self._odom_topic,
            self._on_odom,
            10,
            callback_group=callback_group,
        )
        self._initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self._initial_pose_topic,
            self._on_initial_pose,
            10,
            callback_group=callback_group,
        )
        self._reset_client = self.create_client(
            Trigger,
            self._reset_odometry_service,
            callback_group=callback_group,
        )
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            self._action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )

    def _on_odom(self, msg: Odometry) -> None:
        """缓存最近一帧原始轮式里程计。"""

        pose = msg.pose.pose
        local_pose = Pose2D(
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw=yaw_from_quaternion(
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ),
        )
        with self._pose_lock:
            self._latest_local_pose = local_pose

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """接收 Nav2 标准 `/initialpose` 并重设原始轮式里程计原点。"""

        pose = msg.pose.pose
        requested_pose = Pose2D(
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw=yaw_from_quaternion(
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ),
        )

        with self._pose_lock:
            local_pose = self._latest_local_pose or Pose2D()
            self._origin_pose = invert_compose_pose(requested_pose, local_pose)
            self._initial_pose = requested_pose
            self._global_frame_id = msg.header.frame_id or self._global_frame_id

        self._request_reset_odometry(reset_to_initial_pose=requested_pose)

    def _goal_callback(self, goal_request: NavigateToPose.Goal) -> GoalResponse:
        """始终接受目标，具体执行阶段再决定是否失败。"""

        del goal_request
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        """允许取消当前导航。"""

        del goal_handle
        return CancelResponse.ACCEPT

    def _current_global_pose(self) -> Optional[Pose2D]:
        """读取当前全局位姿。"""

        with self._pose_lock:
            if self._latest_local_pose is None:
                return None
            return compose_pose(self._origin_pose, self._latest_local_pose)

    def _publish_command(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        """发布一帧底盘速度命令。"""

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self._command_frame_id
        cmd.twist.linear.x = linear_x
        cmd.twist.linear.y = linear_y
        cmd.twist.angular.z = angular_z
        self._cmd_pub.publish(cmd)

    def _publish_stop(self) -> None:
        """补发若干帧零速度，确保底盘稳定停车。"""

        for _ in range(max(self._stop_republish_count, 1)):
            self._publish_command(0.0, 0.0, 0.0)
            time.sleep(0.02)

    def _request_reset_odometry(self, reset_to_initial_pose: Optional[Pose2D] = None) -> None:
        """调用 motion 的 reset odom service。"""

        if not self._reset_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning(
                f"reset odometry service {self._reset_odometry_service} is not ready"
            )
            return

        future = self._reset_client.async_send_request(Trigger.Request())

        if reset_to_initial_pose is None:
            return

        def _on_done(done_future) -> None:
            try:
                response = done_future.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"reset odometry call failed: {exc}")
                return

            if not response.success:
                self.get_logger().warning(
                    f"reset odometry rejected: {response.message}"
                )
                return

            with self._pose_lock:
                self._origin_pose = reset_to_initial_pose

        future.add_done_callback(_on_done)

    def _build_feedback(
        self,
        current_pose: Pose2D,
        start_time,
        distance_remaining: float,
        yaw_remaining: float,
    ) -> NavigateToPose.Feedback:
        """构造 Nav2 标准 feedback。"""

        feedback = NavigateToPose.Feedback()
        feedback.current_pose = PoseStamped()
        feedback.current_pose.header.stamp = self.get_clock().now().to_msg()
        feedback.current_pose.header.frame_id = self._global_frame_id
        feedback.current_pose.pose.position.x = current_pose.x
        feedback.current_pose.pose.position.y = current_pose.y
        qx, qy, qz, qw = quaternion_from_yaw(current_pose.yaw)
        feedback.current_pose.pose.orientation.x = qx
        feedback.current_pose.pose.orientation.y = qy
        feedback.current_pose.pose.orientation.z = qz
        feedback.current_pose.pose.orientation.w = qw

        elapsed = self.get_clock().now() - start_time
        feedback.navigation_time = elapsed.to_msg()

        remaining_time_sec = (
            distance_remaining / max(self._max_linear_speed, 1e-6)
            + abs(yaw_remaining) / max(self._max_angular_speed, 1e-6)
        )
        feedback.estimated_time_remaining = Duration(
            seconds=max(remaining_time_sec, 0.0)
        ).to_msg()
        feedback.number_of_recoveries = 0
        feedback.distance_remaining = float(distance_remaining)
        return feedback

    def _goal_matches_initial_pose(self, goal_pose: Pose2D) -> bool:
        """判断当前目标是否就是保存的初始位姿。"""

        with self._pose_lock:
            if self._initial_pose is None:
                return False
            initial_pose = self._initial_pose

        distance = math.hypot(goal_pose.x - initial_pose.x, goal_pose.y - initial_pose.y)
        yaw_error = abs(normalize_angle(goal_pose.yaw - initial_pose.yaw))
        return distance <= self._xy_goal_tolerance and yaw_error <= self._yaw_goal_tolerance

    def _execute_callback(self, goal_handle) -> NavigateToPose.Result:
        """执行简化版 NavigateToPose：先平移，再转向。"""

        result = NavigateToPose.Result()
        goal = goal_handle.request.pose
        goal_yaw = yaw_from_quaternion(
            float(goal.pose.orientation.x),
            float(goal.pose.orientation.y),
            float(goal.pose.orientation.z),
            float(goal.pose.orientation.w),
        )
        goal_pose = Pose2D(
            x=float(goal.pose.position.x),
            y=float(goal.pose.position.y),
            yaw=goal_yaw,
        )

        phase = "translate"
        start_time = self.get_clock().now()
        rate_sec = 1.0 / max(self._control_rate_hz, 1.0)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._publish_stop()
                goal_handle.canceled()
                result.error_code = 1
                result.error_msg = "goal canceled"
                return result

            if self.get_clock().now() - start_time > Duration(seconds=self._timeout_sec):
                self._publish_stop()
                goal_handle.abort()
                result.error_code = 1
                result.error_msg = "navigation timeout"
                return result

            current_pose = self._current_global_pose()
            if current_pose is None:
                time.sleep(rate_sec)
                continue

            dx = goal_pose.x - current_pose.x
            dy = goal_pose.y - current_pose.y
            distance = math.hypot(dx, dy)
            yaw_error = normalize_angle(goal_pose.yaw - current_pose.yaw)

            if phase == "translate":
                if distance <= self._xy_goal_tolerance:
                    self._publish_stop()
                    phase = "rotate"
                    time.sleep(rate_sec)
                    continue

                cos_yaw = math.cos(current_pose.yaw)
                sin_yaw = math.sin(current_pose.yaw)
                local_x = cos_yaw * dx + sin_yaw * dy
                local_y = -sin_yaw * dx + cos_yaw * dy
                linear_norm = math.hypot(local_x, local_y)
                scale = 0.0
                if linear_norm > 1e-9:
                    scale = min(
                        self._max_linear_speed,
                        max(self._min_linear_speed, linear_norm),
                    ) / linear_norm
                self._publish_command(local_x * scale, local_y * scale, 0.0)
            else:
                if abs(yaw_error) <= self._yaw_goal_tolerance:
                    self._publish_stop()
                    goal_handle.succeed()
                    if self._goal_matches_initial_pose(goal_pose):
                        self._request_reset_odometry(reset_to_initial_pose=goal_pose)
                    result.error_code = NavigateToPose.Result.NONE
                    result.error_msg = ""
                    return result

                angular_speed = math.copysign(
                    min(
                        self._max_angular_speed,
                        max(self._min_angular_speed, abs(yaw_error)),
                    ),
                    yaw_error,
                )
                self._publish_command(0.0, 0.0, angular_speed)

            feedback = self._build_feedback(current_pose, start_time, distance, yaw_error)
            goal_handle.publish_feedback(feedback)
            time.sleep(rate_sec)

        self._publish_stop()
        goal_handle.abort()
        result.error_code = 1
        result.error_msg = "navigation interrupted"
        return result


def main() -> int:
    """节点入口。"""

    rclpy.init()
    node = WheelOdomNavigator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号，停止 wheel odom navigator。")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
