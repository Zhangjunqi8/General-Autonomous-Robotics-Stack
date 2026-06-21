#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Randomly loop through selected HanMole navigation targets."""

from __future__ import annotations

import argparse
import random
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from hanmole_msgs.srv import SetString
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage as NavFeedbackMessage

try:
    import yaml
except ImportError:  # pragma: no cover - ROS 2 environments normally provide PyYAML.
    yaml = None


DEFAULT_TARGETS = ["矿泉水区", "HOME", "补货备货仓", "收银台"]
TERMINAL_STATES = {"succeeded", "failed", "canceled"}
ACTIVE_STATES = {"waiting_for_nav2", "navigating", "canceling"}


def default_target_map_path() -> Path:
    return Path(__file__).resolve().parents[1] / "config" / "target_map.yaml"


def load_target_names(target_map: Path, group: Optional[str]) -> List[str]:
    if yaml is None:
        raise RuntimeError("PyYAML is required to read target_map.yaml")
    with target_map.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    groups = data.get("target_groups", {}) or {}
    selected_group = group or data.get("default_group") or "default"
    targets = groups.get(selected_group)
    if not isinstance(targets, dict):
        available = ", ".join(sorted(groups.keys())) or "<none>"
        raise RuntimeError(f"target group not found: {selected_group}; available groups: {available}")
    return list(targets.keys())


def seconds_from_duration(duration) -> float:
    return float(getattr(duration, "sec", 0)) + float(getattr(duration, "nanosec", 0)) / 1e9


class RandomTargetNavLoop(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("random_target_nav_loop")
        self.args = args
        self.targets = self._resolve_targets(args)
        self.service_name = args.service
        self.status_topic = args.status_topic
        self.target_state_topic = args.target_state_topic
        self.feedback_topic = args.feedback_topic
        self.interval = args.interval
        self.cooldown_ticks = max(1, int(round(args.cooldown / args.interval)))
        self.cooldown_remaining = 0

        self.nav_status = "unknown"
        self.target_state = "unknown"
        self.feedback_summary = "等待 feedback"
        self.current_target: Optional[str] = None
        self.previous_target: Optional[str] = None
        self.active = False
        self.waiting_service_response = False
        self.observed_active_state = False
        self.elapsed_ticks = 0
        self.last_result = "idle"

        latched_qos = QoSProfile(depth=10)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.status_sub = self.create_subscription(
            String, self.status_topic, self._on_status, latched_qos
        )
        self.target_state_sub = self.create_subscription(
            String, self.target_state_topic, self._on_target_state, latched_qos
        )
        self.feedback_sub = self.create_subscription(
            NavFeedbackMessage,
            self.feedback_topic,
            self._on_feedback,
            QoSProfile(depth=10),
        )
        self.client = self.create_client(SetString, self.service_name)
        self.timer = self.create_timer(self.interval, self._on_timer)

        self.get_logger().info("随机导航循环脚本已启动")
        self.get_logger().info(f"目标池: {', '.join(self.targets)}")
        self.get_logger().info(f"服务: {self.service_name}")
        self.get_logger().info(f"状态话题: {self.status_topic}, {self.target_state_topic}")
        self.get_logger().info(f"反馈话题: {self.feedback_topic}")

    def _resolve_targets(self, args: argparse.Namespace) -> List[str]:
        target_map = Path(args.target_map).expanduser().resolve()
        target_names = set(load_target_names(target_map, args.group))
        requested = list(args.targets)
        missing = [target for target in requested if target not in target_names]
        if missing:
            raise RuntimeError(
                "target_map.yaml 中找不到目标: "
                + ", ".join(missing)
                + f"; 文件: {target_map}"
            )
        return requested

    def _on_status(self, message: String) -> None:
        self.nav_status = message.data.strip() or "unknown"
        if self.active and self.nav_status in ACTIVE_STATES:
            self.observed_active_state = True
        if self.active and self.nav_status in TERMINAL_STATES:
            self._finish_current(self.nav_status)

    def _on_target_state(self, message: String) -> None:
        self.target_state = message.data.strip() or "unknown"

    def _on_feedback(self, message: NavFeedbackMessage) -> None:
        feedback = message.feedback
        parts = []
        distance_remaining = getattr(feedback, "distance_remaining", None)
        if distance_remaining is not None:
            parts.append(f"剩余距离 {distance_remaining:.3f}m")
        navigation_time = getattr(feedback, "navigation_time", None)
        if navigation_time is not None:
            parts.append(f"导航耗时 {seconds_from_duration(navigation_time):.1f}s")
        estimated_time = getattr(feedback, "estimated_time_remaining", None)
        if estimated_time is not None:
            parts.append(f"预计剩余 {seconds_from_duration(estimated_time):.1f}s")
        recoveries = getattr(feedback, "number_of_recoveries", None)
        if recoveries is not None:
            parts.append(f"恢复次数 {recoveries}")
        self.feedback_summary = " | ".join(parts) if parts else "收到 feedback"

    def _on_timer(self) -> None:
        if self.active:
            self.elapsed_ticks += 1
            if (
                self.observed_active_state
                and self.nav_status == "idle"
                and self.target_state == "idle"
            ):
                self._finish_current("idle_after_active")
                return
            self._print_status()
            return

        if self.waiting_service_response:
            self._print_status(prefix="等待服务响应")
            return

        if self.cooldown_remaining > 0:
            self.cooldown_remaining -= 1
            self._print_status(prefix="等待下一轮")
            return

        if not self.client.service_is_ready():
            self._print_status(prefix="等待导航服务")
            self.client.wait_for_service(timeout_sec=0.1)
            return

        self._send_next_goal()

    def _send_next_goal(self) -> None:
        target = self._choose_target()
        request = SetString.Request()
        request.data = target
        self.current_target = target
        self.previous_target = target
        self.elapsed_ticks = 0
        self.observed_active_state = False
        self.feedback_summary = "等待 feedback"
        self.waiting_service_response = True
        self.last_result = "service_calling"
        print(f"\n>>> 开始导航 -> {target}", flush=True)
        future = self.client.call_async(request)
        future.add_done_callback(lambda done_future: self._on_service_response(target, done_future))

    def _choose_target(self) -> str:
        if len(self.targets) == 1 or self.previous_target is None:
            return random.choice(self.targets)
        candidates = [target for target in self.targets if target != self.previous_target]
        return random.choice(candidates)

    def _on_service_response(self, target: str, future) -> None:
        self.waiting_service_response = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - log service exceptions clearly for operators.
            self.last_result = f"service_error: {exc}"
            print(f"!!! 导航服务调用异常 -> {target}: {exc}", flush=True)
            self._finish_current("service_error")
            return

        result_text = getattr(response, "result", "") or getattr(response, "message", "")
        if not getattr(response, "success", False):
            self.last_result = f"service_rejected: {result_text}"
            print(f"!!! 导航服务拒绝 -> {target}: {result_text}", flush=True)
            self._finish_current("service_rejected")
            return

        self.last_result = result_text or "goal accepted"
        print(f"服务已接受 -> {target}: {self.last_result}", flush=True)
        if "already at target" in self.last_result:
            self._finish_current("already_at_target")
            return
        self.active = True

    def _finish_current(self, result: str) -> None:
        target = self.current_target or "<none>"
        elapsed = self.elapsed_ticks * self.interval
        self.active = False
        self.waiting_service_response = False
        self.current_target = None
        self.observed_active_state = False
        self.cooldown_remaining = self.cooldown_ticks
        self.last_result = result
        print(
            f"<<< 导航结束 -> {target} | 结果: {result} | 耗时: {elapsed:.1f}s",
            flush=True,
        )

    def _print_status(self, prefix: str = "状态") -> None:
        target = self.current_target or "-"
        elapsed = self.elapsed_ticks * self.interval
        print(
            f"[{prefix}] 当前目标: {target} | nav_status: {self.nav_status} | "
            f"target_state: {self.target_state} | 耗时: {elapsed:.1f}s | {self.feedback_summary}",
            flush=True,
        )


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="随机循环调用 /hanmole/agv/navigate_to_pose，并等待上一轮导航完成。"
    )
    parser.add_argument(
        "--target-map",
        default=str(default_target_map_path()),
        help="target_map.yaml 路径。",
    )
    parser.add_argument("--group", default=None, help="target_map.yaml 中的 target group。")
    parser.add_argument(
        "--targets",
        nargs="+",
        default=DEFAULT_TARGETS,
        help="参与随机切换的目标名。",
    )
    parser.add_argument(
        "--service",
        default="/hanmole/agv/navigate_to_pose",
        help="导航目标服务名。",
    )
    parser.add_argument("--status-topic", default="/nav_status", help="导航状态话题。")
    parser.add_argument(
        "--target-state-topic",
        default="/hanmole_navigation/target_state",
        help="目标状态话题。",
    )
    parser.add_argument(
        "--feedback-topic",
        default="/hanmole/agv/nav_feedback",
        help="Nav2 feedback 转发话题。",
    )
    parser.add_argument("--interval", type=float, default=1.0, help="终端状态输出周期，单位秒。")
    parser.add_argument("--cooldown", type=float, default=2.0, help="到达目标后再等多久才继续下一个目标，单位秒。")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    rclpy.init(args=None)
    node: Optional[RandomTargetNavLoop] = None
    try:
        node = RandomTargetNavLoop(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C，退出随机导航循环。", flush=True)
    except Exception as exc:  # noqa: BLE001 - command-line script should print operator-friendly errors.
        print(f"启动失败: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


