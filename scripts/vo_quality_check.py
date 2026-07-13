#!/usr/bin/env python3
"""Measure stereo image synchronization and VO odometry stability."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Iterable, List, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def percentile(values: List[float], pct: float) -> Optional[float]:
    if not values:
        return None
    if len(values) == 1:
        return values[0]
    ordered = sorted(values)
    rank = (len(ordered) - 1) * pct / 100.0
    low = int(math.floor(rank))
    high = int(math.ceil(rank))
    if low == high:
        return ordered[low]
    return ordered[low] + (ordered[high] - ordered[low]) * (rank - low)


def summarize(values: List[float]) -> Dict[str, Optional[float]]:
    if not values:
        return {
            "count": 0,
            "mean": None,
            "p50": None,
            "p95": None,
            "p99": None,
            "max": None,
        }
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "p50": percentile(values, 50.0),
        "p95": percentile(values, 95.0),
        "p99": percentile(values, 99.0),
        "max": max(values),
    }


def format_ms(value: Optional[float]) -> str:
    return "n/a" if value is None else f"{value * 1000.0:.3f} ms"


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(b - a), math.cos(b - a))


@dataclass
class TimedPose:
    stamp: float
    x: float
    y: float
    z: float
    yaw: float


class VoQualityCheck(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("vo_quality_check")
        self.args = args
        self.start_monotonic = time.monotonic()
        self.left_queue: Deque[float] = deque()
        self.right_queue: Deque[float] = deque()
        self.left_stamps: List[float] = []
        self.right_stamps: List[float] = []
        self.pair_dts: List[float] = []
        self.dropped_left = 0
        self.dropped_right = 0
        self.odom_poses: List[TimedPose] = []

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=args.qos_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=args.qos_depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Image, args.left_topic, self.left_cb, image_qos)
        self.create_subscription(Image, args.right_topic, self.right_cb, image_qos)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, odom_qos)
        self.create_timer(args.report_interval, self.report_progress)
        self.create_timer(0.2, self.stop_if_done)

        self.get_logger().info(
            "VO quality check running for %.1f s: left=%s right=%s odom=%s"
            % (args.duration, args.left_topic, args.right_topic, args.odom_topic)
        )

    def left_cb(self, msg: Image) -> None:
        stamp = stamp_to_sec(msg.header.stamp)
        self.left_stamps.append(stamp)
        self.left_queue.append(stamp)
        self.trim_queue(self.left_queue)
        self.match_pairs()

    def right_cb(self, msg: Image) -> None:
        stamp = stamp_to_sec(msg.header.stamp)
        self.right_stamps.append(stamp)
        self.right_queue.append(stamp)
        self.trim_queue(self.right_queue)
        self.match_pairs()

    def odom_cb(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.odom_poses.append(
            TimedPose(
                stamp=stamp_to_sec(msg.header.stamp),
                x=float(position.x),
                y=float(position.y),
                z=float(position.z),
                yaw=yaw_from_quaternion(orientation),
            )
        )

    def trim_queue(self, queue: Deque[float]) -> None:
        while len(queue) > self.args.max_queue:
            queue.popleft()

    def match_pairs(self) -> None:
        threshold = self.args.match_threshold
        while self.left_queue and self.right_queue:
            left_time = self.left_queue[0]
            best_index = min(
                range(len(self.right_queue)),
                key=lambda index: abs(self.right_queue[index] - left_time),
            )
            best_dt = abs(self.right_queue[best_index] - left_time)
            if best_dt <= threshold:
                self.pair_dts.append(best_dt)
                self.left_queue.popleft()
                del self.right_queue[best_index]
                continue

            if left_time < self.right_queue[0]:
                self.left_queue.popleft()
                self.dropped_left += 1
            else:
                self.right_queue.popleft()
                self.dropped_right += 1

    def elapsed(self) -> float:
        return time.monotonic() - self.start_monotonic

    def stop_if_done(self) -> None:
        if self.elapsed() >= self.args.duration:
            self.print_summary()
            raise SystemExit(0)

    def report_progress(self) -> None:
        if not self.args.progress:
            return
        self.get_logger().info(
            "elapsed=%.1f s left=%d right=%d pairs=%d dropped_left=%d dropped_right=%d odom=%d"
            % (
                self.elapsed(),
                len(self.left_stamps),
                len(self.right_stamps),
                len(self.pair_dts),
                self.dropped_left,
                self.dropped_right,
                len(self.odom_poses),
            )
        )

    def stamp_gaps(self, stamps: List[float]) -> List[float]:
        if len(stamps) < 2:
            return []
        return [b - a for a, b in zip(stamps, stamps[1:]) if b >= a]

    def rate(self, stamps: List[float]) -> Optional[float]:
        if len(stamps) < 2:
            return None
        span = stamps[-1] - stamps[0]
        if span <= 0.0:
            return None
        return float(len(stamps) - 1) / span

    def odom_summary(self) -> Dict[str, Optional[float]]:
        poses = self.odom_poses
        if not poses:
            return {
                "count": 0,
                "rate_hz": None,
                "final_xy_m": None,
                "final_xyz_m": None,
                "path_length_xy_m": None,
                "max_xy_from_start_m": None,
                "final_yaw_deg": None,
                "expected_distance_m": self.args.expected_distance,
                "distance_error_m": None,
                "distance_error_pct": None,
            }

        first = poses[0]
        previous = first
        path_length_xy = 0.0
        max_xy = 0.0
        for pose in poses[1:]:
            path_length_xy += math.hypot(pose.x - previous.x, pose.y - previous.y)
            max_xy = max(max_xy, math.hypot(pose.x - first.x, pose.y - first.y))
            previous = pose

        last = poses[-1]
        final_xy = math.hypot(last.x - first.x, last.y - first.y)
        final_xyz = math.sqrt(
            (last.x - first.x) ** 2 + (last.y - first.y) ** 2 + (last.z - first.z) ** 2
        )
        rate_hz = None
        span = poses[-1].stamp - poses[0].stamp
        if len(poses) > 1 and span > 0.0:
            rate_hz = float(len(poses) - 1) / span

        distance_error = None
        distance_error_pct = None
        distance_error_xyz = None
        distance_error_xyz_pct = None
        if self.args.expected_distance is not None:
            distance_error = final_xy - self.args.expected_distance
            distance_error_xyz = final_xyz - self.args.expected_distance
            if self.args.expected_distance != 0.0:
                distance_error_pct = distance_error / self.args.expected_distance * 100.0
                distance_error_xyz_pct = (
                    distance_error_xyz / self.args.expected_distance * 100.0
                )

        return {
            "count": len(poses),
            "rate_hz": rate_hz,
            "final_xy_m": final_xy,
            "final_xyz_m": final_xyz,
            "path_length_xy_m": path_length_xy,
            "max_xy_from_start_m": max_xy,
            "final_yaw_deg": math.degrees(angle_delta(first.yaw, last.yaw)),
            "expected_distance_m": self.args.expected_distance,
            "distance_error_m": distance_error,
            "distance_error_pct": distance_error_pct,
            "distance_error_xyz_m": distance_error_xyz,
            "distance_error_xyz_pct": distance_error_xyz_pct,
        }

    def build_summary(self) -> Dict[str, object]:
        duration = max(self.elapsed(), 1.0e-9)
        pair_count = len(self.pair_dts)
        rejected = self.dropped_left + self.dropped_right
        total_pair_candidates = pair_count + rejected
        publish_ratio = None
        if total_pair_candidates > 0:
            publish_ratio = pair_count / float(total_pair_candidates)

        return {
            "duration_sec": self.elapsed(),
            "left": {
                "topic": self.args.left_topic,
                "count": len(self.left_stamps),
                "rate_hz": self.rate(self.left_stamps),
                "arrival_rate_hz": len(self.left_stamps) / duration,
                "gap_sec": summarize(self.stamp_gaps(self.left_stamps)),
            },
            "right": {
                "topic": self.args.right_topic,
                "count": len(self.right_stamps),
                "rate_hz": self.rate(self.right_stamps),
                "arrival_rate_hz": len(self.right_stamps) / duration,
                "gap_sec": summarize(self.stamp_gaps(self.right_stamps)),
            },
            "stereo_pairs": {
                "match_threshold_sec": self.args.match_threshold,
                "count": pair_count,
                "dropped_left": self.dropped_left,
                "dropped_right": self.dropped_right,
                "match_ratio": publish_ratio,
                "dt_sec": summarize(self.pair_dts),
                "over_10ms": sum(1 for value in self.pair_dts if value > 0.010),
                "over_25ms": sum(1 for value in self.pair_dts if value > 0.025),
            },
            "odom": self.odom_summary(),
        }

    def print_summary(self) -> None:
        summary = self.build_summary()
        pairs = summary["stereo_pairs"]
        left = summary["left"]
        right = summary["right"]
        odom = summary["odom"]

        print("\nVO quality summary")
        print("==================")
        print(
            "left:  count=%d rate=%s gap_p95=%s gap_max=%s"
            % (
                left["count"],
                "n/a" if left["rate_hz"] is None else f"{left['rate_hz']:.2f} Hz",
                format_ms(left["gap_sec"]["p95"]),
                format_ms(left["gap_sec"]["max"]),
            )
        )
        print(
            "right: count=%d rate=%s gap_p95=%s gap_max=%s"
            % (
                right["count"],
                "n/a" if right["rate_hz"] is None else f"{right['rate_hz']:.2f} Hz",
                format_ms(right["gap_sec"]["p95"]),
                format_ms(right["gap_sec"]["max"]),
            )
        )
        print(
            "pairs: count=%d dropped_left=%d dropped_right=%d match_ratio=%s"
            % (
                pairs["count"],
                pairs["dropped_left"],
                pairs["dropped_right"],
                "n/a" if pairs["match_ratio"] is None else f"{pairs['match_ratio'] * 100.0:.1f}%",
            )
        )
        print(
            "pair dt: mean=%s p50=%s p95=%s p99=%s max=%s over_10ms=%d over_25ms=%d"
            % (
                format_ms(pairs["dt_sec"]["mean"]),
                format_ms(pairs["dt_sec"]["p50"]),
                format_ms(pairs["dt_sec"]["p95"]),
                format_ms(pairs["dt_sec"]["p99"]),
                format_ms(pairs["dt_sec"]["max"]),
                pairs["over_10ms"],
                pairs["over_25ms"],
            )
        )
        print(
            "odom: count=%d rate=%s final_xy=%.4f m final_xyz=%.4f m "
            "path_xy=%.4f m max_xy=%.4f m yaw_delta=%.3f deg"
            % (
                odom["count"],
                "n/a" if odom["rate_hz"] is None else f"{odom['rate_hz']:.2f} Hz",
                odom["final_xy_m"] or 0.0,
                odom["final_xyz_m"] or 0.0,
                odom["path_length_xy_m"] or 0.0,
                odom["max_xy_from_start_m"] or 0.0,
                odom["final_yaw_deg"] or 0.0,
            )
        )
        if odom["expected_distance_m"] is not None:
            print(
                "distance error: expected=%.4f m xy_error=%.4f m xy_error_pct=%.2f%% "
                "xyz_error=%.4f m xyz_error_pct=%.2f%%"
                % (
                    odom["expected_distance_m"],
                    odom["distance_error_m"] or 0.0,
                    odom["distance_error_pct"] or 0.0,
                    odom["distance_error_xyz_m"] or 0.0,
                    odom["distance_error_xyz_pct"] or 0.0,
                )
            )

        if self.args.json:
            print("\nJSON")
            print(json.dumps(summary, indent=2, sort_keys=True))


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Measure stereo image sync and VO odometry drift/scale."
    )
    parser.add_argument(
        "--left-topic",
        default="/hanmole/sensor/camera_head_left/image_raw",
        help="left camera image topic",
    )
    parser.add_argument(
        "--right-topic",
        default="/hanmole/sensor/camera_head_right/image_raw",
        help="right camera image topic",
    )
    parser.add_argument("--odom-topic", default="/vio/odom", help="VO odometry topic")
    parser.add_argument("--duration", type=float, default=60.0, help="measurement seconds")
    parser.add_argument(
        "--match-threshold",
        type=float,
        default=0.025,
        help="max left/right header stamp delta for a valid stereo pair, seconds",
    )
    parser.add_argument("--max-queue", type=int, default=30, help="max pending image stamps")
    parser.add_argument("--qos-depth", type=int, default=30, help="ROS QoS queue depth")
    parser.add_argument(
        "--expected-distance",
        type=float,
        default=None,
        help="known straight-line movement distance in meters for scale error",
    )
    parser.add_argument(
        "--report-interval",
        type=float,
        default=5.0,
        help="progress report interval, seconds",
    )
    parser.add_argument("--progress", action="store_true", help="print periodic progress")
    parser.add_argument("--json", action="store_true", help="also print machine-readable JSON")
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    rclpy.init(args=list(argv) if argv is not None else None)
    parser = build_arg_parser()
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:] if argv is None else list(argv))

    node = VoQualityCheck(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.print_summary()
    except SystemExit as exc:
        node.destroy_node()
        rclpy.shutdown()
        return int(exc.code or 0)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
