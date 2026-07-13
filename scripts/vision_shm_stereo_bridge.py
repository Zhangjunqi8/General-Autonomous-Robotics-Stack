#!/usr/bin/env python3
import argparse
import mmap
import os
import struct
import time
from dataclasses import dataclass
from typing import Dict, Optional

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image

from hanmole_msgs.msg import ShmFrameInfo


SHM_MAGIC = 0x484D4652
HEADER = struct.Struct("<IIIII4xQ")


@dataclass
class SharedImage:
    seq: int
    width: int
    height: int
    encoding: str
    data: bytes


class VisionShmStereoBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("vision_shm_stereo_bridge")
        self.info_topic = args.info_topic
        self.left_out = args.left_out
        self.right_out = args.right_out
        self.resize_width = args.resize_width
        self.resize_height = args.resize_height
        self.max_pending = args.max_pending
        self.left_frame_id = args.left_frame_id
        self.right_frame_id = args.right_frame_id
        self.pending: Dict[int, Dict[str, SharedImage]] = {}
        self.published_pairs = 0
        self.dropped_frames = 0
        self.read_errors = 0
        self.start_time = time.monotonic()

        info_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=args.qos_depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=args.qos_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.left_pub = self.create_publisher(Image, self.left_out, image_qos)
        self.right_pub = self.create_publisher(Image, self.right_out, image_qos)
        self.create_subscription(ShmFrameInfo, self.info_topic, self.info_cb, info_qos)
        self.create_timer(5.0, self.report)

        self.get_logger().info(
            "vision shm stereo bridge: %s -> %s, %s"
            % (self.info_topic, self.left_out, self.right_out)
        )
        self.get_logger().info(
            "output resize: %dx%d" % (self.resize_width, self.resize_height)
        )

    def info_cb(self, msg: ShmFrameInfo) -> None:
        side = self.side_from_camera_name(msg.camera_name)
        if side is None:
            return
        if msg.encoding != "bgr8":
            self.dropped_frames += 1
            self.get_logger().warn(
                "drop %s frame: unsupported encoding=%s" % (side, msg.encoding),
                throttle_duration_sec=2.0,
            )
            return

        frame = self.read_shared_image(msg)
        if frame is None:
            self.read_errors += 1
            return

        bucket = self.pending.setdefault(frame.seq, {})
        bucket[side] = frame
        if "left" in bucket and "right" in bucket:
            left = bucket["left"]
            right = bucket["right"]
            del self.pending[frame.seq]
            self.publish_pair(left, right)
        self.prune_pending()

    def side_from_camera_name(self, camera_name: str) -> Optional[str]:
        lowered = camera_name.lower()
        if lowered == "left":
            return "left"
        if lowered == "right":
            return "right"
        return None

    def read_shared_image(self, msg: ShmFrameInfo) -> Optional[SharedImage]:
        shm_path = self.shm_path(msg.shm_name)
        try:
            fd = os.open(shm_path, os.O_RDONLY)
        except OSError as exc:
            self.get_logger().warn(
                "failed to open shared memory %s: %s" % (shm_path, exc),
                throttle_duration_sec=2.0,
            )
            return None

        try:
            size = os.fstat(fd).st_size
            if size < HEADER.size:
                self.get_logger().warn(
                    "drop shm frame %s: size=%d smaller than header" % (msg.shm_name, size),
                    throttle_duration_sec=2.0,
                )
                return None
            with mmap.mmap(fd, size, access=mmap.ACCESS_READ) as mapped:
                first = self.unpack_header(mapped[:HEADER.size], msg.shm_name)
                if first is None:
                    return None
                magic, height, width, channels, payload_size, seq = first
                if magic != SHM_MAGIC or channels != 3:
                    self.get_logger().warn(
                        "drop shm frame %s: magic=0x%08x channels=%d"
                        % (msg.shm_name, magic, channels),
                        throttle_duration_sec=2.0,
                    )
                    return None
                required = HEADER.size + payload_size
                if required > size or width == 0 or height == 0:
                    self.get_logger().warn(
                        "drop shm frame %s: invalid shape %dx%d payload=%d size=%d"
                        % (msg.shm_name, width, height, payload_size, size),
                        throttle_duration_sec=2.0,
                    )
                    return None
                data = bytes(mapped[HEADER.size:required])
                second = self.unpack_header(mapped[:HEADER.size], msg.shm_name)
                if second is None or second[-1] != seq:
                    self.dropped_frames += 1
                    return None
        except OSError as exc:
            self.get_logger().warn(
                "failed to map shared memory %s: %s" % (shm_path, exc),
                throttle_duration_sec=2.0,
            )
            return None
        finally:
            os.close(fd)

        if msg.frame_seq != 0 and (seq & 0xFFFFFFFF) != msg.frame_seq:
            self.dropped_frames += 1
            return None
        return SharedImage(
            seq=int(seq),
            width=int(width),
            height=int(height),
            encoding=msg.encoding,
            data=data,
        )

    def unpack_header(self, data: bytes, name: str):
        try:
            return HEADER.unpack(data)
        except struct.error:
            self.get_logger().warn(
                "drop shm frame %s: cannot unpack header" % name,
                throttle_duration_sec=2.0,
            )
            return None

    def publish_pair(self, left: SharedImage, right: SharedImage) -> None:
        stamp = self.get_clock().now().to_msg()
        self.left_pub.publish(self.to_image_msg(left, stamp, self.left_frame_id))
        self.right_pub.publish(self.to_image_msg(right, stamp, self.right_frame_id))
        self.published_pairs += 1

    def to_image_msg(self, frame: SharedImage, stamp, frame_id: str) -> Image:
        width = frame.width
        height = frame.height
        data = frame.data
        if self.resize_width > 0 and self.resize_height > 0 and (
            width != self.resize_width or height != self.resize_height
        ):
            array = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
            resized = cv2.resize(
                array,
                (self.resize_width, self.resize_height),
                interpolation=cv2.INTER_AREA,
            )
            data = resized.tobytes()
            width = self.resize_width
            height = self.resize_height

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = height
        msg.width = width
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = width * 3
        msg.data = data
        return msg

    def prune_pending(self) -> None:
        while len(self.pending) > self.max_pending:
            oldest = min(self.pending)
            del self.pending[oldest]
            self.dropped_frames += 1

    def shm_path(self, name: str) -> str:
        return "/dev/shm/" + name.lstrip("/")

    def report(self) -> None:
        elapsed = max(time.monotonic() - self.start_time, 1e-6)
        self.get_logger().info(
            "published_pairs=%d avg_rate=%.3fHz pending=%d dropped=%d read_errors=%d"
            % (
                self.published_pairs,
                self.published_pairs / elapsed,
                len(self.pending),
                self.dropped_frames,
                self.read_errors,
            )
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--info-topic", default="/hanmole/vision/shm_frame_info")
    parser.add_argument("--left-out", default="/vo/left/image_raw")
    parser.add_argument("--right-out", default="/vo/right/image_raw")
    parser.add_argument("--resize-width", type=int, default=1280)
    parser.add_argument("--resize-height", type=int, default=720)
    parser.add_argument("--left-frame-id", default="left_eye_Link")
    parser.add_argument("--right-frame-id", default="right_eye_Link")
    parser.add_argument("--max-pending", type=int, default=32)
    parser.add_argument("--qos-depth", type=int, default=20)
    return parser.parse_args(remove_ros_args()[1:])


def main() -> None:
    args = parse_args()
    if args.resize_width < 0 or args.resize_height < 0:
        raise SystemExit("--resize-width and --resize-height must be >= 0")
    if args.max_pending < 2:
        raise SystemExit("--max-pending must be >= 2")
    rclpy.init()
    node = VisionShmStereoBridge(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
