#!/usr/bin/env python3
import argparse
import csv
import shutil
import struct
import subprocess
import sys
from pathlib import Path
from typing import Dict, Tuple

import cv2
import numpy as np


HEADER = struct.Struct("<IHHQHHIIII")
MAGIC = 0x314D4348
PIXEL_MJPEG = 1
PIXEL_RGB8 = 2
PIXEL_BGR8 = 3
PIXEL_GRAY8 = 4


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export strict SDK stereo calibration payloads to a Kalibr image folder."
    )
    parser.add_argument("dataset", help="sdk_stereo_calib_* directory")
    parser.add_argument(
        "--bag",
        default="",
        help="optional ROS1 bag output path; requires kalibr_bagcreater in the current shell",
    )
    return parser.parse_args()


def read_payload(path: Path) -> Tuple[Dict[str, int], bytes]:
    data = path.read_bytes()
    if len(data) < HEADER.size:
        raise RuntimeError(f"payload too small: {path}")
    values = HEADER.unpack(data[: HEADER.size])
    header = {
        "magic": values[0],
        "version": values[1],
        "flags": values[2],
        "timestamp_ns": values[3],
        "width": values[4],
        "height": values[5],
        "stride_bytes": values[6],
        "pixel_format": values[7],
        "fps": values[8],
        "image_size": values[9],
    }
    if header["magic"] != MAGIC or header["version"] != 1:
        raise RuntimeError(f"unexpected camera payload header: {path}")
    start = HEADER.size
    end = start + header["image_size"]
    if end > len(data):
        raise RuntimeError(f"camera payload image overflow: {path}")
    return header, data[start:end]


def decode_gray(header: Dict[str, int], image: bytes, path: Path) -> np.ndarray:
    pixel_format = header["pixel_format"]
    width = header["width"]
    height = header["height"]
    if width <= 0 or height <= 0:
        raise RuntimeError(f"invalid image size in {path}: {width}x{height}")

    if pixel_format == PIXEL_MJPEG:
        encoded = np.frombuffer(image, dtype=np.uint8)
        decoded = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)
        if decoded is None or decoded.size == 0:
            raise RuntimeError(f"failed to decode MJPEG payload: {path}")
        return decoded

    if pixel_format in (PIXEL_RGB8, PIXEL_BGR8):
        step = header["stride_bytes"] or width * 3
        required = step * height
        if step < width * 3 or required > len(image):
            raise RuntimeError(f"invalid RGB/BGR stride in {path}")
        array = np.frombuffer(image[:required], dtype=np.uint8).reshape((height, step))
        bgr_or_rgb = array[:, : width * 3].reshape((height, width, 3))
        code = cv2.COLOR_RGB2GRAY if pixel_format == PIXEL_RGB8 else cv2.COLOR_BGR2GRAY
        return cv2.cvtColor(bgr_or_rgb, code)

    if pixel_format == PIXEL_GRAY8:
        step = header["stride_bytes"] or width
        required = step * height
        if step < width or required > len(image):
            raise RuntimeError(f"invalid gray stride in {path}")
        array = np.frombuffer(image[:required], dtype=np.uint8).reshape((height, step))
        return array[:, :width].copy()

    raise RuntimeError(f"unsupported pixel_format={pixel_format} in {path}")


def export_images(dataset: Path) -> Path:
    pairs_csv = dataset / "pairs.csv"
    if not pairs_csv.exists():
        raise RuntimeError(f"missing pairs.csv: {pairs_csv}")

    kalibr_dir = dataset / "kalibr"
    cam0_dir = kalibr_dir / "cam0"
    cam1_dir = kalibr_dir / "cam1"
    cam0_dir.mkdir(parents=True, exist_ok=True)
    cam1_dir.mkdir(parents=True, exist_ok=True)

    exported = 0
    with pairs_csv.open(newline="") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            stamp_ns = row["stamp_ns"]
            left_payload = dataset / row["left_payload"]
            right_payload = dataset / row["right_payload"]
            left_header, left_image = read_payload(left_payload)
            right_header, right_image = read_payload(right_payload)
            left_gray = decode_gray(left_header, left_image, left_payload)
            right_gray = decode_gray(right_header, right_image, right_payload)

            left_out = cam0_dir / f"{stamp_ns}.png"
            right_out = cam1_dir / f"{stamp_ns}.png"
            if not cv2.imwrite(str(left_out), left_gray):
                raise RuntimeError(f"failed to write {left_out}")
            if not cv2.imwrite(str(right_out), right_gray):
                raise RuntimeError(f"failed to write {right_out}")
            exported += 1

    command = (
        "kalibr_bagcreater "
        f"--folder {kalibr_dir} "
        "--image-topics /cam0/image_raw /cam1/image_raw "
        f"--output-bag {dataset / 'kalibr_stereo.bag'}"
    )
    bag_script = kalibr_dir / "create_ros1_bag.sh"
    bag_script.write_text("#!/usr/bin/env bash\nset -e\n" + command + "\n")
    bag_script.chmod(0o755)
    (kalibr_dir / "README.txt").write_text(
        "Kalibr image folder exported from strict SDK stereo payloads.\n"
        "cam0 is left/Camera1; cam1 is right/Camera2.\n"
        "Image filenames are exact SDK capture timestamps in nanoseconds.\n"
        "To create a ROS1 bag inside a Kalibr/ROS1 shell, run:\n"
        f"{command}\n"
    )
    print(f"Exported {exported} stereo pairs to {kalibr_dir}")
    return kalibr_dir


def create_bag(kalibr_dir: Path, bag_path: Path) -> None:
    executable = shutil.which("kalibr_bagcreater")
    if executable is None:
        raise RuntimeError("kalibr_bagcreater not found; run this in a Kalibr/ROS1 shell")
    subprocess.run(
        [
            executable,
            "--folder",
            str(kalibr_dir),
            "--image-topics",
            "/cam0/image_raw",
            "/cam1/image_raw",
            "--output-bag",
            str(bag_path),
        ],
        check=True,
    )


def main() -> int:
    args = parse_args()
    dataset = Path(args.dataset).expanduser().resolve()
    try:
        kalibr_dir = export_images(dataset)
        if args.bag:
            create_bag(kalibr_dir, Path(args.bag).expanduser().resolve())
    except Exception as exc:
        print(f"sdk_stereo_calib_export_kalibr.py failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
