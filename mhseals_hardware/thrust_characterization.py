"""Fit a boat velocity model from a ROS bag or simple JSON/CSV samples.

The fitter is deliberately ROS-independent.  A sample is a mapping with
``command`` (surge, sway, yaw), ``velocity`` (measured vx, vy, wz), and an
optional ``time`` in seconds.  The CLI can read a rosbag2 directory when ROS
is installed, or newline-delimited JSON for offline CI and field analysis.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from bisect import bisect_right
from pathlib import Path
from typing import Iterable, Mapping, Sequence

from mhseals_hardware.configuration import load_config, save_config
from mhseals_hardware.thruster_mixer import THRUSTER_MIXER, validate_mixer

AXES = ("surge", "sway", "yaw")


def nav2_profile(result, source):
    """Return a complete Nav2 profile with measured component limits."""
    import yaml
    profile = yaml.safe_load(Path(source).read_text())
    limits = result["motion_limits"]
    velocity = limits["velocity"]
    acceleration = limits["acceleration"]
    deceleration = limits["deceleration"]
    controller = profile["controller_server"]["ros__parameters"]
    controller["FollowPath"]["desired_linear_vel"] = velocity["surge"]
    smoother = profile["velocity_smoother"]["ros__parameters"]
    smoother["max_velocity"] = [velocity[axis] for axis in AXES]
    smoother["min_velocity"] = [-velocity[axis] for axis in AXES]
    smoother["max_accel"] = [acceleration[axis] for axis in AXES]
    smoother["max_decel"] = [-deceleration[axis] for axis in AXES]
    return profile


def _vector(value, name):
    if isinstance(value, Mapping):
        value = [value.get(axis, 0.0) for axis in AXES]
    if len(value) != 3:
        raise ValueError(f"{name} must contain surge, sway, yaw")
    values = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in values):
        raise ValueError(f"{name} must contain finite values")
    return values


def _inverse(matrix, regularization=1e-6):
    """Invert a 3x3 matrix, with a small ridge term for weak experiments."""
    rows = [list(map(float, row)) for row in matrix]
    for i in range(3):
        rows[i][i] += regularization
    aug = [rows[i] + [1.0 if i == j else 0.0 for j in range(3)]
           for i in range(3)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(aug[row][col]))
        if abs(aug[pivot][col]) < 1e-9:
            raise ValueError("commands do not excite all three motion axes")
        aug[col], aug[pivot] = aug[pivot], aug[col]
        scale = aug[col][col]
        aug[col] = [item / scale for item in aug[col]]
        for row in range(3):
            if row == col:
                continue
            scale = aug[row][col]
            aug[row] = [a - scale * b for a, b in zip(aug[row], aug[col])]
    return tuple(tuple(aug[row][3 + col] for col in range(3)) for row in range(3))


def _matmul(left, right):
    return tuple(tuple(sum(left[i][k] * right[k][j] for k in range(3))
                         for j in range(3)) for i in range(len(left)))


def _normal_equations(samples):
    # Fit v = bias + gain * command using ordinary least squares.  The tiny
    # 4x4 solve avoids requiring NumPy on the robot image.
    gram = [[0.0] * 4 for _ in range(4)]
    rhs = [[0.0] * 4 for _ in range(3)]
    for command, velocity, _ in samples:
        row = (1.0, *command)
        for i in range(4):
            for j in range(4):
                gram[i][j] += row[i] * row[j]
            for axis in range(3):
                rhs[axis][i] += velocity[axis] * row[i]
    for i in range(4):
        gram[i][i] += 1e-8
    # Do not turn a one-axis bag into a seemingly valid 3-axis calibration.
    rank_rows = [row[:] for row in gram]
    rank = 0
    for col in range(4):
        pivot = max(range(rank, 4), key=lambda row: abs(rank_rows[row][col]))
        if abs(rank_rows[pivot][col]) < 1e-6:
            continue
        rank_rows[rank], rank_rows[pivot] = rank_rows[pivot], rank_rows[rank]
        scale = rank_rows[rank][col]
        rank_rows[rank] = [value / scale for value in rank_rows[rank]]
        for row in range(4):
            if row != rank:
                factor = rank_rows[row][col]
                rank_rows[row] = [a - factor * b for a, b in zip(rank_rows[row], rank_rows[rank])]
        rank += 1
    if rank < 4:
        raise ValueError('samples must excite neutral, surge, sway, and yaw')
    coefficients = []
    for axis in range(3):
        aug = [gram[row][:] + [rhs[axis][row]] for row in range(4)]
        for col in range(4):
            pivot = max(range(col, 4), key=lambda row: abs(aug[row][col]))
            if abs(aug[pivot][col]) < 1e-10:
                raise ValueError("not enough varied command samples")
            aug[col], aug[pivot] = aug[pivot], aug[col]
            scale = aug[col][col]
            aug[col] = [item / scale for item in aug[col]]
            for row in range(4):
                if row != col:
                    scale = aug[row][col]
                    aug[row] = [a - scale * b for a, b in zip(aug[row], aug[col])]
        coefficients.append(tuple(aug[row][4] for row in range(4)))
    return tuple(row[0] for row in coefficients), tuple(
        tuple(row[index + 1] for index in range(3)) for row in coefficients)


def characterize_samples(samples: Iterable[Mapping], mixer=THRUSTER_MIXER, source="samples"):
    """Return calibration data suitable for writing to hardware YAML.

    Samples are paired command and odometry observations.  Commands are the
    normalized inputs consumed by ``thruster_pwm_node``; velocities are SI
    units from odometry. The fitted gain drives the optional calibrated
    velocity controller; the physical effort mixer remains unchanged unless
    ``--apply-matrix`` is explicitly requested.
    """
    mixer = tuple(value for row in mixer for value in row) if len(mixer) == 4 and hasattr(mixer[0], "__iter__") else mixer
    normalized = []
    for sample in samples:
        command = _vector(sample.get("command", sample.get("cmd_vel")), "command")
        velocity = _vector(sample.get("velocity", sample.get("odom")), "velocity")
        stamp = float(sample.get("time", sample.get("stamp", 0.0)))
        if not math.isfinite(stamp):
            stamp = 0.0
        normalized.append((command, velocity, stamp))
    if len(normalized) < 8:
        raise ValueError("at least eight command/odometry samples are required")
    bias, gain = _normal_equations(normalized)
    try:
        inverse_gain = _inverse(gain)
        corrected = _matmul(validate_mixer(mixer), inverse_gain)
        corrected = tuple(tuple(max(-1.0, min(1.0, value)) for value in row)
                          for row in corrected)
    except ValueError:
        corrected = validate_mixer(mixer)
    limits = {}
    for axis, index in zip(AXES, range(3)):
        values = [abs(row[0][index]) for row in normalized if abs(row[0][index]) > .02]
        limits[axis] = max((abs(row[1][index]) for row in normalized), default=0.0)
        limits[axis] = {"velocity": limits[axis], "command": max(values, default=1.0)}
    acceleration = {axis: 0.0 for axis in AXES}
    deceleration = {axis: 0.0 for axis in AXES}
    for previous, current in zip(normalized, normalized[1:]):
        dt = current[2] - previous[2]
        if dt <= 1e-3:
            continue
        for index, axis in enumerate(AXES):
            change = (current[1][index] - previous[1][index]) / dt
            acceleration[axis] = max(acceleration[axis], change)
            deceleration[axis] = max(deceleration[axis], -change)
    combined = {}
    for command, velocity, _ in normalized:
        active = tuple(axis for axis, value in zip(AXES, command) if abs(value) > .02)
        if len(active) < 2:
            continue
        direction = ','.join(f'{axis}={1 if command[AXES.index(axis)] > 0 else -1}'
                             for axis in active)
        speed = math.sqrt(sum(velocity[index] ** 2 for index in range(3)))
        combined[direction] = max(combined.get(direction, 0.0), speed)
    return {
        "version": 1,
        "source": source,
        "identified_at": datetime.now(timezone.utc).isoformat(),
        "sample_count": len(normalized),
        "bias": {axis: bias[i] for i, axis in enumerate(AXES)},
        "velocity_gain": {axis: {name: gain[i][j] for j, name in enumerate(AXES)}
                           for i, axis in enumerate(AXES)},
        # This is an optional feed-forward allocation expressed in the
        # original command units.  It is not a replacement for the physical
        # effort mixer unless the operator explicitly applies it.
        "recommended_velocity_allocation": [value for row in corrected for value in row],
        "thruster_matrix": [value for row in validate_mixer(mixer) for value in row],
        "motion_limits": {
            "velocity": {axis: limits[axis]["velocity"] for axis in AXES},
            "acceleration": acceleration,
            "deceleration": deceleration,
            "combined": {"surge": limits["surge"]["velocity"],
                         "sway": limits["sway"]["velocity"],
                         "yaw": limits["yaw"]["velocity"]},
            "combined_directions": combined,
        },
    }


def read_json_samples(path: Path):
    text = path.read_text()
    try:
        value = json.loads(text)
    except json.JSONDecodeError:
        value = [json.loads(line) for line in text.splitlines() if line.strip()]
    if isinstance(value, Mapping):
        value = value.get("samples", [])
    return value


def read_csv_samples(path: Path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = []
        for row in csv.DictReader(stream):
            rows.append({"time": row.get("time", row.get("stamp", 0)),
                         "command": [row.get("cmd_surge", row.get("surge", 0)),
                                     row.get("cmd_sway", row.get("sway", 0)),
                                     row.get("cmd_yaw", row.get("yaw", 0))],
                         "velocity": [row.get("vx", 0), row.get("vy", 0), row.get("wz", 0)]})
    return rows


def read_rosbag(path: Path, command_topic="/cmd_vel", odom_topic="/odom/local"):
    """Read nearest command/odometry pairs from a rosbag2 directory."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
    except ImportError as error:
        raise RuntimeError("ROS 2 rosbag dependencies are required for --bag") from error
    reader = rosbag2_py.SequentialReader()
    storage_id = "sqlite3"
    metadata = path / "metadata.yaml"
    if metadata.exists() and "storage_identifier: mcap" in metadata.read_text():
        storage_id = "mcap"
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    topics = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if command_topic not in topics or odom_topic not in topics:
        raise ValueError(f"bag must contain {command_topic} and {odom_topic}")
    messages = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic in (command_topic, odom_topic):
            cls = Twist if topic == command_topic else Odometry
            messages.append((stamp / 1e9, topic, deserialize_message(data, cls)))
    commands = []
    samples = []
    for timestamp, topic, msg in messages:
        if topic == command_topic:
            commands.append((timestamp, (msg.linear.x, msg.linear.y, msg.angular.z)))
        else:
            if not commands:
                continue
            # Hold the latest command already received.  Pairing with a future
            # command creates artificial response and stopping samples at
            # phase boundaries.
            index = bisect_right([item[0] for item in commands], timestamp) - 1
            if index < 0:
                continue
            command = commands[index][1]
            twist = msg.twist.twist
            samples.append({"time": timestamp, "command": command,
                            "velocity": (twist.linear.x, twist.linear.y, twist.angular.z)})
    return samples


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--bag", type=Path, help="rosbag2 directory")
    source.add_argument("--samples", type=Path, help="JSON or CSV samples")
    parser.add_argument("--config", type=Path, help="hardware YAML to update")
    parser.add_argument("--output", type=Path, help="write calibration YAML")
    parser.add_argument("--apply-matrix", action="store_true",
                        help="replace the physical effort mixer after review")
    parser.add_argument("--nav2-params", type=Path,
                        help="source Nav2 YAML when generating --nav2-overlay")
    parser.add_argument("--nav2-overlay", type=Path,
                        help="write a complete Nav2 YAML with measured limits")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--odom-topic", default="/odom/local")
    args = parser.parse_args(argv)
    if args.bag:
        samples = read_rosbag(args.bag, args.cmd_topic, args.odom_topic)
    elif args.samples.suffix.lower() == ".csv":
        samples = read_csv_samples(args.samples)
    else:
        samples = read_json_samples(args.samples)
    config = load_config(args.config) if args.config else load_config()
    result = characterize_samples(samples, config["thruster_matrix"],
                                 source="rosbag2" if args.bag else "samples")
    if args.apply_matrix:
        config["thruster_matrix"] = result["recommended_velocity_allocation"]
    config["characterization"] = result
    target = args.output or args.config
    if target:
        save_config(config, target)
    if args.nav2_overlay:
        if not args.nav2_params:
            parser.error('--nav2-overlay requires --nav2-params')
        import yaml
        args.nav2_overlay.parent.mkdir(parents=True, exist_ok=True)
        args.nav2_overlay.write_text(yaml.safe_dump(
            nav2_profile(result, args.nav2_params), sort_keys=False))
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
