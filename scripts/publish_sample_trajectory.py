#!/usr/bin/env python3
"""Publish a CSV joint trajectory and verify its visualization outputs.

The CSV format matches trajectory_replay_node / ReplayManager:

    joint_names,<joint_1>,...,<joint_N>
    t,positions...,velocities...,accelerations...

The script publishes:
  /excavator/planned_trajectory  (transient local, once)
  /joint_states                  (active joints + passive cylinder joints)
  /excavator/execution_status    (IDLE/RUNNING/COMPLETED with elapsed time)

With --check it also waits for the trajectory visualizer outputs and verifies:
  /excavator/planned_path        (MarkerArray with DELETEALL, path and endpoints)
  /excavator/end_effector_path   (one pose per trajectory point)
  /excavator/trajectory_occupancy (mesh markers for multiple links)

Typical usage
-------------
Terminal 1 - visualization (RViz, robot_state_publisher, visualizer):
    ros2 launch excavator_visualization visualization.launch.py \
        start_static_joint_states:=false

Terminal 2 - publish and inspect:
    python3 scripts/publish_sample_trajectory.py --loop

or run the automated visualization check:
    python3 scripts/publish_sample_trajectory.py --check

Do not run the replay node or a simulator at the same time: this script is the
authoritative /joint_states source.

The launch files default to an isolated ROS domain (77 when ROS_DOMAIN_ID is
not set) and publish the robot model on /excavator/robot_description, so
unrelated ROS 2 systems on the network do not disturb the visualization. This
script uses the same default domain; set ROS_DOMAIN_ID in the shell to override
it (the launch files then pick up the same value).
"""

import argparse
import bisect
import math
import os
import sys
import time

# Match the default domain used by the project launch files, so that a plain
# `python3 scripts/publish_sample_trajectory.py` still talks to them.
os.environ.setdefault("ROS_DOMAIN_ID", "77")

import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from excavator_interfaces.msg import ExecutionStatus
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

# Home positions for the passive cylinder/linkage joints. They are not part of
# the CSV but are required for the URDF model and TF.
PASSIVE_JOINTS = [
    ("boom_cyl_l_body_joint", -0.30301),
    ("boom_cyl_l_rod_joint", -0.23228),
    ("boom_cyl_r_body_joint", -0.30301),
    ("boom_cyl_r_rod_joint", -0.23228),
    ("stick_cyl_body_joint", 0.04501),
    ("stick_cyl_rod_joint", -0.36905),
    ("bucket_cyl_body_joint", -0.08259),
    ("bucket_cyl_rod_joint", 0.11620),
    ("bucket_linkage_joint", -0.40),
    ("stick_rocker_l_joint", -0.40),
    ("stick_rocker_r_joint", -0.40),
]


def latch_qos():
    return QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def as_duration(seconds):
    """Convert seconds to a builtin_interfaces Duration with carry handling."""
    whole = int(math.floor(seconds))
    nanosec = int(round((seconds - whole) * 1e9))
    if nanosec >= 1000000000:
        whole += 1
        nanosec -= 1000000000
    return Duration(sec=whole, nanosec=nanosec)


def load_csv(path):
    """Read the joint trajectory CSV.

    Returns (joint_names, points), where every point is a tuple:
    (t, positions, velocities, accelerations).
    """
    if not os.path.isfile(path):
        raise ValueError(f"trajectory file not found: {path}")
    with open(path, "r", encoding="utf-8") as stream:
        lines = [line.strip() for line in stream if line.strip()]
    if not lines:
        raise ValueError(f"trajectory file is empty: {path}")

    header = [field.strip() for field in lines[0].split(",")]
    if len(header) < 2 or header[0] != "joint_names":
        raise ValueError(
            "CSV must start with a 'joint_names,...' header row")
    joint_names = header[1:]
    if any(not name for name in joint_names):
        raise ValueError("CSV contains an empty joint name")
    joint_count = len(joint_names)
    expected_columns = 1 + 3 * joint_count

    points = []
    previous_t = None
    for line_number, line in enumerate(lines[1:], start=2):
        fields = [field.strip() for field in line.split(",")]
        if len(fields) != expected_columns:
            raise ValueError(
                f"line {line_number}: expected {expected_columns} columns, "
                f"got {len(fields)}")
        try:
            values = [float(field) for field in fields]
        except ValueError as error:
            raise ValueError(f"line {line_number}: {error}") from error
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"line {line_number}: non-finite value")
        t = values[0]
        if t < 0.0:
            raise ValueError(f"line {line_number}: time must be non-negative")
        if previous_t is not None and t <= previous_t:
            raise ValueError(
                f"line {line_number}: time must be strictly increasing")
        previous_t = t
        positions = values[1:1 + joint_count]
        velocities = values[1 + joint_count:1 + 2 * joint_count]
        accelerations = values[1 + 2 * joint_count:1 + 3 * joint_count]
        points.append((t, positions, velocities, accelerations))

    if not points:
        raise ValueError(f"trajectory file contains no data rows: {path}")
    return joint_names, points


def to_ros_trajectory(joint_names, points, stamp):
    message = JointTrajectory()
    message.header.stamp = stamp
    message.header.frame_id = "base_link"
    message.joint_names = joint_names
    for t, positions, velocities, accelerations in points:
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.velocities = list(velocities)
        point.accelerations = list(accelerations)
        point.time_from_start = as_duration(t)
        message.points.append(point)
    return message


class SampleTrajectoryPublisher(Node):
    def __init__(self, args, joint_names, points):
        super().__init__("sample_trajectory_publisher")
        self.args = args
        self.joint_names = joint_names
        self.points = points
        self.times = [point[0] for point in points]
        self.duration = self.times[-1]
        self.passive_names = [name for name, _ in PASSIVE_JOINTS]
        self.passive_positions = [value for _, value in PASSIVE_JOINTS]
        if self.duration <= 0.0:
            raise ValueError("trajectory duration must be positive")

        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, "/excavator/planned_trajectory", latch_qos())
        self.joint_state_publisher = self.create_publisher(
            JointState, "/joint_states", 10)
        self.status_publisher = self.create_publisher(
            ExecutionStatus, "/excavator/execution_status", 10)

        self.start_at = time.monotonic() + args.start_delay
        self.motion_started_at = None
        self.last_tick_at = None
        self.elapsed = 0.0
        self.finished = False
        self.request_exit = False
        self.exit_code = 0

        self.check_received = {
            "planned_path": None,
            "end_effector_path": None,
            "occupancy": None,
        }
        if args.check:
            self.create_subscription(
                MarkerArray, "/excavator/planned_path",
                self.on_planned_path, latch_qos())
            self.create_subscription(
                Path, "/excavator/end_effector_path",
                self.on_end_effector_path, latch_qos())
            self.create_subscription(
                MarkerArray, "/excavator/trajectory_occupancy",
                self.on_occupancy, latch_qos())
            self.check_deadline = time.monotonic() + args.check_timeout
            self.check_done = False
            self.check_timer = self.create_timer(0.1, self.run_check)

        message = to_ros_trajectory(
            joint_names, points, self.get_clock().now().to_msg())
        self.trajectory_publisher.publish(message)
        self.get_logger().info(
            f"loaded {len(points)} points, {len(joint_names)} joints, "
            f"duration {self.duration:.3f} s")
        self.get_logger().info(
            "publishing /excavator/planned_trajectory (transient local), "
            "/joint_states and /excavator/execution_status at "
            f"{args.rate:.1f} Hz")
        if args.start_delay > 0.0:
            self.get_logger().info(
                f"waiting {args.start_delay:.1f} s before motion")
        self.timer = self.create_timer(1.0 / args.rate, self.tick)
        self.conflict_timer = self.create_timer(1.0, self.check_conflicts)
        self.conflicts_checked = False

    def check_conflicts(self):
        """Warn about another ROS 2 stack publishing the same global topics."""
        if self.conflicts_checked:
            return
        self.conflicts_checked = True
        self.conflict_timer.cancel()
        joint_publishers = self.get_publishers_info_by_topic("/joint_states")
        description_publishers = self.get_publishers_info_by_topic(
            "/excavator/robot_description")
        others = [
            f"{info.node_namespace.rstrip('/')}/{info.node_name}"
            for info in joint_publishers
            if info.node_name != self.get_name()
        ]
        if others:
            self.get_logger().warn(
                "/joint_states has additional publishers: "
                + ", ".join(others)
                + "; the curve data may be mixed with another robot")
        if len(description_publishers) != 1:
            self.get_logger().warn(
                f"/excavator/robot_description has "
                f"{len(description_publishers)} publishers (expected 1); "
                "start visualization.launch.py or check ROS_DOMAIN_ID.")

    # --- publishers -----------------------------------------------------
    def publish_state(self, sample, state, message):
        t, positions, velocities, _ = sample
        now = self.get_clock().now().to_msg()
        status = ExecutionStatus()
        status.header.stamp = now
        status.state = state
        status.message = message
        status.elapsed_time = max(0.0, min(t, self.duration))
        status.duration = self.duration
        status.progress = status.elapsed_time / self.duration
        status.tracking_error_norm = 0.0
        self.status_publisher.publish(status)

        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.name = list(self.joint_names) + self.passive_names
        joint_state.position = (
            list(positions) + list(self.passive_positions))
        joint_state.velocity = (
            list(velocities) + [0.0] * len(self.passive_names))
        # self.joint_state_publisher.publish(joint_state)

    def sample_at(self, t):
        if t <= self.times[0]:
            return self.points[0]
        if t >= self.times[-1]:
            return self.points[-1]
        index = bisect.bisect_right(self.times, t)
        first = self.points[index - 1]
        second = self.points[index]
        span = second[0] - first[0]
        ratio = 0.0 if span <= 0.0 else (t - first[0]) / span
        positions = [
            a + ratio * (b - a)
            for a, b in zip(first[1], second[1])
        ]
        velocities = [
            a + ratio * (b - a)
            for a, b in zip(first[2], second[2])
        ]
        accelerations = [
            a + ratio * (b - a)
            for a, b in zip(first[3], second[3])
        ]
        return (t, positions, velocities, accelerations)

    def tick(self):
        now = time.monotonic()
        if now < self.start_at:
            # Keep the model visible but do not collect measured samples yet.
            self.publish_state(
                self.points[0], ExecutionStatus.IDLE,
                "sample trajectory ready; waiting to start")
            return

        if self.motion_started_at is None:
            self.motion_started_at = now
            self.last_tick_at = now
        elapsed_dt = now - self.last_tick_at
        self.last_tick_at = now
        if elapsed_dt <= 0.0:
            return
        self.elapsed += elapsed_dt * self.args.replay_rate

        if self.elapsed > self.duration:
            if self.args.loop:
                self.elapsed = self.elapsed % self.duration
            else:
                self.elapsed = self.duration
                final = self.points[-1]
                self.publish_state(
                    final, ExecutionStatus.COMPLETED,
                    "sample trajectory replay finished")
                self.timer.cancel()
                if not self.finished:
                    self.finished = True
                    self.get_logger().info(
                        "replay finished; press Ctrl+C to stop the node")
                return

        sample = self.sample_at(self.elapsed)
        self.publish_state(
            sample, ExecutionStatus.RUNNING, "sample trajectory replay")

    # --- visualization checks ------------------------------------------
    def on_planned_path(self, message):
        self.check_received["planned_path"] = message

    def on_end_effector_path(self, message):
        self.check_received["end_effector_path"] = message

    def on_occupancy(self, message):
        self.check_received["occupancy"] = message

    def run_check(self):
        if not self.args.check or self.check_done:
            return
        received_all = all(
            message is not None for message in self.check_received.values())
        timed_out = time.monotonic() > self.check_deadline
        if not received_all and not timed_out:
            return

        self.check_done = True
        failures = []
        planned = self.check_received["planned_path"]
        if planned is None:
            failures.append("/excavator/planned_path not received")
        else:
            markers = planned.markers
            has_clear = any(
                marker.action == Marker.DELETEALL for marker in markers)
            if len(markers) < 5 or not has_clear:
                failures.append(
                    "/excavator/planned_path markers are incomplete")
            else:
                self.get_logger().info(
                    f"planned_path: {len(markers)} markers, DELETEALL present")

        end_effector = self.check_received["end_effector_path"]
        if end_effector is None:
            failures.append("/excavator/end_effector_path not received")
        elif len(end_effector.poses) != len(self.points):
            failures.append(
                "end-effector pose count does not match trajectory points")
        else:
            self.get_logger().info(
                f"end_effector_path: {len(end_effector.poses)} poses")

        occupancy = self.check_received["occupancy"]
        if occupancy is None:
            failures.append("/excavator/trajectory_occupancy not received")
        else:
            meshes = [marker for marker in occupancy.markers
                      if marker.type == Marker.MESH_RESOURCE]
            namespaces = sorted({marker.ns for marker in meshes})
            if len(meshes) < 8 or len(namespaces) < 2:
                failures.append(
                    "occupancy mesh markers are incomplete")
            else:
                self.get_logger().info(
                    f"occupancy: {len(meshes)} mesh markers, "
                    f"{len(namespaces)} link namespaces")

        if failures:
            print("VISUALIZATION CHECK: FAIL")
            for failure in failures:
                print(f"  - {failure}")
            print("  hint: start visualization.launch.py or "
                  "visualizer.launch.py before this check")
            self.exit_code = 1
            self.request_exit = True
            return

        print("VISUALIZATION CHECK: PASS")
        if not self.args.loop:
            self.request_exit = True


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Publish a CSV joint trajectory and verify visualization")
    try:
        default_file = os.path.join(
            get_package_share_directory("excavator_visualization"),
            "config", "sample_trajectory.csv")
    except Exception:  # noqa: BLE001 - package may not be sourced
        default_file = os.path.join(
            os.path.dirname(__file__), "..", "src",
            "excavator_visualization", "config", "sample_trajectory.csv")
    parser.add_argument(
        "--trajectory-file", default=default_file,
        help="CSV file with the trajectory to publish")
    parser.add_argument(
        "--rate", type=float, default=50.0,
        help="joint-state/status publication rate in Hz")
    parser.add_argument(
        "--replay-rate", type=float, default=1.0,
        help="trajectory speed multiplier (1.0 = real time)")
    parser.add_argument(
        "--start-delay", type=float, default=2.0,
        help="seconds to wait before starting the motion")
    parser.add_argument(
        "--loop", action="store_true",
        help="restart the trajectory after reaching its end")
    parser.add_argument(
        "--check", action="store_true",
        help="wait for and verify the trajectory visualizer outputs")
    parser.add_argument(
        "--check-timeout", type=float, default=10.0,
        help="seconds to wait for visualization outputs")
    return parser.parse_args()


def main():
    args = parse_arguments()
    if args.rate <= 0.0 or args.replay_rate <= 0.0:
        print("FAIL: --rate and --replay-rate must be positive")
        return 2
    if args.check_timeout <= 0.0:
        print("FAIL: --check-timeout must be positive")
        return 2
    try:
        joint_names, points = load_csv(
            os.path.abspath(args.trajectory_file))
    except ValueError as error:
        print(f"FAIL: {error}")
        return 2

    rclpy.init()
    node = None
    try:
        node = SampleTrajectoryPublisher(args, joint_names, points)
        while rclpy.ok() and not node.request_exit:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    return node.exit_code if node is not None else 1


if __name__ == "__main__":
    sys.exit(main())
