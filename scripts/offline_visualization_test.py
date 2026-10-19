#!/usr/bin/env python3
"""Offline visualization end-to-end test.

Starts only replay.launch.py (robot_state_publisher + trajectory replay +
trajectory visualizer). No planner, controller or simulator is launched. The
test then verifies that the saved trajectory is replayed and converted into:

  - /excavator/planned_trajectory
  - /joint_states
  - /excavator/planned_path          (MarkerArray)
  - /excavator/end_effector_path     (nav_msgs/Path)
  - /excavator/trajectory_occupancy  (mesh MarkerArray)

Run (after sourcing the workspace):

  python3 scripts/offline_visualization_test.py
  python3 scripts/offline_visualization_test.py --trajectory-file /path/to.csv
  python3 scripts/offline_visualization_test.py --start-rviz

The test is headless by default. Pass --start-rviz to also open RViz and keep
the replay alive for 20 s (override with --hold-seconds) so the curves can be
inspected.
"""

import argparse
import os
import signal
import subprocess
import sys
import tempfile
import time

# Match the default domain used by the project launch files, so that a plain
# `python3 scripts/offline_visualization_test.py` still talks to them.
os.environ.setdefault("ROS_DOMAIN_ID", "77")

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from excavator_interfaces.msg import ExecutionStatus
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray


def latch_qos(depth=1):
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class OfflineVisualizationCheck(Node):
    def __init__(self):
        super().__init__("offline_visualization_test")
        self.trajectory = None
        self.planned_markers = None
        self.end_effector_path = None
        self.occupancy = None
        self.status = None
        self.last_joint_state = None
        self.joint_state_samples = 0
        self.consistent_pairs = 0
        self.create_subscription(
            JointTrajectory, "/excavator/planned_trajectory",
            self.on_trajectory, latch_qos())
        self.create_subscription(
            MarkerArray, "/excavator/planned_path",
            self.on_planned_markers, latch_qos())
        self.create_subscription(
            Path, "/excavator/end_effector_path",
            self.on_end_effector_path, latch_qos())
        self.create_subscription(
            MarkerArray, "/excavator/trajectory_occupancy",
            self.on_occupancy, latch_qos())
        self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, 10)
        self.create_subscription(
            ExecutionStatus, "/excavator/execution_status",
            self.on_execution_status, 10)

    def on_trajectory(self, msg):
        self.trajectory = msg

    def on_planned_markers(self, msg):
        self.planned_markers = msg

    def on_end_effector_path(self, msg):
        self.end_effector_path = msg

    def on_occupancy(self, msg):
        self.occupancy = msg

    def on_execution_status(self, msg):
        self.status = msg

    def on_joint_state(self, msg):
        self.joint_state_samples += 1
        self.last_joint_state = msg
        # The replay publishes ExecutionStatus immediately before the
        # corresponding JointState in the same tick. Use the reported
        # trajectory time to check that the measured curve is not time-shifted
        # relative to the reference trajectory.
        if (self.status is not None and self.trajectory is not None and
                self.trajectory.points):
            expected = sample_trajectory(
                self.trajectory, self.status.elapsed_time)
            position_error = 0.0
            velocity_error = 0.0
            for index, name in enumerate(self.trajectory.joint_names):
                if name not in msg.name:
                    continue
                measured_index = msg.name.index(name)
                if measured_index >= len(msg.position):
                    continue
                position_error = max(
                    position_error,
                    abs(msg.position[measured_index] - expected[index][0]))
                if measured_index < len(msg.velocity):
                    velocity_error = max(
                        velocity_error,
                        abs(msg.velocity[measured_index] - expected[index][1]))
            if position_error < 0.02 and velocity_error < 0.03:
                self.consistent_pairs += 1


def sample_trajectory(trajectory, t):
    """Linearly interpolate (position, velocity) at trajectory time t."""
    points = trajectory.points
    if t <= to_seconds(points[0].time_from_start):
        index = 0
        ratio = 0.0
    elif t >= to_seconds(points[-1].time_from_start):
        index = len(points) - 1
        ratio = 0.0
    else:
        index = 1
        while to_seconds(points[index].time_from_start) < t:
            index += 1
        start = to_seconds(points[index - 1].time_from_start)
        end = to_seconds(points[index].time_from_start)
        ratio = (t - start) / (end - start) if end > start else 0.0
    first = points[max(0, index - 1)]
    second = points[index]
    values = []
    for joint in range(len(trajectory.joint_names)):
        first_position = (
            first.positions[joint] if joint < len(first.positions) else 0.0)
        second_position = (
            second.positions[joint] if joint < len(second.positions) else 0.0)
        first_velocity = (
            first.velocities[joint] if joint < len(first.velocities) else 0.0)
        second_velocity = (
            second.velocities[joint] if joint < len(second.velocities) else 0.0)
        values.append((
            first_position + ratio * (second_position - first_position),
            first_velocity + ratio * (second_velocity - first_velocity)))
    return values


def to_seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


def wait_until(node, predicate, timeout):
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def stop_process(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--trajectory-file",
        default=os.path.join(
            get_package_share_directory("excavator_visualization"),
            "config", "sample_trajectory.csv"),
        help="CSV trajectory file used for offline replay")
    parser.add_argument("--timeout", type=float, default=25.0)
    parser.add_argument(
        "--start-rviz", action="store_true",
        help="Start RViz to display the replayed curves (headless by default)")
    parser.add_argument(
        "--hold-seconds", type=float, default=0.0,
        help="Keep replay/RViz alive after the checks; defaults to 20 s "
             "when --start-rviz is used")
    args = parser.parse_args()
    hold_seconds = args.hold_seconds
    if args.start_rviz and hold_seconds <= 0.0:
        hold_seconds = 20.0

    if not os.path.isfile(args.trajectory_file):
        print(f"FAIL: trajectory file not found: {args.trajectory_file}")
        return 1

    log_path = os.path.join(tempfile.gettempdir(),
                            "excavator_offline_visualization_test.log")
    log = open(log_path, "w")
    mode = "RViz enabled" if args.start_rviz else "headless, RViz disabled"
    print(f"launching offline replay ({mode}) with {args.trajectory_file}")
    process = subprocess.Popen(
        [
            "ros2", "launch", "excavator_visualization", "replay.launch.py",
            f"trajectory_file:={args.trajectory_file}",
            "loop:=true",
            f"start_rviz:={'true' if args.start_rviz else 'false'}",
        ],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )

    rclpy.init()
    node = OfflineVisualizationCheck()
    failures = []
    try:
        ok = wait_until(
            node,
            lambda: (node.trajectory is not None and
                     node.planned_markers is not None and
                     node.end_effector_path is not None and
                     node.occupancy is not None and
                     node.status is not None and
                     node.joint_state_samples >= 10),
            args.timeout,
        )
        if not ok:
            failures.append("timed out waiting for visualization data")

        if node.trajectory is None:
            failures.append("/excavator/planned_trajectory not received")
        else:
            points = len(node.trajectory.points)
            names = list(node.trajectory.joint_names)
            print(f"trajectory: {points} points, joints={names}")
            if points < 5:
                failures.append("replayed trajectory has too few points")
            if len(names) != 4:
                failures.append("replayed trajectory must have 4 joints")

        if node.joint_state_samples < 10:
            failures.append("/joint_states stream is missing")
        else:
            print(f"joint_states samples: {node.joint_state_samples}")

        if node.status is None:
            failures.append("/excavator/execution_status not received")
        else:
            print(f"execution status: state={node.status.state}, "
                  f"elapsed={node.status.elapsed_time:.3f} s, "
                  f"duration={node.status.duration:.3f} s")
            if node.status.state not in (
                    ExecutionStatus.RUNNING, ExecutionStatus.COMPLETED):
                failures.append("offline replay status is not active")
            if node.trajectory is not None and node.trajectory.points:
                duration = to_seconds(
                    node.trajectory.points[-1].time_from_start)
                if abs(node.status.duration - duration) > 1.0e-3:
                    failures.append(
                        "execution status duration does not match trajectory")

        print(f"reference/measured consistency pairs: "
              f"{node.consistent_pairs}")
        if node.consistent_pairs < 5:
            failures.append(
                "replayed joint states do not match the reported "
                "trajectory time")

        if node.planned_markers is None:
            failures.append("/excavator/planned_path not received")
        else:
            markers = node.planned_markers.markers
            has_clear = any(
                marker.action == Marker.DELETEALL for marker in markers)
            print(f"planned_path markers: {len(markers)}")
            if len(markers) < 5 or not has_clear:
                failures.append("planned_path markers are incomplete")

        if node.end_effector_path is None:
            failures.append("/excavator/end_effector_path not received")
        else:
            poses = len(node.end_effector_path.poses)
            print(f"end_effector_path poses: {poses}")
            if node.trajectory is not None and poses != len(
                    node.trajectory.points):
                failures.append(
                    "end-effector path pose count does not match trajectory")

        if node.occupancy is None:
            failures.append("/excavator/trajectory_occupancy not received")
        else:
            meshes = [marker for marker in node.occupancy.markers
                      if marker.type == Marker.MESH_RESOURCE]
            namespaces = sorted({marker.ns for marker in meshes})
            alpha = meshes[0].color.a if meshes else 0.0
            print(f"occupancy markers: {len(node.occupancy.markers)}, "
                  f"mesh markers: {len(meshes)}, "
                  f"namespaces: {namespaces}, alpha: {alpha:.2f}")
            if len(meshes) < 8:
                failures.append("occupancy mesh markers are missing")
            if len(namespaces) < 2:
                failures.append("occupancy markers must cover multiple links")
            if meshes and any(
                    not marker.mesh_resource for marker in meshes):
                failures.append("occupancy marker has an empty mesh resource")
            if node.trajectory is not None:
                expected_pose_count = len(node.trajectory.points)
                if (node.end_effector_path is not None and
                        len(node.end_effector_path.poses) !=
                        expected_pose_count):
                    failures.append("visualization data is inconsistent")

        if not failures and hold_seconds > 0.0:
            print(f"keeping replay/RViz alive for {hold_seconds:.0f} s "
                  "(Ctrl+C to stop early)")
            hold_until = time.time() + hold_seconds
            while time.time() < hold_until:
                rclpy.spin_once(node, timeout_sec=0.05)

        if not failures:
            print("OFFLINE VISUALIZATION TEST: PASS")
            return 0
        print("OFFLINE VISUALIZATION TEST: FAIL")
        for failure in failures:
            print(f"  - {failure}")
        print(f"launch log: {log_path}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
        stop_process(process)
        log.close()


if __name__ == "__main__":
    raise SystemExit(main())
