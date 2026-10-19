#!/usr/bin/env python3
"""Headless end-to-end smoke test for the excavator framework.

Prerequisites (three terminals):
  ros2 launch excavator_sim_viz simulation.launch.py
  ros2 launch excavator_planning planning_server.launch.py
  ros2 launch excavator_visualization visualizer.launch.py

Then run:
  source install/setup.bash
  python3 scripts/e2e_smoke_test.py [--with-update]
"""

import argparse
import os
import time

# Match the default domain used by the project launch files, so that a plain
# `python3 scripts/e2e_smoke_test.py` still talks to them.
os.environ.setdefault("ROS_DOMAIN_ID", "77")

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from excavator_interfaces.action import ExecuteTrajectory, PlanTrajectory
from excavator_interfaces.msg import (
    ExecutionStatus,
    PlanningScene,
    TrajectoryUpdate,
)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ["swing_joint", "boom_joint", "stick_joint", "bucket_joint"]
START = [0.0, -0.35, 0.45, -0.4]
GOAL = [0.2, -0.3, 0.4, -0.3]
UPDATE_GOAL = [0.35, -0.28, 0.36, -0.34]


def duration(seconds):
    whole = int(seconds)
    return Duration(sec=whole, nanosec=int((seconds - whole) * 1e9))


class SmokeTest(Node):
    def __init__(self):
        super().__init__("excavator_e2e_smoke_test")
        self.joint_state = None
        self.status = None
        self.plan_target_ = list(GOAL)
        self.status_history = []
        self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, 10)
        self.create_subscription(
            ExecutionStatus, "/excavator/execution_status",
            self.on_status, 10)
        self.plan_client = ActionClient(self, PlanTrajectory, "/plan_trajectory")
        self.execute_client = ActionClient(
            self, ExecuteTrajectory, "/excavator/execute_trajectory")
        self.update_publisher = self.create_publisher(
            TrajectoryUpdate, "/excavator/trajectory_update", 10)

    def on_joint_state(self, msg):
        self.joint_state = msg

    def on_status(self, msg):
        self.status = msg
        if not self.status_history or self.status_history[-1] != msg.state:
            self.status_history.append(msg.state)
            self.get_logger().info(
                f"status={msg.state} progress={msg.progress:.2f} "
                f"message={msg.message}")

    def spin_for(self, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, predicate, timeout, description):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        raise RuntimeError(f"timed out waiting for {description}")

    def plan(self):
        if not self.plan_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("plan action server unavailable")
        self.wait_for(lambda: self.joint_state is not None, 10.0,
                      "an initial /joint_states sample")
        start = [self.joint_state.position[self.joint_state.name.index(name)]
                 for name in JOINTS]
        # Alternate between the two goals so the test always exercises a real
        # motion instead of a near-zero segment.
        self.plan_target_ = (
            list(START)
            if abs(start[0] - GOAL[0]) < abs(start[0] - START[0])
            else list(GOAL))
        goal = PlanTrajectory.Goal()
        goal.start_state.name = JOINTS
        goal.start_state.position = start
        goal.goal_state.name = JOINTS
        goal.goal_state.position = self.plan_target_
        goal.target_link = "bucket_tip"
        goal.allowed_planning_time = duration(2.0)
        goal.planning_scene = PlanningScene()
        send = self.plan_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if not handle.accepted:
            raise RuntimeError("plan goal rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=15.0)
        result = result_future.result().result
        self.get_logger().info(
            f"plan success={result.success} points={len(result.trajectory.points)} "
            f"message={result.message}")
        if not result.success:
            raise RuntimeError("planning failed")
        return result.trajectory

    def send_update(self):
        current = self.joint_state
        if current is None:
            raise RuntimeError("no joint state available for update")
        update = TrajectoryUpdate()
        update.header.stamp = self.get_clock().now().to_msg()
        update.header.frame_id = "base_link"
        update.update_mode = TrajectoryUpdate.REPLACE
        update.joint_names = JOINTS
        first = JointTrajectoryPoint()
        first.time_from_start = duration(0.0)
        first.positions = [current.position[current.name.index(n)] for n in JOINTS]
        second = JointTrajectoryPoint()
        second.time_from_start = duration(1.5)
        second.positions = list(UPDATE_GOAL)
        update.points = [first, second]
        self.update_publisher.publish(update)
        self.get_logger().info("published online trajectory update")

    def execute(self, trajectory, with_update):
        if not self.execute_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("execute action server unavailable")
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        goal.require_confirmation = True
        send = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if not handle.accepted:
            raise RuntimeError("execute goal rejected")

        if with_update:
            self.spin_for(0.5)
            self.send_update()
        result_future = handle.get_result_async()
        deadline = time.time() + 30.0
        while not result_future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not result_future.done():
            handle.cancel_goal_async()
            raise RuntimeError("execution timed out")
        result = result_future.result().result
        self.get_logger().info(
            f"execute success={result.success} message={result.message}")
        if not result.success:
            raise RuntimeError("execution failed")
        expected = UPDATE_GOAL if with_update else self.plan_target_

        def final_error():
            return max(abs(a - b) for a, b in zip(
                [self.joint_state.position[self.joint_state.name.index(n)]
                 for n in JOINTS], expected))

        try:
            self.wait_for(lambda: final_error() <= 0.03, 3.0,
                          "final joint positions")
        except RuntimeError:
            pass
        final = [self.joint_state.position[self.joint_state.name.index(n)]
                 for n in JOINTS]
        error = max(abs(a - b) for a, b in zip(final, expected))
        self.get_logger().info(
            f"final positions={['%.4f' % v for v in final]} max_error={error:.4f}")
        if error > 0.03:
            raise RuntimeError(f"final position error too large: {error}")
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--with-update", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    node = SmokeTest()
    try:
        trajectory = node.plan()
        node.execute(trajectory, args.with_update)
        print("E2E RESULT: PASS")
    except Exception as error:  # noqa: BLE001 - report all failures as failure
        print(f"E2E RESULT: FAIL: {error}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
