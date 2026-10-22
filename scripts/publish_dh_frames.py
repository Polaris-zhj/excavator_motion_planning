#!/usr/bin/env python3
"""Publish modified-DH frames for RViz inspection.

Frames:
  base_link -> dh_1 -> dh_2 -> dh_3 -> dh_4 -> dh_tool

The transforms use the same modified-DH convention as DhKinematicsModel:
  A_i = Rx(alpha_prev) Tx(a_prev) Rz(theta + theta_offset) Tz(d)

Run together with a robot model, for example:
  ros2 launch excavator_visualization visualization.launch.py \
      model:=excavator_box.urdf.xacro start_static_joint_states:=true

Then:
  python3 scripts/publish_dh_frames.py --q 0,0,0,0
"""

import argparse
import math
import os
import sys

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @
            np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @
            np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def transform(xyz, rpy):
    result = np.eye(4)
    result[:3, :3] = rpy_to_matrix(*rpy)
    result[:3, 3] = xyz
    return result


def rx(angle):
    result = np.eye(4)
    result[:3, :3] = rpy_to_matrix(angle, 0.0, 0.0)
    return result


def rz(angle):
    result = np.eye(4)
    result[:3, :3] = rpy_to_matrix(0.0, 0.0, angle)
    return result


def tx(distance):
    result = np.eye(4)
    result[0, 3] = distance
    return result


def tz(distance):
    result = np.eye(4)
    result[2, 3] = distance
    return result


def quaternion(matrix):
    trace = np.trace(matrix)
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale
    return x, y, z, w


class DhFrames(Node):
    def __init__(self, parameters, q, rate):
        super().__init__("dh_frames_publisher")
        self.parameters = parameters
        self.q = q
        self.broadcaster = TransformBroadcaster(self)
        latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/excavator/dh_frames", latched)
        self.timer = self.create_timer(1.0 / rate, self.publish)

    def frame_transforms(self):
        base = self.parameters.get("base") or {}
        current = transform(
            tuple(base.get("xyz", [0.0, 0.0, 0.0])),
            tuple(base.get("rpy", [0.0, 0.0, 0.0])))
        frames = []
        for index, joint in enumerate(self.parameters["joints"]):
            sign = float(joint.get("theta_sign", 1.0))
            if index == 0:
                current = current @ rz(sign * self.q[index])
            else:
                current = (current @ rx(float(joint.get("alpha_prev", 0.0))) @
                           tx(float(joint.get("a_prev", 0.0))) @
                           rz(sign * self.q[index] +
                              float(joint.get("theta_offset", 0.0))) @
                           tz(float(joint.get("d", 0.0))))
            frames.append((joint["name"], current.copy()))
        tool = (self.parameters.get("links") or {}).get("bucket_tip")
        if tool is not None:
            current = current @ transform(
                tuple(tool.get("xyz", [0.0, 0.0, 0.0])),
                tuple(tool.get("rpy", [0.0, 0.0, 0.0])))
            frames.append(("bucket_tip_dh", current.copy()))
        return frames

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        frames = self.frame_transforms()
        parent = "base_link"
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        marker_id = 0
        for name, matrix in frames:
            label = "bucket_tip" if name == "bucket_tip_dh" else name
            message = TransformStamped()
            message.header.stamp = stamp
            message.header.frame_id = parent
            message.child_frame_id = name
            message.transform.translation.x = matrix[0, 3]
            message.transform.translation.y = matrix[1, 3]
            message.transform.translation.z = matrix[2, 3]
            x, y, z, w = quaternion(matrix[:3, :3])
            message.transform.rotation.x = x
            message.transform.rotation.y = y
            message.transform.rotation.z = z
            message.transform.rotation.w = w
            self.broadcaster.sendTransform(message)
            parent = name

            origin = matrix[:3, 3]
            for axis, color in enumerate((
                    (1.0, 0.2, 0.2), (0.2, 1.0, 0.2),
                    (0.2, 0.4, 1.0))):
                marker = Marker()
                marker.header.stamp = stamp
                marker.header.frame_id = "base_link"
                marker.ns = name
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.LINE_LIST
                marker.action = Marker.ADD
                marker.scale.x = 0.035
                marker.color.r, marker.color.g, marker.color.b = color
                marker.color.a = 1.0
                start = origin
                end = origin + matrix[:3, :3][:, axis] * 0.55
                for point in (start, end):
                    p = type(marker.pose.position)()
                    p.x, p.y, p.z = float(point[0]), float(point[1]), float(point[2])
                    marker.points.append(p)
                markers.markers.append(marker)

            sphere = Marker()
            sphere.header.stamp = stamp
            sphere.header.frame_id = "base_link"
            sphere.ns = name + "_origin"
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.scale.x = 0.06
            sphere.scale.y = 0.06
            sphere.scale.z = 0.06
            sphere.color.r = 1.0
            sphere.color.g = 1.0
            sphere.color.b = 1.0
            sphere.color.a = 1.0
            sphere.pose.position.x = float(origin[0])
            sphere.pose.position.y = float(origin[1])
            sphere.pose.position.z = float(origin[2])
            sphere.pose.orientation.w = 1.0
            markers.markers.append(sphere)

            text = Marker()
            text.header.stamp = stamp
            text.header.frame_id = "base_link"
            text.ns = name + "_label"
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.scale.z = 0.18
            text.color.r = 1.0
            text.color.g = 0.85
            text.color.b = 0.2
            text.color.a = 1.0
            text.pose.position.x = float(origin[0])
            text.pose.position.y = float(origin[1])
            text.pose.position.z = float(origin[2] + 0.10)
            text.pose.orientation.w = 1.0
            text.text = label
            markers.markers.append(text)
        self.marker_publisher.publish(markers)


def main():
    parser = argparse.ArgumentParser(description="Publish modified-DH frames")
    parser.add_argument(
        "--dh-config",
        default=os.path.join(PROJECT_ROOT, "src", "excavator_description",
                             "config", "excavator_dh.yaml"))
    parser.add_argument(
        "--q", default="0,0,0,0",
        help="comma-separated active joint positions")
    parser.add_argument("--rate", type=float, default=20.0)
    args = parser.parse_args()
    with open(args.dh_config, "r", encoding="utf-8") as stream:
        parameters = yaml.safe_load(stream)
    q = [float(value) for value in args.q.split(",")]
    if len(q) != len(parameters.get("joints", [])):
        print("FAIL: --q must provide one value per DH joint", file=sys.stderr)
        return 1

    rclpy.init()
    node = DhFrames(parameters, q, args.rate)
    try:
        print("publishing dh_1..dh_4 and dh_tool under base_link; Ctrl+C to stop")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
