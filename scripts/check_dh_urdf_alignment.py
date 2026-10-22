#!/usr/bin/env python3
"""Check that a generated DH URDF keeps the source model's visual posture.

It compares the world pose of every visual geometry common to the source and
generated models at several active-joint configurations. DH link frames may
differ; only the resulting geometry poses must match.

Usage:
  python3 scripts/check_dh_urdf_alignment.py \
      --source-model src/excavator_description/urdf/excavator_box.urdf.xacro \
      --generated src/excavator_description/urdf/excavator_dh.urdf
"""

import argparse
import math
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

JOINTS = ["swing_joint", "boom_joint", "stick_joint", "bucket_joint"]


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def transform(xyz, rpy):
    result = np.eye(4)
    result[:3, :3] = rpy_to_matrix(*rpy)
    result[:3, 3] = xyz
    return result


def parse_origin(element):
    origin = element.find("origin")
    if origin is None:
        return np.eye(4)
    xyz = tuple(float(value)
                for value in origin.get("xyz", "0 0 0").split())
    rpy = tuple(float(value)
                for value in origin.get("rpy", "0 0 0").split())
    return transform(xyz, rpy)


def axis_rotation(axis, angle):
    axis = np.array(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    rotation = np.array([
        [c + x * x * (1 - c), x * y * (1 - c) - z * s,
         x * z * (1 - c) + y * s],
        [y * x * (1 - c) + z * s, c + y * y * (1 - c),
         y * z * (1 - c) - x * s],
        [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s,
         c + z * z * (1 - c)],
    ])
    result = np.eye(4)
    result[:3, :3] = rotation
    return result


def load_model(path):
    if path.endswith(".xacro"):
        xacro = shutil.which("xacro")
        if xacro is None:
            raise RuntimeError("xacro executable not found")
        xml = subprocess.check_output([xacro, path])
    else:
        with open(path, "rb") as stream:
            xml = stream.read()
    root = ET.fromstring(xml)
    links = {link.get("name"): link for link in root.findall("link")}
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    parents = {}
    for name, joint in joints.items():
        child = joint.find("child").get("link")
        parent = joint.find("parent").get("link")
        parents[child] = (name, parent)
    return links, joints, parents


def visual_poses(path, q):
    links, joints, parents = load_model(path)
    cache = {}

    def link_pose(link_name):
        if link_name in cache:
            return cache[link_name]
        if link_name not in parents:
            cache[link_name] = np.eye(4)
            return cache[link_name]
        joint_name, parent = parents[link_name]
        joint = joints[joint_name]
        base = link_pose(parent)
        joint_type = joint.get("type")
        if joint_type in ("revolute", "continuous", "prismatic"):
            axis_element = joint.find("axis")
            axis = (tuple(float(value) for value in
                          axis_element.get("xyz", "1 0 0").split())
                    if axis_element is not None else (1.0, 0.0, 0.0))
            value = q.get(joint_name, 0.0)
            if joint_type == "prismatic":
                motion = np.eye(4)
                direction = np.array(axis, dtype=float)
                direction /= np.linalg.norm(direction)
                motion[:3, 3] = direction * value
            else:
                motion = axis_rotation(axis, value)
            pose = base @ parse_origin(joint) @ motion
        else:
            pose = base @ parse_origin(joint)
        cache[link_name] = pose
        return pose

    poses = {}
    for link_name, link in links.items():
        for visual in link.findall("visual"):
            poses.setdefault(link_name, []).append(
                link_pose(link_name) @ parse_origin(visual))
    return poses


def main():
    parser = argparse.ArgumentParser(
        description="Compare visual poses of a source and a DH URDF")
    parser.add_argument(
        "--source-model",
        default=os.path.join(
            os.path.dirname(__file__), "..", "src",
            "excavator_description", "urdf", "excavator_box.urdf.xacro"))
    parser.add_argument(
        "--generated",
        default=os.path.join(
            os.path.dirname(__file__), "..", "src",
            "excavator_description", "urdf", "excavator_dh.urdf"))
    parser.add_argument("--tolerance", type=float, default=1.0e-6)
    args = parser.parse_args()

    configurations = [
        ("zero", {name: 0.0 for name in JOINTS}),
        ("home", {"swing_joint": 0.0, "boom_joint": -0.35,
                  "stick_joint": 0.45, "bucket_joint": -0.4}),
        ("swing", {"swing_joint": 0.3, "boom_joint": 0.0,
                   "stick_joint": 0.0, "bucket_joint": 0.0}),
        ("mixed", {"swing_joint": 0.3, "boom_joint": -0.25,
                   "stick_joint": 0.5, "bucket_joint": -0.3}),
    ]
    max_position = 0.0
    max_rotation = 0.0
    for label, q in configurations:
        source = visual_poses(args.source_model, q)
        generated = visual_poses(args.generated, q)
        common = sorted(set(source) & set(generated))
        if not common:
            print("FAIL: no common visual links", file=sys.stderr)
            return 1
        for link in common:
            for source_pose, generated_pose in zip(source[link],
                                                    generated[link]):
                position_error = float(
                    np.linalg.norm(source_pose[:3, 3] - generated_pose[:3, 3]))
                rotation = source_pose[:3, :3].T @ generated_pose[:3, :3]
                cosine = max(-1.0, min(1.0, (np.trace(rotation) - 1.0) / 2.0))
                rotation_error = math.acos(cosine)
                max_position = max(max_position, position_error)
                max_rotation = max(max_rotation, rotation_error)
                print(f"{label:5s} {link:16s} "
                      f"pos_err={position_error:.3e} "
                      f"rot_err={rotation_error:.3e}")

    print(f"max position error: {max_position:.3e} m")
    print(f"max rotation error: {max_rotation:.3e} rad")
    if max_position > args.tolerance or max_rotation > args.tolerance:
        print("DH URDF ALIGNMENT: FAIL")
        return 1
    print("DH URDF ALIGNMENT: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
