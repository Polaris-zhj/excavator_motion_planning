#!/usr/bin/env python3
"""Generate a DH-frame URDF from the modified-DH parameter file.

The generated URDF keeps the same link/joint names and physical geometry as
the source URDF, but the four main link frames are the modified-DH frames:

    A_i = Rx(alpha_prev) Tx(a_prev) Rz(theta + theta_offset) Tz(d)

Visual/collision/inertial origins of the main links are transformed from the
physical URDF frames into the DH frames. By default only the four-link DH
chain is emitted. The hydraulic-cylinder/linkage branches are decorative and
are omitted; pass --include-passive to keep them with transformed attachment
origins.

Usage:
  python3 scripts/generate_dh_urdf.py
  python3 scripts/generate_dh_urdf.py \
      --source-model src/excavator_description/urdf/excavator_box.urdf.xacro \
      --output src/excavator_description/urdf/excavator_dh.urdf
"""

import argparse
import math
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def matrix_to_rpy(rotation):
    sp = -rotation[2, 0]
    sp = max(-1.0, min(1.0, sp))
    cp = math.sqrt(max(0.0, 1.0 - sp * sp))
    pitch = math.atan2(sp, cp)
    if abs(cp) > 1.0e-8:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = 0.0
        yaw = math.atan2(-rotation[0, 1], rotation[1, 1])
    return roll, pitch, yaw


def transform(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    result = np.eye(4)
    result[:3, :3] = rpy_to_matrix(*rpy)
    result[:3, 3] = xyz
    return result


def inverse(transform_matrix):
    result = np.eye(4)
    rotation = transform_matrix[:3, :3]
    translation = transform_matrix[:3, 3]
    result[:3, :3] = rotation.T
    result[:3, 3] = -rotation.T @ translation
    return result


def parse_xyz_rpy(node):
    xyz = node.get("xyz", "0 0 0").split()
    rpy = node.get("rpy", "0 0 0").split()
    return (
        tuple(float(value) for value in xyz),
        tuple(float(value) for value in rpy),
    )


def parse_origin(element):
    origin = element.find("origin")
    if origin is None:
        return np.eye(4)
    xyz, rpy = parse_xyz_rpy(origin)
    return transform(xyz, rpy)


def set_origin(element, matrix):
    origin = element.find("origin")
    if origin is None:
        origin = ET.Element("origin")
        element.insert(0, origin)
    xyz = matrix[:3, 3]
    rpy = matrix_to_rpy(matrix[:3, :3])
    origin.set("xyz", " ".join(format_number(value) for value in xyz))
    origin.set("rpy", " ".join(format_number(value) for value in rpy))


def format_number(value):
    if abs(value) < 5.0e-10:
        value = 0.0
    text = f"{value:.9f}".rstrip("0").rstrip(".")
    return text if text not in ("", "-0") else "0"


def dh_fixed_transform(joint):
    alpha = float(joint.get("alpha_prev", 0.0))
    a = float(joint.get("a_prev", 0.0))
    d = float(joint.get("d", 0.0))
    offset = float(joint.get("theta_offset", 0.0))
    rx = np.eye(4)
    rx[:3, :3] = rpy_to_matrix(alpha, 0.0, 0.0)
    tx = np.eye(4)
    tx[0, 3] = a
    rz = np.eye(4)
    rz[:3, :3] = rpy_to_matrix(0.0, 0.0, offset)
    tz = np.eye(4)
    tz[2, 3] = d
    return rx @ tx @ rz @ tz


def transform_inertia(element, physical_to_dh):
    inertia = element.find("inertial")
    if inertia is None:
        return
    set_origin(inertia, physical_to_dh @ parse_origin(inertia))
    inertia_node = inertia.find("inertia")
    if inertia_node is None:
        return
    try:
        values = {key: float(inertia_node.get(key, "0"))
                  for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
    except ValueError:
        return
    matrix = np.array([
        [values["ixx"], values["ixy"], values["ixz"]],
        [values["ixy"], values["iyy"], values["iyz"]],
        [values["ixz"], values["iyz"], values["izz"]],
    ])
    rotation = physical_to_dh[:3, :3]
    rotated = rotation @ matrix @ rotation.T
    inertia_node.set("ixx", format_number(rotated[0, 0]))
    inertia_node.set("iyy", format_number(rotated[1, 1]))
    inertia_node.set("izz", format_number(rotated[2, 2]))
    inertia_node.set("ixy", format_number(rotated[0, 1]))
    inertia_node.set("ixz", format_number(rotated[0, 2]))
    inertia_node.set("iyz", format_number(rotated[1, 2]))


def main():
    parser = argparse.ArgumentParser(
        description="Generate a DH-frame URDF from modified-DH parameters")
    parser.add_argument(
        "--dh-config",
        default=os.path.join(PROJECT_ROOT, "src", "excavator_description",
                             "config", "excavator_dh.yaml"))
    parser.add_argument(
        "--source-model",
        default=os.path.join(PROJECT_ROOT, "src", "excavator_description",
                             "urdf", "excavator_box.urdf.xacro"))
    parser.add_argument(
        "--output",
        default=os.path.join(PROJECT_ROOT, "src", "excavator_description",
                             "urdf", "excavator_dh.urdf"))
    parser.add_argument("--robot-name", default="excavator_dh")
    parser.add_argument(
        "--include-passive", action="store_true",
        help="Keep hydraulic-cylinder/linkage visual branches (default: omit; "
             "they are not part of the DH chain)")
    args = parser.parse_args()

    with open(args.dh_config, "r", encoding="utf-8") as stream:
        parameters = yaml.safe_load(stream)
    if parameters.get("convention", "modified_dh") != "modified_dh":
        print("FAIL: only modified_dh is supported", file=sys.stderr)
        return 1

    xacro = shutil.which("xacro")
    if xacro is None:
        print("FAIL: xacro executable not found in PATH", file=sys.stderr)
        return 1
    try:
        source_xml = subprocess.check_output(
            [xacro, args.source_model], stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as error:
        print("FAIL: xacro failed:\n" + error.output.decode(errors="replace"),
              file=sys.stderr)
        return 1

    root = ET.fromstring(source_xml)
    root.set("name", args.robot_name)

    base = parameters.get("base") or {}
    base_transform = transform(
        tuple(base.get("xyz", [0.0, 0.0, 0.0])),
        tuple(base.get("rpy", [0.0, 0.0, 0.0])))
    offsets = {}
    for link_name, node in (parameters.get("links") or {}).items():
        offsets[link_name] = transform(
            tuple(node.get("xyz", [0.0, 0.0, 0.0])),
            tuple(node.get("rpy", [0.0, 0.0, 0.0])))

    joints = parameters.get("joints") or []
    joint_by_name = {joint["name"]: joint for joint in joints}
    main_links = {joint["link"] for joint in joints}
    main_links.add("base_link")
    main_joint_names = set(joint_by_name)

    # Main joint origins/axes: modified DH fixed transform + z axis.
    for joint_element in root.findall("joint"):
        name = joint_element.get("name", "")
        if name not in joint_by_name:
            continue
        joint = joint_by_name[name]
        if name == joints[0]["name"]:
            origin = base_transform
        else:
            origin = dh_fixed_transform(joint)
        set_origin(joint_element, origin)
        axis = joint_element.find("axis")
        if axis is None:
            axis = ET.SubElement(joint_element, "axis")
        sign = float(joint.get("theta_sign", 1.0))
        axis.set("xyz", f"0 0 {sign:g}")

    # Main link geometry: express physical visual/collision/inertial geometry
    # in the DH link frame.
    for link_element in root.findall("link"):
        link_name = link_element.get("name", "")
        if link_name not in main_links:
            continue
        # T_phys = T_DH * offset, so geometry expressed in the DH frame is
        # offset * origin_phys (not inverse(offset)).
        physical_to_dh = offsets.get(link_name, np.eye(4))
        for visual in link_element.findall("visual"):
            set_origin(visual, physical_to_dh @ parse_origin(visual))
        for collision in link_element.findall("collision"):
            set_origin(collision, physical_to_dh @ parse_origin(collision))
        transform_inertia(link_element, physical_to_dh)

    # Passive branches attached to a main link: the parent frame is now the
    # DH frame, so transform the attachment origin. Their own child chains stay
    # in their original physical frames.
    if args.include_passive:
        for joint_element in root.findall("joint"):
            name = joint_element.get("name", "")
            if name in main_joint_names or name == "bucket_tip_joint":
                continue
            parent_element = joint_element.find("parent")
            if parent_element is None:
                continue
            parent = parent_element.get("link", "")
            if parent not in offsets:
                continue
            set_origin(joint_element,
                       offsets[parent] @ parse_origin(joint_element))
    else:
        # A DH model is a four-link serial chain. Drop the decorative
        # hydraulic/linkage branches and their ros2_control entries.
        keep_links = set(main_links)
        keep_links.update({"world", "bucket_tip"})
        for link_element in list(root.findall("link")):
            if link_element.get("name", "") not in keep_links:
                root.remove(link_element)
        keep_joints = set(main_joint_names)
        keep_joints.update({"world_to_base_link", "bucket_tip_joint"})
        for joint_element in list(root.findall("joint")):
            if joint_element.get("name", "") not in keep_joints:
                root.remove(joint_element)
        for control in root.findall("ros2_control"):
            for joint_element in list(control.findall("joint")):
                if joint_element.get("name", "") not in main_joint_names:
                    control.remove(joint_element)

    # Tool frame: DH frame 4 -> bucket_tip, using the YAML link offset.
    for joint_element in root.findall("joint"):
        if joint_element.get("name") == "bucket_tip_joint":
            set_origin(joint_element,
                       offsets.get("bucket_tip", np.eye(4)))
            break

    ET.indent(root, space="  ")
    output_dir = os.path.dirname(os.path.abspath(args.output))
    os.makedirs(output_dir, exist_ok=True)
    header = (
        "<!-- Generated by scripts/generate_dh_urdf.py.\n"
        "     Modified-DH frames; do not edit by hand. -->\n")
    body = ET.tostring(root, encoding="unicode")
    with open(args.output, "w", encoding="utf-8") as stream:
        stream.write('<?xml version="1.0" encoding="utf-8"?>\n')
        stream.write(header)
        stream.write(body)
        stream.write("\n")

    print(f"DH URDF generated: {args.output}")
    print(f"  base: {args.dh_config}")
    print(f"  source: {args.source_model}")
    print(f"  main joints: {', '.join(joint_by_name)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
