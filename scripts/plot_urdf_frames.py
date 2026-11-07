#!/usr/bin/env python3
"""Plot the link frames of a URDF/xacro model (no ROS runtime required).

The script parses the URDF, evaluates forward kinematics for a given joint
configuration and draws every selected link frame: X red, Y green, Z blue.
Xacro files are expanded with the ``xacro`` executable when available.

By default the main chain ending at ``bucket_tip`` is drawn, which keeps the
hydraulic-cylinder/linkage branches of the excavator out of the picture. Use
``--all`` to draw every link.

Usage:
  python3 scripts/plot_urdf_frames.py
  python3 scripts/plot_urdf_frames.py --q 0,-0.35,0.45,-0.4
  python3 scripts/plot_urdf_frames.py --model src/excavator_description/urdf/excavator_dh.urdf
  python3 scripts/plot_urdf_frames.py --reference base_link --output /tmp/urdf_frames.png
"""

import argparse
import glob
import math
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MOVABLE_TYPES = ("revolute", "continuous", "prismatic")


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


def parse_origin(element):
    origin = element.find("origin") if element is not None else None
    if origin is None:
        return np.eye(4)
    xyz = tuple(float(value) for value in origin.get("xyz", "0 0 0").split())
    rpy = tuple(float(value) for value in origin.get("rpy", "0 0 0").split())
    return transform(xyz, rpy)


def parse_vector(element, default):
    if element is None:
        return default
    text = element.get("xyz")
    return (tuple(float(value) for value in text.split())
            if text else default)


def axis_rotation(axis, angle):
    axis = np.array(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1.0e-12:
        return np.eye(4)
    x, y, z = axis / norm
    c, s = math.cos(angle), math.sin(angle)
    result = np.eye(4)
    result[:3, :3] = np.array([
        [c + x * x * (1 - c), x * y * (1 - c) - z * s,
         x * z * (1 - c) + y * s],
        [y * x * (1 - c) + z * s, c + y * y * (1 - c),
         y * z * (1 - c) - x * s],
        [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s,
         c + z * z * (1 - c)],
    ])
    return result


def axis_translation(axis, distance):
    result = np.eye(4)
    direction = np.array(axis, dtype=float)
    norm = np.linalg.norm(direction)
    if norm > 1.0e-12:
        result[:3, 3] = direction / norm * distance
    return result


def find_xacro(explicit):
    if explicit:
        if os.path.isfile(explicit):
            return explicit
        found = shutil.which(explicit)
        if found:
            return found
        raise SystemExit(f"xacro executable not found: {explicit}")
    found = shutil.which("xacro")
    if found:
        return found
    # The shell running this script may not have sourced ROS yet.
    candidates = sorted(glob.glob("/opt/ros/*/bin/xacro"))
    if candidates:
        return candidates[-1]
    return None


def load_urdf(path, xacro_arguments, xacro_executable):
    if path.endswith(".xacro"):
        xacro = find_xacro(xacro_executable)
        if xacro is None:
            raise SystemExit(
                "xacro executable not found; source ROS or pass --xacro")
        command = [xacro, path] + list(xacro_arguments)
        environment = os.environ.copy()
        # xacro installed by ROS needs the ROS Python packages on PYTHONPATH;
        # that is not guaranteed when this script runs outside a sourced shell.
        ros_python_paths = (
            sorted(glob.glob("/opt/ros/*/local/lib/python3.*/dist-packages")) +
            sorted(glob.glob("/opt/ros/*/lib/python3.*/site-packages")))
        if ros_python_paths:
            existing = environment.get("PYTHONPATH")
            environment["PYTHONPATH"] = os.pathsep.join(
                ros_python_paths + ([existing] if existing else []))
        try:
            xml = subprocess.check_output(
                command, stderr=subprocess.PIPE, env=environment)
        except subprocess.CalledProcessError as error:
            sys.stderr.write(error.stderr.decode("utf-8", "replace"))
            raise SystemExit(f"xacro failed: {' '.join(command)}")
    else:
        with open(path, "rb") as stream:
            xml = stream.read()
    return ET.fromstring(xml)


class UrdfModel:
    def __init__(self, root):
        self.links = {}
        self.joints = {}
        self.parents = {}
        for link in root.findall("link"):
            name = link.get("name")
            self.links[name] = {"element": link}
        for joint in root.findall("joint"):
            name = joint.get("name")
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            mimic = joint.find("mimic")
            self.joints[name] = {
                "name": name,
                "type": joint.get("type", "fixed"),
                "parent": parent,
                "child": child,
                "origin": parse_origin(joint),
                "axis": parse_vector(joint.find("axis"), (1.0, 0.0, 0.0)),
                "mimic": ({
                    "joint": mimic.get("joint"),
                    "multiplier": float(mimic.get("multiplier", "1")),
                    "offset": float(mimic.get("offset", "0")),
                } if mimic is not None else None),
            }
            self.parents[child] = name
        if not self.links:
            raise SystemExit("no <link> elements found in the model")
        self.roots = [name for name in self.links if name not in self.parents]

    def chain_to(self, tip):
        if tip not in self.links:
            return None
        chain = [tip]
        while chain[-1] in self.parents:
            parent = self.joints[self.parents[chain[-1]]]["parent"]
            if parent in chain:
                return None
            chain.append(parent)
        chain.reverse()
        return chain

    def chain_joints(self, links):
        return [self.parents[link] for link in links[1:] if link in self.parents]

    def breadth_first_links(self):
        order = []
        queue = list(self.roots)
        while queue:
            link = queue.pop(0)
            if link in order:
                continue
            order.append(link)
            for joint in self.joints.values():
                if joint["parent"] == link and joint["child"] not in order:
                    queue.append(joint["child"])
        for name in self.links:
            if name not in order:
                order.append(name)
        return order


def joint_value(model, joint, q, cache=None):
    cache = cache if cache is not None else {}
    name = joint["name"]
    if name in cache:
        return cache[name]
    cache[name] = 0.0
    mimic = joint["mimic"]
    if mimic is not None and mimic["joint"] in model.joints:
        source = model.joints[mimic["joint"]]
        value = (mimic["multiplier"] * joint_value(model, source, q, cache) +
                 mimic["offset"])
    elif joint["type"] in MOVABLE_TYPES:
        value = q.get(name, 0.0)
    else:
        value = 0.0
    cache[name] = value
    return value


def compute_poses(model, q):
    poses = {}

    def link_pose(name):
        if name in poses:
            return poses[name]
        joint_name = model.parents.get(name)
        if joint_name is None:
            poses[name] = np.eye(4)
            return poses[name]
        joint = model.joints[joint_name]
        parent_pose = link_pose(joint["parent"])
        poses[name] = np.eye(4)  # cycle guard
        joint_frame = parent_pose @ joint["origin"]
        value = joint_value(model, joint, q)
        if joint["type"] in ("revolute", "continuous"):
            motion = axis_rotation(joint["axis"], value)
        elif joint["type"] == "prismatic":
            motion = axis_translation(joint["axis"], value)
        else:
            motion = np.eye(4)
        poses[name] = joint_frame @ motion
        return poses[name]

    for name in model.links:
        link_pose(name)
    return poses


def parse_joint_overrides(values):
    overrides = {}
    for value in values:
        if "=" not in value:
            raise SystemExit(f"--joint expects NAME=VALUE, got {value!r}")
        name, raw = value.split("=", 1)
        overrides[name.strip()] = float(raw)
    return overrides


def auto_axis_length(points):
    if len(points) < 2:
        return 0.5
    lower, upper = points.min(axis=0), points.max(axis=0)
    return float(np.clip(0.07 * np.max(upper - lower), 0.15, 0.8))


def frame_label_offset(name, index):
    known = {
        "world": (-0.75, 0.0, -0.15),
        "base_link": (0.25, 0.0, -0.35),
        "swing_link": (-0.85, -0.45, 0.35),
        "boom_link": (0.55, 0.50, 0.30),
        "stick_link": (-0.40, 0.35, 0.30),
        "bucket_link": (-0.65, 0.35, 0.65),
        "bucket_tip": (0.30, -0.15, -0.35),
    }
    if name in known:
        return known[name]
    side = -1.0 if index % 2 else 1.0
    return (0.15, 0.15 * side, 0.15)


def main():
    parser = argparse.ArgumentParser(description="Plot URDF link frames")
    parser.add_argument(
        "--model",
        default=os.path.join(PROJECT_ROOT, "src", "excavator_description",
                             "urdf", "excavator_box.urdf.xacro"),
        help="URDF or xacro file")
    parser.add_argument(
        "--tip", default="bucket_tip",
        help="plot the chain from the root to this link (default: bucket_tip)")
    parser.add_argument(
        "--all", action="store_true",
        help="plot every link instead of the root-to-tip chain")
    parser.add_argument(
        "--reference", default=None,
        help="express and draw all frames relative to this link")
    parser.add_argument(
        "--q", default=None,
        help="comma-separated joint values in chain order, "
             "e.g. 0,-0.35,0.45,-0.4")
    parser.add_argument(
        "--joint", action="append", default=[], metavar="NAME=VALUE",
        help="override a single joint value (repeatable)")
    parser.add_argument("--axis-length", type=float, default=None)
    parser.add_argument("--show-joint-axes", action="store_true",
                        help="draw the rotation/translation axis of each joint")
    parser.add_argument("--xacro", default=None,
                        help="path/name of the xacro executable")
    parser.add_argument("--xacro-arg", action="append", default=[],
                        metavar="NAME:=VALUE", help="argument passed to xacro")
    parser.add_argument(
        "--output",
        default=os.path.join(PROJECT_ROOT, "docs", "screenshots",
                             "urdf_frames.png"))
    args = parser.parse_args()

    if not os.path.isfile(args.model):
        raise SystemExit(f"model not found: {args.model}")
    model = UrdfModel(load_urdf(args.model, args.xacro_arg, args.xacro))

    chain = None if args.all else model.chain_to(args.tip)
    if chain is None:
        if not args.all:
            print(f"tip link {args.tip!r} not found; plotting all links",
                  file=sys.stderr)
        links_to_plot = model.breadth_first_links()
        joint_path = []
    else:
        links_to_plot = chain
        joint_path = model.chain_joints(chain)

    if joint_path:
        ordered_joints = joint_path
    else:
        ordered_joints = []
        for link in links_to_plot:
            joint_name = model.parents.get(link)
            if joint_name is not None:
                ordered_joints.append(joint_name)
    movable = [name for name in ordered_joints
               if model.joints[name]["type"] in MOVABLE_TYPES and
               model.joints[name]["mimic"] is None]
    if args.q is None:
        q_values = [0.0] * len(movable)
    else:
        q_values = [float(value) for value in args.q.split(",")]
        if len(q_values) != len(movable):
            raise SystemExit(
                f"--q needs {len(movable)} values for ({', '.join(movable)}), "
                f"got {len(q_values)}")
    q = dict(zip(movable, q_values))
    q.update(parse_joint_overrides(args.joint))

    poses = compute_poses(model, q)
    reference = args.reference
    if reference is None:
        reference = next((name for name in model.roots
                          if name in links_to_plot), model.roots[0])
    if reference not in poses:
        raise SystemExit(f"reference link not found: {reference}")
    reference_inverse = np.linalg.inv(poses[reference])
    frames = [(name, reference_inverse @ poses[name]) for name in links_to_plot]
    joint_frames = []
    for name in joint_path:
        joint = model.joints[name]
        parent_pose = poses[joint["parent"]]
        joint_pose = reference_inverse @ parent_pose @ joint["origin"]
        joint_frames.append((name, joint, joint_pose))

    points = np.array([matrix[:3, 3] for _, matrix in frames])
    axis_length = (args.axis_length if args.axis_length is not None
                   else auto_axis_length(points))

    figure = plt.figure(figsize=(15, 9), facecolor="#202226")
    axis = figure.add_subplot(111, projection="3d")
    figure.subplots_adjust(left=0.02, right=0.67, bottom=0.04, top=0.92)
    axis.set_facecolor("#202226")
    colors = ("#ff4d4d", "#4dff6a", "#4d7dff")
    label_offsets = {
        name: frame_label_offset(name, index)
        for index, (name, _) in enumerate(frames)}

    if args.show_joint_axes:
        for name, joint, joint_pose in joint_frames:
            if joint["type"] not in MOVABLE_TYPES:
                continue
            direction = joint_pose[:3, :3] @ np.array(joint["axis"], float)
            norm = np.linalg.norm(direction)
            if norm < 1.0e-12:
                continue
            # Arrow along the URDF <axis> direction, so the sign of the
            # axis vector (and hence the sign of a positive q) is visible.
            direction = direction / norm * axis_length * 0.9
            origin = joint_pose[:3, 3]
            axis.quiver(origin[0], origin[1], origin[2],
                        direction[0], direction[1], direction[2],
                        color="#e07cff", arrow_length_ratio=0.30,
                        linewidth=1.6)

    previous_origin = None
    table_rows = []
    for name, matrix in frames:
        origin = matrix[:3, 3]
        if previous_origin is not None:
            axis.plot([previous_origin[0], origin[0]],
                      [previous_origin[1], origin[1]],
                      [previous_origin[2], origin[2]],
                      color="#d6c85a", linewidth=2.0)
        previous_origin = origin
        for column, color in enumerate(colors):
            direction = matrix[:3, :3][:, column] * axis_length
            axis.quiver(origin[0], origin[1], origin[2],
                        direction[0], direction[1], direction[2],
                        color=color, arrow_length_ratio=0.18, linewidth=2.0)
        axis.scatter([origin[0]], [origin[1]], [origin[2]],
                     color="white", s=28)
        offset = label_offsets[name]
        axis.text(origin[0] + offset[0], origin[1] + offset[1],
                  origin[2] + offset[2], name, color="white", fontsize=10)
        table_rows.append(
            f"{name:20s} "
            f"({origin[0]:7.3f}, {origin[1]:7.3f}, {origin[2]:7.3f})")

    lower = points.min(axis=0) - 1.0
    upper = points.max(axis=0) + 1.0
    center = 0.5 * (lower + upper)
    radius = max(0.5 * float(np.max(upper - lower)), 0.5)
    axis.set_xlim(center[0] - radius, center[0] + radius)
    axis.set_ylim(center[1] - radius, center[1] + radius)
    axis.set_zlim(center[2] - radius, center[2] + radius)
    axis.set_box_aspect((1, 1, 1))
    axis.set_xlabel("X [m]", color="#bbbbbb")
    axis.set_ylabel("Y [m]", color="#bbbbbb")
    axis.set_zlabel("Z [m]", color="#bbbbbb")
    axis.tick_params(colors="#999999")
    axis.view_init(elev=24, azim=-62)
    axis.grid(True, color="#44484f")
    for pane in (axis.xaxis.pane, axis.yaxis.pane, axis.zaxis.pane):
        pane.set_facecolor("#25282d")
        pane.set_edgecolor("#44484f")
    title_q = ", ".join(f"{q.get(name, 0.0):g}" for name in movable)
    axis.set_title(
        f"URDF link frames relative to {reference} (q = {title_q})",
        color="white", pad=14)
    table = "link / frame          origin xyz [m]\n" + "\n".join(table_rows)
    figure.text(0.685, 0.91,
                f"URDF frame origins ({reference})\n\n" + table +
                "\n\nX: red   Y: green   Z: blue"
                "\n\nGolden lines connect each link"
                "\nto its parent."
                + ("\n\nMagenta arrows: joint axes"
                   "\n(along the URDF <axis> vector)." if
                   args.show_joint_axes else ""),
                color="#dddddd", fontsize=8.2, family="monospace", va="top")
    axis.text2D(0.02, 0.96, "X", transform=axis.transAxes,
                color=colors[0], fontsize=13, fontweight="bold")
    axis.text2D(0.055, 0.96, "Y", transform=axis.transAxes,
                color=colors[1], fontsize=13, fontweight="bold")
    axis.text2D(0.09, 0.96, "Z", transform=axis.transAxes,
                color=colors[2], fontsize=13, fontweight="bold")

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    figure.tight_layout()
    figure.savefig(args.output, dpi=140, facecolor=figure.get_facecolor())
    print(f"URDF frame view: {args.output}")
    print(f"model: {args.model}")
    print(f"reference: {reference}")
    if movable:
        print("q: " + ", ".join(f"{name}={q.get(name, 0.0):g}"
                                for name in movable))
    for name, matrix in frames:
        origin = matrix[:3, 3]
        print(f"  {name:20s} ({origin[0]:8.4f}, {origin[1]:8.4f}, "
              f"{origin[2]:8.4f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
