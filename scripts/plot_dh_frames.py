#!/usr/bin/env python3
"""Plot the modified-DH frames from excavator_dh.yaml (no ROS required)."""

import argparse
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

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


def main():
    parser = argparse.ArgumentParser(description="Plot modified-DH frames")
    parser.add_argument(
        "--dh-config",
        default=os.path.join(PROJECT_ROOT, "src", "excavator_description",
                             "config", "excavator_dh.yaml"))
    parser.add_argument("--q", default="0,0,0,0")
    parser.add_argument(
        "--base-xyz", default=None,
        help="override base.xyz, e.g. 0,0,1.7572")
    parser.add_argument(
        "--base-rpy", default=None,
        help="override base.rpy, e.g. 0,3.141592653589793,0")
    parser.add_argument(
        "--output",
        default=os.path.join(PROJECT_ROOT, "docs", "screenshots",
                             "dh_frames.png"))
    args = parser.parse_args()

    with open(args.dh_config, "r", encoding="utf-8") as stream:
        parameters = yaml.safe_load(stream)
    q = [float(value) for value in args.q.split(",")]
    if len(q) != len(parameters["joints"]):
        raise SystemExit("--q must contain one value per DH joint")

    base = parameters.get("base") or {}
    if args.base_xyz is not None:
        base["xyz"] = [float(value) for value in args.base_xyz.split(",")]
    if args.base_rpy is not None:
        base["rpy"] = [float(value) for value in args.base_rpy.split(",")]
    current = transform(
        tuple(base.get("xyz", [0.0, 0.0, 0.0])),
        tuple(base.get("rpy", [0.0, 0.0, 0.0])))
    frames = [("base_link", np.eye(4), "base_link")]
    for index, joint in enumerate(parameters["joints"]):
        sign = float(joint.get("theta_sign", 1.0))
        if index == 0:
            current = current @ rz(sign * q[index])
        else:
            current = (current @ rx(float(joint.get("alpha_prev", 0.0))) @
                       tx(float(joint.get("a_prev", 0.0))) @
                       rz(sign * q[index] +
                          float(joint.get("theta_offset", 0.0))) @
                       tz(float(joint.get("d", 0.0))))
        frames.append((joint["name"], current.copy(), joint["name"]))
    tool = (parameters.get("links") or {}).get("bucket_tip")
    if tool is not None:
        current = current @ transform(
            tuple(tool.get("xyz", [0.0, 0.0, 0.0])),
            tuple(tool.get("rpy", [0.0, 0.0, 0.0])))
        frames.append(("bucket_tip", current.copy(), "bucket_tip"))

    figure = plt.figure(figsize=(14, 9), facecolor="#202226")
    axis = figure.add_subplot(111, projection="3d")
    figure.subplots_adjust(left=0.02, right=0.67, bottom=0.04, top=0.92)
    axis.set_facecolor("#202226")
    axis_length = 0.65
    colors = ("#ff4d4d", "#4dff6a", "#4d7dff")
    previous_origin = None
    all_points = []
    label_offsets = {
        "base_link": (0.0, 0.0, -0.35),
        "swing_joint": (-1.00, -0.75, 0.35),
        "boom_joint": (0.65, 0.65, 0.30),
        "stick_joint": (-0.30, 0.40, 0.30),
        "bucket_joint": (-0.90, 0.45, 0.85),
        "bucket_tip": (0.30, -0.10, -0.40),
    }
    table_rows = []
    for name, matrix, label in frames:
        origin = matrix[:3, 3]
        all_points.append(origin)
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
                        color=color, arrow_length_ratio=0.18,
                        linewidth=2.0)
        axis.scatter([origin[0]], [origin[1]], [origin[2]],
                     color="white", s=28)
        offset = label_offsets.get(name, (0.0, 0.0, 0.2))
        axis.text(origin[0] + offset[0], origin[1] + offset[1],
                  origin[2] + offset[2], name,
                  color="white", fontsize=10)
        table_rows.append(
            f"{name:16s} "
            f"({origin[0]:7.3f}, {origin[1]:7.3f}, {origin[2]:7.3f})")

    points = np.array(all_points)
    lower = points.min(axis=0) - 1.0
    upper = points.max(axis=0) + 1.0
    center = 0.5 * (lower + upper)
    radius = 0.5 * max(upper - lower)
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
    axis.set_title(
        "Modified DH frames (q = " +
        ", ".join(f"{value:g}" for value in q) + ", base rpy = " +
        ", ".join(f"{value:g}" for value in
                  base.get("rpy", [0.0, 0.0, 0.0])) + ")",
        color="white", pad=14)
    table = "frame / joint       origin xyz [m]\n" + "\n".join(table_rows)
    figure.text(0.685, 0.91,
                "DH frame origins (base_link)\n\n" + table +
                "\n\nX: red   Y: green   Z: blue"
                "\n\nswing_joint/boom_joint origins"
                "\ndiffer by 9 mm"
                "\n(common normal); axes are\n"
                "drawn at the same point.",
                color="#dddddd", fontsize=8.2, family="monospace",
                va="top")
    axis.text2D(0.02, 0.96, "X", transform=axis.transAxes,
                color=colors[0], fontsize=13, fontweight="bold")
    axis.text2D(0.055, 0.96, "Y", transform=axis.transAxes,
                color=colors[1], fontsize=13, fontweight="bold")
    axis.text2D(0.09, 0.96, "Z", transform=axis.transAxes,
                color=colors[2], fontsize=13, fontweight="bold")

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    figure.tight_layout()
    figure.savefig(args.output, dpi=140, facecolor=figure.get_facecolor())
    print(f"DH frame view: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
