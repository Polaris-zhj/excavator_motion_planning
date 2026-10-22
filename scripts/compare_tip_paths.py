#!/usr/bin/env python3
"""Compare end-effector paths of two robot models for the same joint CSV.

The CSV stores joint positions over time, not Cartesian positions. The same
joint sequence can produce very different tip paths when the two URDFs differ
in joint names, zero/mounting angles, axes, link offsets or tip frames. This
script expands both models, applies the same joint samples, computes forward
kinematics and writes a side-by-side comparison figure.

Usage:
  source /opt/ros/humble/setup.bash
  /usr/bin/python3 scripts/compare_tip_paths.py \
    --csv src/excavator_visualization/sample_trajectory.csv \
    --model-a src/excavator_description/urdf/excavator.urdf.xacro \
    --model-b src/excavator_description/urdf/zx200.xacro \
    --output /tmp/tip_path_comparison.png

If PyQt5/QtSvg is available, a PNG is also rendered. Otherwise the SVG is
kept and the PNG conversion is skipped.
"""

import argparse
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_DEFAULT_CSV_CANDIDATES = [
    os.path.join(REPO_ROOT, "src", "excavator_visualization",
                 "sample_trajectory.csv"),
    os.path.join(REPO_ROOT, "src", "excavator_visualization", "config",
                 "sample_trajectory.csv"),
]
DEFAULT_CSV = next(
    (path for path in _DEFAULT_CSV_CANDIDATES if os.path.isfile(path)),
    _DEFAULT_CSV_CANDIDATES[-1])
DEFAULT_MODEL_A = os.path.join(
    REPO_ROOT, "src", "excavator_description", "urdf",
    "excavator.urdf.xacro")
DEFAULT_MODEL_B = os.path.join(
    REPO_ROOT, "src", "excavator_description", "urdf", "zx200.xacro")

# The same physical joint may have different names in different models.
JOINT_ALIASES = {
    "stick_joint": ["arm_joint"],
    "arm_joint": ["stick_joint"],
}

COLOR_A = "#1f77b4"
COLOR_B = "#e07b00"
GRID_COLOR = "#d9dde3"
AXIS_COLOR = "#7a828c"
TEXT_COLOR = "#25303b"


def resolve_path(path):
    if os.path.isabs(path):
        return path
    candidate = os.path.join(REPO_ROOT, path)
    return candidate if os.path.exists(candidate) else os.path.abspath(path)


def expand_urdf(path):
    """Return URDF XML text, expanding .xacro files with the xacro tool."""
    path = resolve_path(path)
    if not os.path.isfile(path):
        raise FileNotFoundError(path)
    if path.endswith((".xacro", ".xacro.urdf")):
        try:
            result = subprocess.run(
                ["xacro", path], check=True, capture_output=True, text=True)
        except FileNotFoundError as exc:
            raise RuntimeError(
                "xacro executable not found; source /opt/ros/humble/setup.bash"
            ) from exc
        except subprocess.CalledProcessError as exc:
            raise RuntimeError(
                f"xacro failed for {path}:\n{exc.stderr}") from exc
        return result.stdout
    with open(path, encoding="utf-8") as stream:
        return stream.read()


def parse_model(path):
    xml_text = expand_urdf(path)
    try:
        root = ET.fromstring(xml_text)
    except ET.ParseError as exc:
        raise RuntimeError(f"cannot parse {path}: {exc}") from exc
    links = {link.get("name"): link for link in root.findall("link")}
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    child_to_joint = {}
    for joint in root.findall("joint"):
        child = joint.find("child")
        if child is not None:
            child_to_joint[child.get("link")] = joint
    return {
        "path": path,
        "links": links,
        "joints": joints,
        "child_to_joint": child_to_joint,
    }


def choose_tip(model, preferred):
    if preferred and preferred in model["links"]:
        return preferred
    for candidate in ("bucket_tip", "bucket_end_link", "bucket_link"):
        if candidate in model["links"]:
            return candidate
    raise RuntimeError(
        f"no tip link found in {model['path']}; available links: "
        + ", ".join(sorted(model["links"])))


def kinematic_chain(model, tip):
    chain = []
    link = tip
    if link not in model["links"]:
        raise RuntimeError(
            f"tip link '{tip}' is not in {model['path']}; available links: "
            + ", ".join(sorted(model["links"])))
    while link in model["child_to_joint"]:
        joint = model["child_to_joint"][link]
        chain.append(joint)
        parent = joint.find("parent")
        if parent is None:
            break
        link = parent.get("link")
    chain.reverse()
    return chain, link


def read_csv(path):
    path = resolve_path(path)
    with open(path, encoding="utf-8-sig") as stream:
        lines = [line.strip() for line in stream if line.strip()]
    if not lines:
        raise RuntimeError(f"empty trajectory CSV: {path}")
    header = [item.strip() for item in lines[0].split(",")]
    if not header or header[0] != "joint_names":
        raise RuntimeError(
            "CSV header must start with 'joint_names', got: " + lines[0])
    joint_names = [item for item in header[1:] if item]
    if not joint_names:
        raise RuntimeError("CSV header contains no joint names")
    samples = []
    for line in lines[1:]:
        fields = [item.strip() for item in line.split(",")]
        if not fields or not fields[0]:
            continue
        try:
            stamp = float(fields[0])
            positions = [float(item) for item in fields[1:1 + len(joint_names)]]
        except ValueError as exc:
            raise RuntimeError(f"invalid CSV row: {line}") from exc
        if len(positions) != len(joint_names):
            raise RuntimeError(
                f"CSV row has {len(positions)} positions, expected "
                f"{len(joint_names)}: {line}")
        samples.append((stamp, positions))
    if not samples:
        raise RuntimeError(f"no samples in {path}")
    return joint_names, samples


def parse_map(entries, model, label):
    """Parse csv_joint=urdf_joint overrides and resolve automatic aliases."""
    explicit = {}
    for entry in entries or []:
        if "=" not in entry:
            raise RuntimeError(f"{label} map must be csv_joint=urdf_joint")
        source, target = entry.split("=", 1)
        source, target = source.strip(), target.strip()
        if not source or not target:
            raise RuntimeError(f"{label} map must be csv_joint=urdf_joint")
        explicit[source] = target
    return explicit


def map_joint_names(csv_names, model, explicit, label):
    """Return {urdf_joint: csv_index_or_None} plus diagnostics."""
    mapping = {}
    notes = []
    for index, csv_name in enumerate(csv_names):
        target = None
        if csv_name in explicit:
            target = explicit[csv_name]
        elif csv_name in model["joints"]:
            target = csv_name
        else:
            for alias in JOINT_ALIASES.get(csv_name, []):
                if alias in model["joints"]:
                    target = alias
                    break
        if target is None:
            notes.append(
                f"{label}: joint '{csv_name}' is not in the URDF; "
                "its value is treated as 0")
            continue
        if target in mapping:
            notes.append(
                f"{label}: CSV joints '{csv_name}' and "
                f"'{csv_names[mapping[target]]}' both map to '{target}'")
        mapping[target] = index
        if target != csv_name:
            notes.append(f"{label}: '{csv_name}' -> '{target}'")
    return mapping, notes


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0.0],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0.0],
        [-sp, cp * sr, cp * cr, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def translation_matrix(x, y, z):
    return [
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def axis_rotation(axis, angle):
    ax, ay, az = axis
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm == 0.0:
        return [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    ax, ay, az = ax / norm, ay / norm, az / norm
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return [
        [t * ax * ax + c, t * ax * ay - s * az, t * ax * az + s * ay, 0.0],
        [t * ax * ay + s * az, t * ay * ay + c, t * ay * az - s * ax, 0.0],
        [t * ax * az - s * ay, t * ay * az + s * ax, t * az * az + c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def axis_translation(axis, distance):
    ax, ay, az = axis
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm == 0.0:
        return translation_matrix(0.0, 0.0, 0.0)
    return translation_matrix(ax / norm * distance, ay / norm * distance,
                              az / norm * distance)


def mat_mul(left, right):
    return [[sum(left[row][k] * right[k][col] for k in range(4))
             for col in range(4)] for row in range(4)]


def origin_transform(joint):
    origin = joint.find("origin")
    if origin is None:
        return translation_matrix(0.0, 0.0, 0.0)
    xyz = [float(value) for value in origin.get("xyz", "0 0 0").split()]
    rpy = [float(value) for value in origin.get("rpy", "0 0 0").split()]
    return mat_mul(translation_matrix(*xyz), rpy_matrix(*rpy))


def joint_axis(joint):
    axis = joint.find("axis")
    if axis is None:
        return (1.0, 0.0, 0.0)
    return tuple(float(value) for value in axis.get("xyz", "1 0 0").split())


def compute_path(chain, values_by_joint, samples, csv_names):
    points = []
    for _, positions in samples:
        transform = translation_matrix(0.0, 0.0, 0.0)
        for joint in chain:
            transform = mat_mul(transform, origin_transform(joint))
            joint_name = joint.get("name")
            joint_type = joint.get("type", "fixed")
            if joint_type == "fixed":
                continue
            index = values_by_joint.get(joint_name)
            value = positions[index] if index is not None else 0.0
            axis = joint_axis(joint)
            if joint_type in ("revolute", "continuous"):
                transform = mat_mul(transform, axis_rotation(axis, value))
            elif joint_type == "prismatic":
                transform = mat_mul(transform, axis_translation(axis, value))
        points.append((transform[0][3], transform[1][3], transform[2][3]))
    return points


def ranges(points):
    return [(min(point[axis] for point in points),
             max(point[axis] for point in points)) for axis in range(3)]


def path_length(points):
    return sum(math.dist(points[index - 1], points[index])
               for index in range(1, len(points)))


def aligned(points):
    x0, y0, z0 = points[0]
    return [(x - x0, y - y0, z - z0) for x, y, z in points]


def panel_geometry(bounds, left, top, width, height, equal_aspect=True):
    x_lo, x_hi, y_lo, y_hi = bounds
    # A trajectory that is almost planar (for example swing = 0) has a very
    # small y range. Keep a readable minimum span instead of squeezing all
    # tick labels into a few pixels.
    min_span = 0.4
    if x_hi - x_lo < min_span:
        center = 0.5 * (x_lo + x_hi)
        x_lo, x_hi = center - 0.5 * min_span, center + 0.5 * min_span
    if y_hi - y_lo < min_span:
        center = 0.5 * (y_lo + y_hi)
        y_lo, y_hi = center - 0.5 * min_span, center + 0.5 * min_span
    pad_x = 0.08 * (x_hi - x_lo)
    pad_y = 0.08 * (y_hi - y_lo)
    x_lo, x_hi = x_lo - pad_x, x_hi + pad_x
    y_lo, y_hi = y_lo - pad_y, y_hi + pad_y
    data_w, data_h = x_hi - x_lo, y_hi - y_lo
    if equal_aspect:
        scale = min(width / data_w, height / data_h)
        used_w, used_h = data_w * scale, data_h * scale
        x_left = left + (width - used_w) * 0.5
        y_top = top + (height - used_h) * 0.5
        scale_x = scale_y = scale
    else:
        # Top views can have a tiny y range (e.g. swing = 0). Stretch both
        # axes independently so the tick labels remain readable.
        scale_x = width / data_w
        scale_y = height / data_h
        x_left = left
        y_top = top

    def project(x, y):
        screen_x = x_left + (x - x_lo) * scale_x
        screen_y = y_top + height - (y - y_lo) * scale_y
        return screen_x, screen_y

    return project, (x_lo, x_hi, y_lo, y_hi), min(scale_x, scale_y)


def svg_panel(lines, title, bounds, series, left, top, width, height,
              x_label, y_label, show_legend=False, equal_aspect=True):
    project, limits, scale = panel_geometry(
        bounds, left, top, width, height, equal_aspect)
    x_lo, x_hi, y_lo, y_hi = limits

    def tx(value):
        return project(value, y_lo)[0]

    def ty(value):
        return project(x_lo, value)[1]

    lines.append(
        f'<rect x="{left}" y="{top}" width="{width}" height="{height}" '
        f'fill="#ffffff" stroke="#c9cfd6" stroke-width="1"/>')
    values = [x_lo + (x_hi - x_lo) * index / 4.0 for index in range(5)]
    for value in values:
        x = tx(value)
        lines.append(
            f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" '
            f'y2="{top + height}" stroke="{GRID_COLOR}" stroke-width="1"/>')
        lines.append(
            f'<text x="{x:.2f}" y="{top + height + 16}" '
            f'font-size="11" fill="{TEXT_COLOR}" text-anchor="middle">'
            f'{value:.2f}</text>')
    values = [y_lo + (y_hi - y_lo) * index / 4.0 for index in range(5)]
    for value in values:
        y = ty(value)
        lines.append(
            f'<line x1="{left}" y1="{y:.2f}" x2="{left + width}" '
            f'y2="{y:.2f}" stroke="{GRID_COLOR}" stroke-width="1"/>')
        lines.append(
            f'<text x="{left - 8}" y="{y + 4:.2f}" font-size="11" '
            f'fill="{TEXT_COLOR}" text-anchor="end">{value:.2f}</text>')
    if x_lo <= 0.0 <= x_hi:
        x = tx(0.0)
        lines.append(
            f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" '
            f'y2="{top + height}" stroke="{AXIS_COLOR}" stroke-width="1.2"/>')
    if y_lo <= 0.0 <= y_hi:
        y = ty(0.0)
        lines.append(
            f'<line x1="{left}" y1="{y:.2f}" x2="{left + width}" '
            f'y2="{y:.2f}" stroke="{AXIS_COLOR}" stroke-width="1.2"/>')
    lines.append(
        f'<text x="{left + 10}" y="{top + 20}" font-size="14" '
        f'font-weight="600" fill="{TEXT_COLOR}">{escape(title)}</text>')
    lines.append(
        f'<text x="{left + width - 8}" y="{top + height - 10}" '
        f'font-size="11" fill="{AXIS_COLOR}" text-anchor="end">'
        f'{escape(x_label)} / {escape(y_label)}</text>')

    for name, color, points in series:
        if len(points) < 2:
            continue
        coords = " ".join(
            f"{project(point[0], point[1])[0]:.2f},"
            f"{project(point[0], point[1])[1]:.2f}" for point in points)
        lines.append(
            f'<polyline points="{coords}" fill="none" stroke="{color}" '
            f'stroke-width="2.4" stroke-linejoin="round" '
            f'stroke-linecap="round"/>')
        sx, sy = project(points[0][0], points[0][1])
        ex, ey = project(points[-1][0], points[-1][1])
        lines.append(
            f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="5" fill="{color}"/>')
        lines.append(
            f'<rect x="{ex - 5:.2f}" y="{ey - 5:.2f}" width="10" height="10" '
            f'fill="{color}" stroke="#ffffff" stroke-width="1"/>')

    if show_legend:
        legend_y = top + 42
        for name, color, _ in series:
            lines.append(
                f'<line x1="{left + 12}" y1="{legend_y}" '
                f'x2="{left + 36}" y2="{legend_y}" stroke="{color}" '
                f'stroke-width="3"/>')
            lines.append(
                f'<text x="{left + 44}" y="{legend_y + 4}" font-size="12" '
                f'fill="{TEXT_COLOR}">{escape(name)}</text>')
            legend_y += 20


def project_series(series, x_index, y_index):
    return [(name, color, [(point[x_index], point[y_index]) for point in points])
            for name, color, points in series]


def write_svg(path, model_a, model_b, tip_a, tip_b, csv_path, samples,
              series_abs, series_aligned, notes, separation):
    width, height = 1500, 1020
    panels = [
        ("Side view (absolute x-z)",
         project_series(series_abs, 0, 2), "x", "z", True),
        ("Side view (start-aligned x-z)",
         project_series(series_aligned, 0, 2), "dx", "dz", True),
        ("Top view (absolute x-y)",
         project_series(series_abs, 0, 1), "x", "y", False),
        ("Top view (start-aligned x-y)",
         project_series(series_aligned, 0, 1), "dx", "dy", False),
    ]
    all_bounds = []
    for _, series, _, _, _ in panels:
        xs = [point[0] for _, _, points in series for point in points]
        ys = [point[1] for _, _, points in series for point in points]
        all_bounds.append((min(xs), max(xs), min(ys), max(ys)))

    margin = 70
    gap = 55
    panel_w = (width - 2 * margin - gap) / 2.0
    panel_h = 330
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
        f'height="{height}" viewBox="0 0 {width} {height}">',
        f'<rect width="{width}" height="{height}" fill="#f4f6f8"/>',
        f'<text x="{margin}" y="36" font-size="22" font-weight="700" '
        f'fill="{TEXT_COLOR}">End-effector path comparison</text>',
        f'<text x="{margin}" y="60" font-size="13" fill="{TEXT_COLOR}">'
        f'A: {escape(os.path.basename(model_a))} (tip={escape(tip_a)})   '
        f'B: {escape(os.path.basename(model_b))} (tip={escape(tip_b)})   '
        f'CSV: {escape(os.path.basename(csv_path))} ({len(samples)} samples)'
        f'</text>',
    ]
    series_names = [("A: " + os.path.basename(model_a), COLOR_A),
                    ("B: " + os.path.basename(model_b), COLOR_B)]
    for index, (title, series, x_label, y_label, equal_aspect) in enumerate(
            panels):
        row, col = divmod(index, 2)
        left = margin + col * (panel_w + gap)
        top = 92 + row * (panel_h + gap)
        svg_panel(lines, title, all_bounds[index], series,
                  left, top, panel_w, panel_h, x_label, y_label,
                  show_legend=(index == 0), equal_aspect=equal_aspect)
    lines.append(
        f'<text x="{margin}" y="{height - 48}" font-size="12" '
        f'fill="{TEXT_COLOR}">start-aligned max sample separation: '
        f'{separation:.3f} m</text>')
    note_y = height - 30
    for note in notes[:4]:
        lines.append(
            f'<text x="{margin}" y="{note_y}" font-size="11" '
            f'fill="#b03a2e">{escape(note)}</text>')
        note_y += 14
    lines.append("</svg>")
    with open(path, "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines))


def write_png(svg_path, png_path, width=1500, height=1020):
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    try:
        from PyQt5.QtGui import QColor, QImage, QPainter
        from PyQt5.QtSvg import QSvgRenderer
        from PyQt5.QtWidgets import QApplication
    except ImportError as exc:
        print(f"PNG skipped (PyQt5/QtSvg unavailable: {exc}); SVG kept")
        return False
    app = QApplication.instance() or QApplication([])
    renderer = QSvgRenderer(svg_path)
    if not renderer.isValid():
        print("PNG skipped (invalid SVG renderer)")
        return False
    image = QImage(width, height, QImage.Format_ARGB32)
    image.fill(QColor("white"))
    painter = QPainter(image)
    renderer.render(painter)
    painter.end()
    if not image.save(png_path):
        print(f"PNG skipped (could not save {png_path})")
        return False
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Compare end-effector paths of two models for one CSV")
    parser.add_argument("--csv", default=DEFAULT_CSV,
                        help="joint trajectory CSV (default: %(default)s)")
    parser.add_argument("--model-a", default=DEFAULT_MODEL_A,
                        help="first URDF/xacro (default: %(default)s)")
    parser.add_argument("--model-b", default=DEFAULT_MODEL_B,
                        help="second URDF/xacro (default: %(default)s)")
    parser.add_argument("--tip-a", default=None,
                        help="tip link for model A (auto-detected)")
    parser.add_argument("--tip-b", default=None,
                        help="tip link for model B (auto-detected)")
    parser.add_argument("--map-a", action="append", metavar="CSV=URDF",
                        help="joint name override for model A (repeatable)")
    parser.add_argument("--map-b", action="append", metavar="CSV=URDF",
                        help="joint name override for model B (repeatable)")
    parser.add_argument("--output", default="tip_path_comparison.png",
                        help="output .png or .svg (default: %(default)s)")
    args = parser.parse_args()

    try:
        csv_names, samples = read_csv(args.csv)
        model_a = parse_model(args.model_a)
        model_b = parse_model(args.model_b)
        tip_a = choose_tip(model_a, args.tip_a)
        tip_b = choose_tip(model_b, args.tip_b)
        chain_a, root_a = kinematic_chain(model_a, tip_a)
        chain_b, root_b = kinematic_chain(model_b, tip_b)
        map_a, notes_a = map_joint_names(
            csv_names, model_a, parse_map(args.map_a, model_a, "model-a"),
            "model-a")
        map_b, notes_b = map_joint_names(
            csv_names, model_b, parse_map(args.map_b, model_b, "model-b"),
            "model-b")
        path_a = compute_path(chain_a, map_a, samples, csv_names)
        path_b = compute_path(chain_b, map_b, samples, csv_names)
    except (RuntimeError, FileNotFoundError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    aligned_a = aligned(path_a)
    aligned_b = aligned(path_b)
    separation = max(
        math.dist(first, second)
        for first, second in zip(aligned_a, aligned_b))
    notes = notes_a + notes_b
    notes.insert(0, f"model A root={root_a}, tip={tip_a}, "
                    f"chain={' -> '.join(j.get('name') for j in chain_a)}")
    notes.insert(1, f"model B root={root_b}, tip={tip_b}, "
                    f"chain={' -> '.join(j.get('name') for j in chain_b)}")

    range_a, range_b = ranges(path_a), ranges(path_b)
    print(f"CSV: {os.path.abspath(args.csv)}")
    print(f"samples: {len(samples)}")
    print(f"model A: {os.path.abspath(args.model_a)}")
    print(f"  tip={tip_a} chain="
          f"{' -> '.join(j.get('name') for j in chain_a)}")
    print("  ranges (x,y,z) m: " + ", ".join(
        f"[{lo:.3f}, {hi:.3f}]" for lo, hi in range_a))
    print(f"  path length: {path_length(path_a):.3f} m")
    print(f"model B: {os.path.abspath(args.model_b)}")
    print(f"  tip={tip_b} chain="
          f"{' -> '.join(j.get('name') for j in chain_b)}")
    print("  ranges (x,y,z) m: " + ", ".join(
        f"[{lo:.3f}, {hi:.3f}]" for lo, hi in range_b))
    print(f"  path length: {path_length(path_b):.3f} m")
    print(f"start-aligned max sample separation: {separation:.3f} m")
    for note in notes:
        print("  " + note)

    output = os.path.abspath(args.output)
    stem, ext = os.path.splitext(output)
    ext = ext.lower()
    svg_path = output if ext == ".svg" else stem + ".svg"
    write_svg(svg_path, os.path.abspath(args.model_a),
              os.path.abspath(args.model_b), tip_a, tip_b,
              os.path.abspath(args.csv), samples,
              [("A: " + os.path.basename(args.model_a), COLOR_A, path_a),
               ("B: " + os.path.basename(args.model_b), COLOR_B, path_b)],
              [("A: " + os.path.basename(args.model_a), COLOR_A, aligned_a),
               ("B: " + os.path.basename(args.model_b), COLOR_B, aligned_b)],
              notes, separation)
    print(f"wrote {svg_path}")
    if ext == ".png":
        if write_png(svg_path, output):
            print(f"wrote {output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
