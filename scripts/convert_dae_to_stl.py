#!/usr/bin/env python3
"""Convert a COLLADA (.dae) mesh to a binary STL for FCL collision checking.

The excavator collision checker only loads ASCII/binary STL, while the zx200
visual meshes are DAE. This tool applies the DAE node transforms to every
triangle and writes the result in the link frame, so the generated STL can be
used directly as the URDF collision geometry.

Usage:
  python3 scripts/convert_dae_to_stl.py \
    src/excavator_description/meshes/zx200-boom.dae \
    src/excavator_description/meshes/zx200-boom-collision.stl
"""

import argparse
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def node_transform(node):
    transform = np.eye(4)
    for child in node:
        tag = child.tag.split("}")[-1]
        text = (child.text or "").split()
        if tag == "matrix" and len(text) == 16:
            values = np.array([float(value) for value in text]).reshape(4, 4)
            # COLLADA matrices are column-major.
            transform = transform @ values.T
        elif tag == "translate" and len(text) == 3:
            value = np.eye(4)
            value[:3, 3] = [float(item) for item in text]
            transform = transform @ value
        elif tag == "rotate" and len(text) == 4:
            axis = np.array([float(item) for item in text[:3]])
            norm = np.linalg.norm(axis)
            if norm == 0.0:
                continue
            axis /= norm
            angle = math.radians(float(text[3]))
            cross = np.array([[0.0, -axis[2], axis[1]],
                              [axis[2], 0.0, -axis[0]],
                              [-axis[1], axis[0], 0.0]])
            rotation = np.eye(4)
            rotation[:3, :3] = (np.eye(3) + math.sin(angle) * cross +
                                (1.0 - math.cos(angle)) * (cross @ cross))
            transform = transform @ rotation
        elif tag == "scale" and len(text) == 3:
            value = np.eye(4)
            value[0, 0], value[1, 1], value[2, 2] = [float(item)
                                                     for item in text]
            transform = transform @ value
    return transform


def geometry_triangles(root):
    geometries = {}
    for geometry in root.findall(".//c:library_geometries/c:geometry", NS):
        geometry_id = geometry.get("id")
        vertices = None
        triangles = []
        for mesh in geometry.findall("c:mesh", NS):
            position_source = None
            for vertex_source in mesh.findall("c:vertices", NS):
                for item in vertex_source.findall("c:input", NS):
                    if item.get("semantic") == "POSITION":
                        position_source = item.get("source", "").lstrip("#")
            if position_source is None:
                continue
            for source in mesh.findall("c:source", NS):
                if source.get("id") != position_source:
                    continue
                array = source.find("c:float_array", NS)
                if array is None:
                    continue
                values = np.array([float(value)
                                   for value in (array.text or "").split()])
                accessor = source.find("c:technique_common/c:accessor", NS)
                stride = int(accessor.get("stride", "3")) \
                    if accessor is not None else 3
                if stride < 3:
                    continue
                vertices = values.reshape(-1, stride)[:, :3]
            if vertices is None:
                continue
            for primitive in mesh.findall("c:triangles", NS):
                indices = primitive.find("c:p", NS)
                if indices is None:
                    continue
                values = np.array([int(value)
                                   for value in (indices.text or "").split()])
                offset = 0
                max_offset = 0
                for item in primitive.findall("c:input", NS):
                    item_offset = int(item.get("offset", "0"))
                    max_offset = max(max_offset, item_offset)
                    if item.get("semantic") == "VERTEX":
                        offset = item_offset
                step = max_offset + 1
                if step <= 0 or len(values) % (3 * step) != 0:
                    continue
                indices = values.reshape(-1, step)[:, offset]
                triangles.append(indices.reshape(-1, 3))
        if vertices is not None and triangles:
            geometries[geometry_id] = (vertices, np.vstack(triangles))
    return geometries


def world_triangles(root, geometries):
    collected = []

    def visit(node, transform):
        transform = transform @ node_transform(node)
        for instance in node.findall("c:instance_geometry", NS):
            geometry_id = instance.get("url", "").lstrip("#")
            if geometry_id not in geometries:
                continue
            vertices, triangles = geometries[geometry_id]
            world = (np.c_[vertices, np.ones(len(vertices))] @ transform.T)[:, :3]
            collected.append(world[triangles])
        for child in node.findall("c:node", NS):
            visit(child, transform)

    scenes = root.findall(".//c:library_visual_scenes/c:visual_scene", NS)
    if not scenes:
        raise RuntimeError("DAE has no visual scene")
    for node in scenes[0].findall("c:node", NS):
        visit(node, np.eye(4))
    if not collected:
        raise RuntimeError("DAE contains no triangle geometry")
    return np.vstack(collected)


def write_binary_stl(path, triangles):
    # Remove degenerate triangles.
    edge1 = triangles[:, 1] - triangles[:, 0]
    edge2 = triangles[:, 2] - triangles[:, 0]
    normals = np.cross(edge1, edge2)
    lengths = np.linalg.norm(normals, axis=1)
    valid = lengths > 1.0e-12
    triangles = triangles[valid]
    normals = normals[valid] / lengths[valid, None]
    if len(triangles) == 0:
        raise RuntimeError("no non-degenerate triangles to write")
    with open(path, "wb") as stream:
        header = b"Binary STL generated from DAE for FCL collision"
        stream.write(header.ljust(80, b" "))
        stream.write(struct.pack("<I", len(triangles)))
        for normal, triangle in zip(normals, triangles):
            stream.write(struct.pack("<12fH", *normal, *triangle.reshape(-1), 0))
    return len(triangles)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="input .dae file")
    parser.add_argument("output", help="output binary .stl file")
    args = parser.parse_args()
    try:
        root = ET.parse(args.input).getroot()
        triangles = world_triangles(root, geometry_triangles(root))
        os.makedirs(os.path.dirname(os.path.abspath(args.output)),
                    exist_ok=True)
        count = write_binary_stl(args.output, triangles)
    except (OSError, RuntimeError, ET.ParseError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print(f"wrote {args.output}: {count} triangles")
    return 0


if __name__ == "__main__":
    sys.exit(main())
