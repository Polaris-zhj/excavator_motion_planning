#!/usr/bin/env python3
"""Convert an ASCII STL file to binary STL.

RViz's built-in STL loader only accepts binary STL. The simplified
base_link mesh shipped in this repository is ASCII, so it must be converted
before it can be displayed by RViz. Binary STL is also accepted by the FCL
collision loader.
"""

import struct
import sys


def convert(source: str, destination: str) -> int:
    with open(source, "rb") as handle:
        data = handle.read()
    stripped = data.lstrip()
    if not stripped.lower().startswith(b"solid") or b"facet" not in data[:4096]:
        print(f"{source} already looks like binary STL; nothing to do")
        return 0

    tokens = data.split()
    facets = []
    index = 0
    while index < len(tokens):
        if (tokens[index].lower() == b"facet" and index + 1 < len(tokens)
                and tokens[index + 1].lower() == b"normal"):
            normal = tuple(float(tokens[index + 2 + offset]) for offset in range(3))
            index += 5
            while index < len(tokens) and tokens[index].lower() != b"vertex":
                index += 1
            vertices = []
            for _ in range(3):
                if index >= len(tokens) or tokens[index].lower() != b"vertex":
                    raise ValueError(f"malformed ASCII STL near token {index}")
                vertices.append(
                    tuple(float(tokens[index + 1 + offset]) for offset in range(3)))
                index += 4
            facets.append((normal, vertices))
        else:
            index += 1

    if not facets:
        raise ValueError("no facets found in ASCII STL")

    with open(destination, "wb") as handle:
        handle.write(b"binary STL converted from ASCII".ljust(80, b"\0"))
        handle.write(struct.pack("<I", len(facets)))
        for normal, vertices in facets:
            handle.write(struct.pack("<3f", *normal))
            for vertex in vertices:
                handle.write(struct.pack("<3f", *vertex))
            handle.write(struct.pack("<H", 0))
    print(f"converted {len(facets)} facets: {source} -> {destination}")
    return 0


def main() -> int:
    if len(sys.argv) not in (2, 3):
        print(f"usage: {sys.argv[0]} input.stl [output.stl]", file=sys.stderr)
        return 2
    source = sys.argv[1]
    destination = sys.argv[2] if len(sys.argv) == 3 else source
    return convert(source, destination)


if __name__ == "__main__":
    raise SystemExit(main())
