#!/usr/bin/env python3
"""Create one open-pose RH56 preview mesh from the supplied MuJoCo XML."""

from __future__ import annotations

import math
import struct
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Transform:
    rotation: tuple[tuple[float, float, float], ...]
    translation: tuple[float, float, float]


IDENTITY = Transform(
    rotation=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    translation=(0.0, 0.0, 0.0),
)


def parse_floats(value: str | None, count: int, default: tuple[float, ...]) -> tuple[float, ...]:
    if value is None:
        return default
    values = tuple(float(part) for part in value.split())
    if len(values) != count:
        raise ValueError(f"Expected {count} values, got {value!r}")
    return values


def quaternion_to_rotation(quaternion: tuple[float, float, float, float]) -> tuple[tuple[float, float, float], ...]:
    w, x, y, z = quaternion
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = (component / norm for component in quaternion)
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def multiply(left: Transform, right: Transform) -> Transform:
    rotation = tuple(
        tuple(sum(left.rotation[row][index] * right.rotation[index][column] for index in range(3)) for column in range(3))
        for row in range(3)
    )
    translation = tuple(
        left.translation[row] + sum(left.rotation[row][index] * right.translation[index] for index in range(3))
        for row in range(3)
    )
    return Transform(rotation=rotation, translation=translation)


def transform_point(transform: Transform, point: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(
        transform.translation[row] + sum(transform.rotation[row][column] * point[column] for column in range(3))
        for row in range(3)
    )


def read_binary_stl(path: Path) -> list[tuple[tuple[float, float, float], ...]]:
    data = path.read_bytes()
    triangles = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + triangles * 50:
        raise ValueError(f"{path} is not a binary STL")
    return [
        tuple(
            tuple(struct.unpack_from("<3f", data, 84 + triangle * 50 + 12 + vertex * 12))
            for vertex in range(3)
        )
        for triangle in range(triangles)
    ]


def write_binary_stl(path: Path, triangles: list[tuple[tuple[float, float, float], ...]]) -> None:
    with path.open("wb") as stream:
        stream.write(b"RH56 open-pose RViz preview".ljust(80, b"\0"))
        stream.write(struct.pack("<I", len(triangles)))
        for triangle in triangles:
            stream.write(struct.pack("<3f", 0.0, 0.0, 0.0))
            for vertex in triangle:
                stream.write(struct.pack("<3f", *vertex))
            stream.write(struct.pack("<H", 0))


def append_body_meshes(body: ET.Element, parent: Transform, mesh_files: dict[str, Path], triangles: list[tuple[tuple[float, float, float], ...]]) -> None:
    body_pose = Transform(
        rotation=quaternion_to_rotation(parse_floats(body.get("quat"), 4, (1.0, 0.0, 0.0, 0.0))),
        translation=parse_floats(body.get("pos"), 3, (0.0, 0.0, 0.0)),
    )
    world_pose = multiply(parent, body_pose)
    for geom in body.findall("geom"):
        if geom.get("class") != "visual":
            continue
        mesh_name = geom.get("mesh")
        if mesh_name is None or mesh_name not in mesh_files:
            continue
        geom_pose = Transform(
            rotation=quaternion_to_rotation(parse_floats(geom.get("quat"), 4, (1.0, 0.0, 0.0, 0.0))),
            translation=parse_floats(geom.get("pos"), 3, (0.0, 0.0, 0.0)),
        )
        mesh_pose = multiply(world_pose, geom_pose)
        triangles.extend(
            tuple(transform_point(mesh_pose, vertex) for vertex in triangle)
            for triangle in read_binary_stl(mesh_files[mesh_name])
        )
    for child in body.findall("body"):
        append_body_meshes(child, world_pose, mesh_files, triangles)


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit("Usage: build_rh56_open_preview.py INSPIRE_RIGHT_XML OUTPUT_STL")
    xml_path = Path(sys.argv[1]).resolve()
    output_path = Path(sys.argv[2]).resolve()
    root = ET.parse(xml_path).getroot()
    mesh_directory = xml_path.parent / root.find("compiler").get("meshdir")
    mesh_files = {
        mesh.get("name"): mesh_directory / mesh.get("file")
        for mesh in root.find("asset").findall("mesh")
    }
    triangles: list[tuple[tuple[float, float, float], ...]] = []
    for body in root.find("worldbody").findall("body"):
        append_body_meshes(body, IDENTITY, mesh_files, triangles)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    write_binary_stl(output_path, triangles)
    print(f"Wrote {len(triangles)} triangles to {output_path}")


if __name__ == "__main__":
    main()
