"""Convert bundled arena YAML and SDF collision geometry to MJCF."""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from xml.sax.saxutils import escape

import yaml

from .resources import package_share_directory


RESOURCE_DIR = package_share_directory() / "assets"


def numbers(text: str | None, count: int, default: tuple[float, ...]) -> tuple[float, ...]:
    if not text:
        return default
    values = tuple(float(value) for value in text.split())
    return values if len(values) == count else default


def pose(element) -> tuple[float, ...]:
    return numbers(element.findtext("pose") if element is not None else None, 6,
                   (0, 0, 0, 0, 0, 0))


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, ...]:
    """Match the RPY convention used by Gazebo's arena spawners (Rz * Ry * Rx)."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def pose_attrs(values, angles_are_degrees: bool = False) -> str:
    x, y, z, roll, pitch, yaw = values
    if angles_are_degrees:
        roll, pitch, yaw = map(math.radians, (roll, pitch, yaw))
    quat = quaternion_from_euler(roll, pitch, yaw)
    return (f'pos="{x:.9g} {y:.9g} {z:.9g}" '
            f'quat="{" ".join(f"{value:.12g}" for value in quat)}"')


def resource_roots() -> tuple[Path, Path]:
    """Return the arena and model roots bundled with this package."""
    arena_root = RESOURCE_DIR / "arenas"
    model_root = RESOURCE_DIR / "models"
    if not arena_root.is_dir() or not model_root.is_dir():
        raise FileNotFoundError(f"bundled arena assets are missing: {RESOURCE_DIR}")
    return arena_root, model_root


def resolve_arena_path(value: str | Path, arena_root: Path | None = None) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        if arena_root is None:
            arena_root, _ = resource_roots()
        path = arena_root / path
    return path.resolve()


def mesh_path(uri: str, model_dir: Path, model_root: Path) -> Path:
    if uri.startswith("model://"):
        relative = uri[len("model://"):]
        return (model_root / relative).resolve()
    if uri.startswith("file://"):
        candidate = Path(uri[len("file://"):])
        return candidate.resolve() if candidate.is_absolute() else (model_dir / candidate).resolve()
    return (model_dir / uri).resolve()


def convex_parts(path: Path) -> list[Path]:
    """Return bundled convex parts, or the original mesh when it is convex."""
    parts = sorted((path.parent / "collision_parts").glob("part_*.obj"))
    return parts or [path]


def geometry_xml(geometry, name: str, model_dir: Path, yaml_scale,
                 model_root: Path, assets: list[str], warnings: list[str],
                 friction) -> str:
    friction_text = " ".join(f"{float(value):.9g}" for value in friction)
    common = (f'contype="1" conaffinity="2" friction="{friction_text}" '
              'solref="0.005 1" solimp="0.98 0.999 0.001" margin="0.001" '
              'rgba="0.48 0.28 0.13 1"')
    box = geometry.find("box")
    if box is not None:
        size = numbers(box.findtext("size"), 3, (1, 1, 1))
        scaled = tuple(size[i] * yaml_scale[i] / 2 for i in range(3))
        return f'<geom name="{name}" type="box" size="{" ".join(f"{v:.9g}" for v in scaled)}" {common}/>'
    sphere = geometry.find("sphere")
    if sphere is not None:
        radius = float(sphere.findtext("radius", "0.5")) * max(yaml_scale)
        return f'<geom name="{name}" type="sphere" size="{radius:.9g}" {common}/>'
    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        radius = float(cylinder.findtext("radius", "0.5")) * max(yaml_scale[0:2])
        half_length = float(cylinder.findtext("length", "1")) * yaml_scale[2] / 2
        return f'<geom name="{name}" type="cylinder" size="{radius:.9g} {half_length:.9g}" {common}/>'
    mesh = geometry.find("mesh")
    if mesh is not None:
        path = mesh_path(mesh.findtext("uri", ""), model_dir, model_root)
        if not path.exists():
            warnings.append(f"{name}: mesh not found: {path}")
            return ""
        if path.suffix.lower() not in {".stl", ".obj", ".msh"}:
            warnings.append(f"{name}: unsupported MuJoCo mesh format: {path.suffix} ({path})")
            return ""
        sdf_scale = numbers(mesh.findtext("scale"), 3, (1, 1, 1))
        scale = tuple(sdf_scale[i] * yaml_scale[i] for i in range(3))
        part_paths = convex_parts(path)
        mesh_index = len(assets)
        visual_name = f"arena_mesh_{mesh_index}_visual"
        assets.append(f'<mesh name="{visual_name}" file="{escape(str(path))}" scale="{scale[0]:.9g} {scale[1]:.9g} {scale[2]:.9g}"/>')
        # Render the original STL exactly, but exclude it from contact.  Each
        # generated convex part is invisible and participates in collision.
        geoms = [f'<geom name="{name}_visual" type="mesh" mesh="{visual_name}" '
                 'contype="0" conaffinity="0" rgba="0.48 0.28 0.13 1"/>']
        collision_common = common.replace('rgba="0.48 0.28 0.13 1"',
                                          'rgba="0 0 0 0"')
        for part_index, part_path in enumerate(part_paths):
            part_name = f"arena_mesh_{mesh_index}_part_{part_index}"
            assets.append(f'<mesh name="{part_name}" file="{escape(str(part_path))}" scale="{scale[0]:.9g} {scale[1]:.9g} {scale[2]:.9g}"/>')
            geoms.append(f'<geom name="{name}_part_{part_index}" type="mesh" '
                         f'mesh="{part_name}" {collision_common}/>')
        return "".join(geoms)
    warnings.append(f"{name}: unsupported SDF geometry")
    return ""


def load_arena(value: str | Path, friction=(2.5, 0.05, 0.005)) -> tuple[str, str, list[str]]:
    arena_root, model_root = resource_roots()
    arena_path = resolve_arena_path(value, arena_root)
    if not arena_path.exists():
        raise FileNotFoundError(f"arena YAML not found: {arena_path}")
    with arena_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    objects = config.get("robocup_arena", {}).get("objects", {})
    assets, bodies, warnings = [], [], []
    for key, placement in objects.items():
        match = re.match(r"^(.+)_([0-9]+)$", str(key))
        if not match:
            warnings.append(f"{key}: expected <model>_<number>; skipped")
            continue
        model_name = match.group(1)
        sdf_path = model_root / model_name / "model.sdf"
        if not sdf_path.exists():
            warnings.append(f"{key}: model.sdf not found: {sdf_path}")
            continue
        root = ET.parse(sdf_path).getroot()
        model = root.find("model") if root.tag == "sdf" else root.find(".//model")
        if model is None:
            warnings.append(f"{key}: no <model> in {sdf_path}")
            continue
        yaml_scale = tuple(float(placement.get(f"scale_{axis}", 1.0)) for axis in "xyz")
        object_pose = tuple(float(placement.get(field, 0.0))
                            for field in ("x", "y", "z", "roll", "pitch", "yaw"))
        link_xml = []
        for link_index, link in enumerate(model.findall("link")):
            geoms = []
            for collision_index, collision in enumerate(link.findall("collision")):
                geometry = collision.find("geometry")
                if geometry is None:
                    continue
                geom = geometry_xml(geometry, f"{key}_collision_{collision_index}",
                                    sdf_path.parent, yaml_scale, model_root,
                                    assets, warnings, friction)
                if geom:
                    geoms.append(f'<body {pose_attrs(pose(collision))}>{geom}</body>')
            if geoms:
                link_xml.append(f'<body name="{key}_link_{link_index}" {pose_attrs(pose(link))}>{"".join(geoms)}</body>')
        if link_xml:
            model_xml = f'<body {pose_attrs(pose(model))}>{"".join(link_xml)}</body>'
            bodies.append(f'<body name="arena_{key}" {pose_attrs(object_pose, True)}>{model_xml}</body>')
        else:
            warnings.append(f"{key}: no supported collision geometry")
    return "\n".join(assets), "\n".join(bodies), warnings
