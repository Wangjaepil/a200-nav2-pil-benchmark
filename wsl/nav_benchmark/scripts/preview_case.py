#!/usr/bin/env python3
import argparse
import math
import subprocess
import sys
from pathlib import Path

import yaml


def fmt(v):
    return f"{float(v):.6f}"


def vertical_support(shape, obs):
    """Return model/link z so the rotated geometry just touches z=0."""
    pose = obs.get("pose", {})
    roll = float(pose.get("roll", 0.0))
    pitch = float(pose.get("pitch", 0.0))

    # 3rd row of Rz(yaw) * Ry(pitch) * Rx(roll).
    # yaw does not affect vertical extent.
    r31 = -math.sin(pitch)
    r32 = math.cos(pitch) * math.sin(roll)
    r33 = math.cos(pitch) * math.cos(roll)

    if shape == "box":
        sx, sy, sz = map(float, obs["size"])
        hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
        extent = abs(r31) * hx + abs(r32) * hy + abs(r33) * hz
        return extent

    if shape == "cylinder":
        radius = float(obs["radius"])
        height = float(obs["height"])
        radial_vertical = radius * math.sqrt(r31 * r31 + r32 * r32)
        axial_vertical = (height / 2.0) * abs(r33)
        return radial_vertical + axial_vertical

    if shape == "polygon":
        height = float(obs["height"])
        points = [(float(p[0]), float(p[1])) for p in obs["points"]]
        z_candidates = []
        for x, y in points:
            for z_local in (0.0, height):
                z_candidates.append(r31 * x + r32 * y + r33 * z_local)
        return -min(z_candidates)

    raise ValueError(f"Unsupported shape: {shape}")


def geometry_xml(obs):
    shape = obs["shape"]

    if shape == "box":
        sx, sy, sz = obs["size"]
        return f"""<box><size>{fmt(sx)} {fmt(sy)} {fmt(sz)}</size></box>"""

    if shape == "cylinder":
        return (
            f"<cylinder><radius>{fmt(obs['radius'])}</radius>"
            f"<length>{fmt(obs['height'])}</length></cylinder>"
        )

    if shape == "polygon":
        pts = "\n".join(
            f"              <point>{fmt(p[0])} {fmt(p[1])}</point>"
            for p in obs["points"]
        )
        return f"""<polyline>
              <height>{fmt(obs['height'])}</height>
{pts}
            </polyline>"""

    raise ValueError(f"Unsupported shape: {shape}")


def obstacle_link(obs, idx):
    p = obs["pose"]
    roll = float(p.get("roll", 0.0))
    pitch = float(p.get("pitch", 0.0))
    yaw = float(p.get("yaw", 0.0))
    z = vertical_support(obs["shape"], obs)
    geom = geometry_xml(obs)

    return f"""
      <link name="obstacle_{idx:02d}">
        <pose>{fmt(p['x'])} {fmt(p['y'])} {fmt(z)} {fmt(roll)} {fmt(pitch)} {fmt(yaw)}</pose>

        <collision name="collision">
          <geometry>
            {geom}
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            {geom}
          </geometry>
          <material>
            <ambient>0.55 0.55 0.55 1</ambient>
            <diffuse>0.55 0.55 0.55 1</diffuse>
          </material>
        </visual>
      </link>
"""


def marker_link(name, x, y, color):
    # Tall visual-only pole so Start / Goal are easy to find.
    r, g, b = color
    return f"""
      <link name="{name}">
        <pose>{fmt(x)} {fmt(y)} 1.0 0 0 0</pose>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.45</radius>
              <length>2.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
          </material>
        </visual>
      </link>
"""


def nominal_line_link(start, goal):
    sx, sy = float(start["x"]), float(start["y"])
    gx, gy = float(goal["x"]), float(goal["y"])
    dx, dy = gx - sx, gy - sy
    length = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    mx, my = (sx + gx) / 2.0, (sy + gy) / 2.0

    return f"""
      <link name="nominal_start_goal_line">
        <pose>{fmt(mx)} {fmt(my)} 0.035 0 0 {fmt(yaw)}</pose>
        <visual name="visual">
          <geometry>
            <box>
              <size>{fmt(length)} 0.10 0.03</size>
            </box>
          </geometry>
          <material>
            <ambient>0.95 0.85 0.10 1</ambient>
            <diffuse>0.95 0.85 0.10 1</diffuse>
          </material>
        </visual>
      </link>
"""


def build_preview_model(data):
    start = data["robot"]
    goal = data["goal"]
    case_id = data["case_id"]

    links = []
    # Green = Start, Red = Goal
    links.append(marker_link("start_marker", start["x"], start["y"], (0.10, 0.90, 0.10)))
    links.append(marker_link("goal_marker", goal["x"], goal["y"], (0.95, 0.10, 0.10)))
    links.append(nominal_line_link(start, goal))

    for i, obs in enumerate(data.get("obstacles", []), start=1):
        links.append(obstacle_link(obs, i))

    return f"""
    <!-- AUTO-GENERATED CASE PREVIEW: {case_id} -->
    <model name="preview_{case_id}">
      <static>true</static>
      {''.join(links)}
    </model>
"""


def main():
    parser = argparse.ArgumentParser(
        description="Preview one nav_benchmark YAML case inside the common Gazebo benchmark world."
    )
    parser.add_argument("case_yaml", help="Path to case YAML, e.g. cases/S1/S1_01.yaml")
    parser.add_argument(
        "--world",
        default=str(Path.home() / "nav_benchmark/worlds/benchmark.sdf"),
        help="Base benchmark.sdf path"
    )
    parser.add_argument(
        "--no-launch",
        action="store_true",
        help="Only generate preview SDF; do not start Gazebo"
    )
    args = parser.parse_args()

    case_path = Path(args.case_yaml).expanduser().resolve()
    world_path = Path(args.world).expanduser().resolve()

    if not case_path.exists():
        sys.exit(f"Case file not found: {case_path}")
    if not world_path.exists():
        sys.exit(f"Base world not found: {world_path}")

    with case_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    world_text = world_path.read_text(encoding="utf-8")
    if "</world>" not in world_text:
        sys.exit("Invalid benchmark world: </world> not found")

    preview_model = build_preview_model(data)
    preview_world = world_text.replace("</world>", preview_model + "\n  </world>", 1)

    out_dir = Path.home() / "nav_benchmark/preview"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{data['case_id']}_preview.sdf"
    out_path.write_text(preview_world, encoding="utf-8")

    start = data["robot"]
    goal = data["goal"]
    dist = math.hypot(float(goal["x"]) - float(start["x"]),
                      float(goal["y"]) - float(start["y"]))

    print(f"[PREVIEW] {data['case_id']}")
    print(f"  Start : ({start['x']}, {start['y']})")
    print(f"  Goal  : ({goal['x']}, {goal['y']})")
    print(f"  Direct distance: {dist:.2f} m")
    print(f"  Preview world: {out_path}")
    print("  Green pole = START")
    print("  Red pole   = GOAL")
    print("  Yellow line = direct Start→Goal line")
    print("  Gray object = actual obstacle")

    if not args.no_launch:
        subprocess.run(["gz", "sim", "-r", str(out_path)], check=False)


if __name__ == "__main__":
    main()
