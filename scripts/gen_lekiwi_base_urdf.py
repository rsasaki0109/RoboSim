#!/usr/bin/env python3
"""Generate assets/robots/lekiwi/lekiwi_base.urdf from upstream LeKiwi.urdf."""

from __future__ import annotations

import pathlib
import re
import textwrap

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / ".tmp_lekiwi" / "URDF" / "LeKiwi.urdf"
OUT = ROOT / "assets" / "robots" / "lekiwi" / "lekiwi_base.urdf"

DROP_PREFIXES = (
    "Base_08q",
    "STS3215_03a",
    "SO_ARM100",
    "Wrist",
    "Moving_Jaw",
    "Camera",
    "WaveShare",
    "Rotation_Pitch",
    "Passive_Horn",
)

WHEEL_LINKS = {
    "4-Omni-Directional-Wheel_Single_Body-v1-2",
    "4-Omni-Directional-Wheel_Single_Body-v1-1",
    "4-Omni-Directional-Wheel_Single_Body-v1",
}

# 4" omni wheel: 101.6 mm diameter per LeKiwi BOM.
WHEEL_RADIUS_M = 0.0508
WHEEL_WIDTH_M = 0.025


def keep_link(name: str) -> bool:
    if name in {"arm_mount", "base_link"}:
        return True
    for prefix in DROP_PREFIXES:
        if name.startswith(prefix):
            return False
    return True


def rewrite_mesh_paths(block: str) -> str:
    return re.sub(
        r'filename="meshes/([^"]+)"',
        r'filename="package://lekiwi/meshes/\1"',
        block,
    )


def cylinder_wheel_link(name: str, visual_origin: str, collision_origin: str) -> str:
    cyl = (
        f'        <cylinder radius="{WHEEL_RADIUS_M}" length="{WHEEL_WIDTH_M}"/>'
    )
    return textwrap.dedent(
        f"""\
    <link name="{name}">
        <visual name="{name}_visual">
{visual_origin}
            <geometry>
{cyl}
            </geometry>
        </visual>
        <collision name="{name}_collision">
{collision_origin}
            <geometry>
{cyl}
            </geometry>
        </collision>
    </link>"""
    )


def main() -> None:
    src = SRC.read_text(encoding="utf-8")
    link_blocks = {
        name: block
        for block, name in re.findall(
            r'(    <link name="([^"]+)">.*?</link>)', src, re.S
        )
    }
    joint_blocks = {
        name: block
        for block, name in re.findall(
            r'(    <joint name="([^"]+)".*?</joint>)', src, re.S
        )
    }

    kept_links = [n for n in link_blocks if keep_link(n)]
    kept_joints = []
    for name, block in joint_blocks.items():
        parent = re.search(r'<parent link="([^"]+)"\s*/>', block)
        child = re.search(r'<child link="([^"]+)"\s*/>', block)
        if not parent or not child:
            continue
        if keep_link(parent.group(1)) and keep_link(child.group(1)):
            kept_joints.append((name, block))

    # Drop camera / arm joints already excluded by link filter.
    kept_joints = [
        (n, b)
        for n, b in kept_joints
        if not any(x in n for x in ("Camera", "Base_08q", "STS3215_03a", "SO_ARM", "Wrist", "Moving_Jaw", "Rotation_Pitch", "Passive_Horn", "WaveShare"))
    ]

    parts: list[str] = [
        "<?xml version='1.0' encoding='utf-8'?>",
        '<robot name="lekiwi_base">',
        "    <!-- Reduced LeKiwi mobile base (no arm/cameras). Upstream Z-up CAD; spawn with initial_rotation_rpy. -->",
    ]

    for name in kept_links:
        block = link_blocks[name]
        if name == "base_plate_layer1-v5":
            block = block.replace('name="base_plate_layer1-v5"', 'name="base_link"', 1)
        if name in WHEEL_LINKS:
            vis = re.search(
                r"(        <visual name=\"[^\"]+\">.*?</visual>)", block, re.S
            )
            col = re.search(
                r"(        <collision name=\"[^\"]+\">.*?</collision>)", block, re.S
            )
            vis_origin = ""
            col_origin = ""
            if vis:
                vis_origin = re.search(r"        <origin[^/]*/>", vis.group(1))
                vis_origin = vis_origin.group(0) if vis_origin else ""
            if col:
                col_origin = re.search(r"        <origin[^/]*/>", col.group(1))
                col_origin = col_origin.group(0) if col_origin else ""
            parts.append(cylinder_wheel_link(name, vis_origin, col_origin))
        else:
            cleaned = rewrite_mesh_paths(block.strip())
            # Strip inertial blocks (unsupported).
            cleaned = re.sub(r"\s*<inertial>.*?</inertial>\s*", "\n", cleaned, flags=re.S)
            parts.append(cleaned)

    parts.append(
        textwrap.dedent(
            """\
    <link name="arm_mount"/>
    <joint name="arm_mount_joint" type="fixed">
        <origin xyz="0.04 0.08 0.007000000000000002" rpy="0 0 0"/>
        <parent link="base_plate_layer2-v3"/>
        <child link="arm_mount"/>
    </joint>"""
        )
    )

    for _, block in kept_joints:
        cleaned = rewrite_mesh_paths(block.strip())
        cleaned = cleaned.replace("base_plate_layer1-v5", "base_link")
        parts.append(cleaned)

    parts.append("</robot>")
    OUT.write_text("\n".join(parts) + "\n", encoding="utf-8")
    print(
        f"wrote {OUT} ({len(kept_links)} links, {len(kept_joints)} joints, "
        f"{len(joint_blocks)} parsed joints)"
    )


if __name__ == "__main__":
    main()
