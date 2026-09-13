#!/usr/bin/env python3
"""Generate assets/robots/mm_mobile_so101/mm_mobile_so101.urdf.

Combines the mm_mobile diff-drive base (base_link + wheels) with the SO101
6-DoF arm. Run from the repository root after editing either source URDF:

    python3 scripts/gen_mm_mobile_so101_urdf.py
"""

from __future__ import annotations

import pathlib
import re

ROOT = pathlib.Path(__file__).resolve().parents[1]
MM = ROOT / "assets" / "robots" / "mm_mobile" / "mm_mobile.urdf"
SO101 = ROOT / "assets" / "robots" / "so101" / "so101.urdf"
OUT = ROOT / "assets" / "robots" / "mm_mobile_so101" / "mm_mobile_so101.urdf"

# Arm mount on the chassis front-top (base half-length 0.25, top at y=0.15):
# puts the jaw pocket ~0.55 m ahead of the base center at zero hold.
MOUNT_ORIGIN = 'xyz="0.20 0.15 0" rpy="0 0 0"'

# Upstream gripper_frame_link carries degenerate (1e-9 mass, zero inertia)
# properties that multibody spawn rejects; substitute a small valid inertial
# (fixed to gripper_link, so dynamics impact is negligible).
FRAME_INERTIAL = """<inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.005"/>
      <inertia ixx="1e-06" ixy="0" ixz="0" iyy="1e-06" iyz="0" izz="1e-06"/>
    </inertial>"""

# Fixed-jaw anvil pad at the frame-joint origin (link-local gripper frame).
# Rides on gripper_link because fixed joints outside the multibody set are not
# wired to physics (the frame link would fall as debris once given a collider).
FRAME_PAD_BOX_SIZE = "0.02 0.012 0.02"
# Moving-jaw grip pad at the jaw tip (link-local).
JAW_PAD_ORIGIN = "0 -0.072 0.019"
JAW_PAD_BOX_SIZE = "0.016 0.01 0.016"


def extract_blocks(xml: str, tag: str) -> list[str]:
    pattern = rf"<{tag}\b[^>]*>.*?</{tag}>"
    return re.findall(pattern, xml, re.S)


def strip_transmission(xml: str) -> str:
    return re.sub(r"\s*<transmission\b.*?</transmission>\s*", "\n", xml, flags=re.S)


def strip_collisions(block: str) -> str:
    return re.sub(r"\s*<collision\b[^>]*>.*?</collision>\s*", "\n", block, flags=re.S)


def main() -> None:
    mm_xml = MM.read_text(encoding="utf-8")
    so_xml = strip_transmission(SO101.read_text(encoding="utf-8"))

    keep_links = {"base_link", "left_wheel", "right_wheel"}
    mm_links = [
        block
        for block in extract_blocks(mm_xml, "link")
        if re.search(r'link name="([^"]+)"', block).group(1) in keep_links
    ]
    keep_joints = {"left_wheel_joint", "right_wheel_joint"}
    mm_joints = [
        block
        for block in extract_blocks(mm_xml, "joint")
        if re.search(r'joint name="([^"]+)"', block).group(1) in keep_joints
    ]

    # Frame-joint origin (anvil pose): parsed from the joint so the anvil tracks
    # upstream geometry instead of hardcoding numbers.
    frame_joint = next(
        block
        for block in extract_blocks(so_xml, "joint")
        if 'name="gripper_frame_joint"' in block
    )
    frame_origin = re.search(r'<origin xyz="([^"]+)" rpy="([^"]+)"', frame_joint)
    assert frame_origin, "gripper_frame_joint origin missing"

    # Explicit grip pads replace the links' mesh-AABB colliders. A URDF link
    # merges every collision element into one union cuboid, so the wrist, motor
    # shell, and frame would otherwise bury the jaws in a bulky box that never
    # contacts a small object cleanly. The anvil sits at the frame-joint origin
    # and the jaw pad at the moving jaw's tip; both are authored in link-local
    # frames (see the runner probes that measured the tip/anvil gap).
    anvil_pad = (
        "<collision>\n"
        f'        <origin xyz="{frame_origin.group(1)}" rpy="{frame_origin.group(2)}"/>\n'
        "        <geometry>\n"
        f"          <box size=\"{FRAME_PAD_BOX_SIZE}\"/>\n"
        "        </geometry>\n"
        "      </collision>"
    )
    jaw_pad = (
        "<collision>\n"
        f'        <origin xyz="{JAW_PAD_ORIGIN}"/>\n'
        "        <geometry>\n"
        f"          <box size=\"{JAW_PAD_BOX_SIZE}\"/>\n"
        "        </geometry>\n"
        "      </collision>"
    )

    so_links = []
    for block in extract_blocks(so_xml, "link"):
        block = re.sub(
            r'filename="package://so101/([^"]+)"',
            r'filename="../so101/\1"',
            block,
        )
        block = block.replace('name="base_link"', 'name="so101_base_link"', 1)
        if 'name="so101_base_link"' not in block:
            # Drop every arm link's mesh-AABB collider: the union cuboid spans
            # the whole STL shell, so the wrist and forearm boxes collide with a
            # tabletop (and each other) long before the jaws reach the object.
            # Only the authored jaw/anvil grip pads below provide contact.
            block = strip_collisions(block)
        if 'name="gripper_frame_link"' in block:
            block = re.sub(r"<inertial>.*?</inertial>", FRAME_INERTIAL, block, flags=re.S)
        if 'name="gripper_link"' in block:
            block = block.replace("</link>", f"    {anvil_pad}\n</link>", 1)
        if 'name="moving_jaw_so101_v1_link"' in block:
            block = block.replace("</link>", f"    {jaw_pad}\n</link>", 1)
        so_links.append(block)

    # Fold the chassis mount into shoulder_pan and mount the arm directly on
    # `base_link`. An intermediate fixed link (so101_base_link) is NOT part of
    # the reduced-coordinate chain in the way a revolute child is: its fixed
    # joint does not reliably hold it inside the multibody assembly, so the
    # whole arm base drifts off the chassis during a drive. Parenting the first
    # revolute straight to base_link keeps the entire arm in one assembly.
    mount_xyz = [float(value) for value in re.findall(r"-?\d+\.?\d*(?:e-?\d+)?", MOUNT_ORIGIN)[:3]]
    so_joints = []
    for block in extract_blocks(so_xml, "joint"):
        block = re.sub(
            r'filename="package://so101/([^"]+)"',
            r'filename="../so101/\1"',
            block,
        )
        block = re.sub(r"\s*<mimic\b[^>]*/>\s*", "\n", block)
        if 'name="shoulder_pan"' in block:
            block = re.sub(r'<parent link="[^"]+"/>', '<parent link="base_link"/>', block)
            origin = re.search(r'<origin xyz="([^"]+)" rpy="([^"]+)"', block)
            assert origin, "shoulder_pan origin missing"
            xyz = [float(value) for value in origin.group(1).split()]
            folded = " ".join(
                f"{xyz[index] + mount_xyz[index]:.9g}" for index in range(3)
            )
            block = block.replace(origin.group(1), folded, 1)
        else:
            block = block.replace('link="base_link"', 'link="so101_base_link"')
        so_joints.append(block)

    # so101_base_link is dropped: the arm mounts directly on base_link, and its
    # decorative base plate would otherwise be an unheld fixed link.
    so_links = [block for block in so_links if 'name="so101_base_link"' not in block]

    parts = [
        '<?xml version="1.0"?>',
        '<robot name="mm_mobile_so101">',
        "    <!-- mm_mobile diff-drive base + SO101 6-DoF arm (generated). -->",
        "    <!-- SO101 inertial kept for multibody; arm mounts directly on base_link. -->",
        *[link.strip() for link in mm_links],
        *[link.strip() for link in so_links],
        *[joint.strip() for joint in mm_joints],
        *[joint.strip() for joint in so_joints],
        "</robot>",
    ]

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(parts) + "\n", encoding="utf-8")
    print(f"wrote {OUT} ({len(so_links)} arm links)")


if __name__ == "__main__":
    main()
