#!/usr/bin/env python3
"""Prepare and audit a native G1 comparison model without changing the source URDF."""

import argparse
import hashlib
import json
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "assets/robots/g1_description/g1_23dof.urdf"


def prepare(
    source,
    output,
    source_soles=False,
    independent_soles=False,
    source_passive_loss=False,
):
    """Remove only empty fixed leaf frames; preserve every physical link and joint."""
    source, output = Path(source).resolve(), Path(output).resolve()
    robot = ET.parse(source).getroot()
    links = robot.findall("link")
    joints = robot.findall("joint")
    removed = []
    for link in links:
        if link.find("inertial") is not None:
            continue
        name = link.get("name")
        incoming = [j for j in joints if j.find("child").get("link") == name]
        outgoing = [j for j in joints if j.find("parent").get("link") == name]
        if (
            len(link)
            or outgoing
            or len(incoming) != 1
            or incoming[0].get("type") != "fixed"
        ):
            raise ValueError(f"unresolved non-inertial physical/branch link: {name}")
        removed.append({"link": name, "joint": incoming[0].get("name")})
        robot.remove(link)
        robot.remove(incoming[0])
    masses = [
        float(link.find("inertial/mass").get("value")) for link in robot.findall("link")
    ]
    if not masses or any(not math.isfinite(m) or m <= 0 for m in masses):
        raise ValueError("finite positive declared masses required")
    mass = sum(masses)
    if source_soles:
        manifest = json.loads(
            (ROOT / "scripts/fixtures/g1_backflip_model.json").read_text()
        )
        for side in ("left", "right"):
            foot = robot.find(f"link[@name='{side}_ankle_roll_link']")
            if foot is None:
                raise ValueError("G1 feet required for source sole profile")
            for collision in foot.findall("collision"):
                foot.remove(collision)
            for x, y, z in manifest["contact_offsets_local_m"]:
                collision = ET.SubElement(foot, "collision")
                ET.SubElement(
                    collision, "origin", xyz=f"{x} {y} {z + 0.002}", rpy="0 0 0"
                )
                geometry = ET.SubElement(collision, "geometry")
                ET.SubElement(geometry, "sphere", radius="0.002")
    passive_joints = []
    if source_passive_loss:
        for joint in robot.findall("joint"):
            if joint.get("type") == "fixed":
                continue
            if joint.get("type") not in ("revolute", "continuous"):
                raise ValueError("source passive profile requires revolute joints")
            dynamics = joint.find("dynamics")
            if dynamics is None:
                dynamics = ET.SubElement(joint, "dynamics")
            dynamics.set("damping", "0.05")
            dynamics.set("friction", "0.2")
            passive_joints.append(joint.get("name"))
    mesh_count = len(robot.findall(".//collision/geometry/mesh"))
    merged = {
        link.get("name"): len(link.findall("collision"))
        for link in robot.findall("link")
        if len(link.findall("collision")) > 1
    }
    # Absolute paths keep generated assets usable outside the source directory.
    for mesh in robot.findall(".//mesh"):
        path = source.parent / "meshes" / Path(mesh.get("filename")).name
        if not path.is_file():
            raise ValueError(f"missing mesh: {path}")
        mesh.set("filename", str(path))
    if shutil.disk_usage(output.parent).free < 30 * 1024**3:
        raise RuntimeError("30 GiB disk reserve required")
    output.mkdir(exist_ok=False)
    ET.indent(robot)
    urdf = ET.tostring(robot, encoding="utf-8", xml_declaration=True)
    (output / "robot.urdf").write_bytes(urdf)
    (output / "robot.rne.robot.toml").write_text("""kind = "urdf"
model_name = "g1_declared_mass_comparison"
[urdf]
path = "robot.urdf"
base_body_type = "dynamic"
initial_translation_m = [0.0, 0.82, 0.0]
initial_rotation_rpy = [-1.5707963267948966, 0.0, 0.0]
articulation = true
collisions = true
mesh_collisions = false
self_collisions = false
multibody = true
use_declared_inertial_masses = true
use_joint_origin_rpy = true
weld_fixed_children = true
""")
    if independent_soles:
        config = output / "robot.rne.robot.toml"
        config.write_text(config.read_text() + "preserve_collision_parts = true\n")
    if source_passive_loss:
        config = output / "robot.rne.robot.toml"
        config.write_text(
            config.read_text()
            + "".join(
                f"\n[[urdf.joint_passive_dynamics]]\njoint = {json.dumps(name)}\n"
                "coulomb_transition_velocity_rad_s = 0.1\n"
                for name in passive_joints
            )
        )
    (output / "scene.rne.scene.toml").write_text("""[world]
gravity_m_s2 = [0.0, -9.81, 0.0]
seed = 2002
[ground]
enabled = true
[[robots]]
path = "robot.rne.robot.toml"
""")
    audit = {
        "source_urdf_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "generated_urdf_sha256": hashlib.sha256(urdf).hexdigest(),
        "removed_empty_fixed_leaf_frames": removed,
        "declared_mass_kg": mass,
        "source_sole_dimensions": source_soles,
        "source_passive_loss": source_passive_loss,
        "passive_loss_joint_count": len(passive_joints),
        "coulomb_transition_velocity_rad_s": 0.1 if source_passive_loss else None,
        "movable_joint_count": sum(
            j.get("type") != "fixed" for j in robot.findall("joint")
        ),
        "mesh_collision_elements_disabled": mesh_count,
        "multi_collision_links_merged_to_aabb": {} if independent_soles else merged,
        "compound_link_part_counts": merged if independent_soles else {},
        "self_collision": False,
        "qualification_ready": False,
        "remaining_differences": (
            []
            if independent_soles
            else [
                "Native multiple sole spheres are merged into an AABB; source uses four independent sole spheres."
            ]
        )
        + [
            "Native mesh collisions are disabled; enabling them currently produces AABBs, not source convex meshes.",
            (
                "Source joint armature 0.01 remains unmatched; native damping 0.05 and regularized Coulomb 0.2 use tanh(v/0.1), not source constraint friction."
                if source_passive_loss
                else "Source adds joint armature 0.01, damping 0.05 and Coulomb friction 0.2; native probe does not match these."
            ),
            "Different contact solvers and motor models; equal mass does not establish equivalent dynamics.",
        ],
    }
    (output / "audit.json").write_text(json.dumps(audit, indent=2) + "\n")
    return audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--source-soles",
        action="store_true",
        help="match source sole points/radius; use --independent-soles to avoid box merging",
    )
    parser.add_argument(
        "--independent-soles",
        action="store_true",
        help="preserve each sole sphere as a compound part",
    )
    parser.add_argument(
        "--source-passive-loss",
        action="store_true",
        help="apply damping 0.05 and regularized Coulomb 0.2 (transition 0.1 rad/s); armature remains unmatched",
    )
    args = parser.parse_args()
    print(
        json.dumps(
            prepare(
                SOURCE,
                args.output,
                args.source_soles,
                args.independent_soles,
                args.source_passive_loss,
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
