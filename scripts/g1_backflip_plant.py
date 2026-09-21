"""Build the optional free-base G1 contact plant for non-RL optimization."""

import hashlib
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "assets/robots/g1_description/g1_23dof.urdf"
MODEL_MANIFEST = ROOT / "scripts/fixtures/g1_backflip_model.json"


def build_model(manifest, dt_s, kp, kd):
    """Build the declared free-base plant, with joint motors and ground contact."""
    if manifest["mass_policy"] not in ("rne", "declared"):
        raise ValueError("unknown missing-inertia policy")
    if not np.isfinite([dt_s, kp, kd]).all() or dt_s <= 0 or kp <= 0 or kd < 0:
        raise ValueError(
            "finite positive timestep/gain and nonnegative damping required"
        )
    stop_s = manifest.get("joint_limit_time_constant_s", 0.004)
    margin_rad = manifest.get("joint_limit_margin_rad", 0.0)
    if not np.isfinite([stop_s, margin_rad]).all() or stop_s <= 0 or margin_rad < 0:
        raise ValueError("invalid joint-stop settings")
    if 2 * dt_s > min(0.004, stop_s):
        raise ValueError("timestep would change the declared solver time constants")
    if manifest["urdf_sha256"] != hashlib.sha256(URDF.read_bytes()).hexdigest():
        raise ValueError("URDF checksum differs from the benchmark manifest")
    robot = ET.parse(URDF).getroot()
    robot.find("mujoco/compiler").set("meshdir", str(URDF.parent / "meshes"))
    for mesh in robot.findall(".//mesh"):
        mesh.set("filename", Path(mesh.get("filename")).name)
    ET.SubElement(robot, "link", name="world")
    joint = ET.SubElement(robot, "joint", name="root", type="floating")
    ET.SubElement(joint, "parent", link="world")
    ET.SubElement(joint, "child", link="pelvis")
    if manifest["mass_policy"] == "rne":
        for link in robot.findall("link"):
            if link.get("name") != "world" and link.find("inertial") is None:
                inertia = ET.SubElement(link, "inertial")
                ET.SubElement(inertia, "mass", value="1")
                ET.SubElement(
                    inertia,
                    "inertia",
                    ixx="1e-8",
                    iyy="1e-8",
                    izz="1e-8",
                    ixy="0",
                    ixz="0",
                    iyz="0",
                )
    original = mujoco.MjModel.from_xml_string(ET.tostring(robot, encoding="unicode"))
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "model.xml"
        mujoco.mj_saveLastXML(str(path), original)
        xml = ET.parse(path).getroot()
    ET.SubElement(
        xml,
        "option",
        timestep=str(dt_s),
        integrator="implicitfast",
        gravity="0 0 -9.81",
        iterations="100",
        tolerance="1e-10",
    )
    self_collision = manifest.get("self_collision", False)
    world = xml.find("worldbody")
    ET.SubElement(
        world,
        "geom",
        name="ground",
        type="plane",
        size="5 5 .1",
        contype="2",
        conaffinity="5" if self_collision else "1",
        friction=".7 .005 .0001",
        solref=".004 1",
        rgba=".2 .25 .3 1",
    )
    for joint in world.findall(".//joint"):
        if joint.get("type") != "free":
            joint.set(
                "solreflimit", f"{manifest.get('joint_limit_time_constant_s', 0.004)} 1"
            )
            # Unitree's G1 MJCF motor parameters, absent from the URDF.
            joint.set("margin", str(manifest.get("joint_limit_margin_rad", 0.0)))
            joint.set("armature", ".01")
            joint.set("damping", ".05")
            joint.set("frictionloss", ".2")
    for geom in world.findall(".//body/geom"):
        if geom.get("contype", "1") != "0":
            geom.set("contype", "1")
            geom.set("conaffinity", "3" if self_collision else "2")
    for side in ("left", "right"):
        body = world.find(f".//body[@name='{side}_ankle_roll_link']")
        for geom in body.findall("geom"):
            collidable = self_collision and geom.get("contype", "1") != "0"
            geom.set("contype", "1" if collidable else "0")
            geom.set("conaffinity", "1" if collidable else "0")
        for index, offset in enumerate(manifest["contact_offsets_local_m"]):
            position = np.array(offset) + [0, 0, 0.002]
            ET.SubElement(
                body,
                "geom",
                name=f"{side}_sole_{index}",
                type="sphere",
                pos=" ".join(map(str, position)),
                size=".002",
                mass="0",
                contype="4" if self_collision else "1",
                conaffinity="2",
                condim="3",
                friction=".7 .005 .0001",
                solref=".004 1",
                rgba=".1 .1 .1 1",
            )
    actuators = ET.SubElement(xml, "actuator")
    for name, limit in zip(manifest["joint_names"], manifest["torque_limits_nm"]):
        ET.SubElement(
            actuators,
            "general",
            name=name,
            joint=name,
            gaintype="fixed",
            biastype="affine",
            gainprm=str(kp),
            biasprm=f"0 {-kp} {-kd}",
            forcelimited="true",
            forcerange=f"{-limit} {limit}",
        )
    return mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
