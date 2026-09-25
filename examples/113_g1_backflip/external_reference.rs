//! Checks the external Pinocchio transcription with native RNE dynamics.
//!
//! This evaluates the same saved states, accelerations and contact forces; it
//! does not integrate a physics plant or claim a successful maneuver.

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use rne_dynamics::{frame_jacobian, rnea, ArticulatedModel};
use rne_math::{Quat, Vec3};
use rne_urdf_import::SpawnedUrdfRobot;
use serde::Deserialize;
use sha2::{Digest, Sha256};

#[derive(Debug, Deserialize)]
struct Manifest {
    schema: String,
    urdf_sha256: String,
    contact_offsets_local_m: Vec<[f64; 3]>,
    mass_kg: f64,
    trajectory_sha256: Option<String>,
}

#[derive(Debug, Deserialize)]
struct Trajectory {
    world_up: String,
    joint_names: Vec<String>,
    nodes: Vec<Node>,
}

#[derive(Debug, Deserialize)]
struct Node {
    q_xyzw: Vec<f64>,
    v_body: Vec<f64>,
    a_body: Vec<f64>,
    tau_nm: Vec<f64>,
    contact_forces_world_n: BTreeMap<String, [f64; 3]>,
}

fn world_vector(value: [f64; 3]) -> Vec3 {
    Vec3::new(value[0], value[2], -value[1])
}

fn base_configuration(position: [f64; 3], rotation_xyzw: [f64; 4]) -> [f64; 6] {
    let axes = Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2);
    // RNE composes Euler * stored_root, while Pinocchio's root is the full pose.
    let rotation = axes * Quat::from_array(rotation_xyzw) * axes.conjugate();
    let Quat { x, y, z, w } = rotation;
    let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
    let pitch = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0).asin();
    let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
    let position = world_vector(position);
    [position.x, position.y, position.z, roll, pitch, yaw]
}

/// Audits a saved external trajectory against the native articulated model.
pub(super) fn check(
    model: &ArticulatedModel,
    spawned: &SpawnedUrdfRobot,
    urdf: &str,
    directory: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    let manifest: Manifest =
        serde_json::from_slice(&std::fs::read(directory.join("summary.json"))?)?;
    let trajectory_bytes = std::fs::read(directory.join("trajectory.json"))?;
    let trajectory: Trajectory = serde_json::from_slice(&trajectory_bytes)?;
    let require = |condition: bool, message: &str| -> Result<(), Box<dyn std::error::Error>> {
        if condition {
            Ok(())
        } else {
            Err(message.into())
        }
    };
    require(
        manifest.schema == "rne.g1.trajopt_reference.v1"
            && manifest.urdf_sha256 == format!("{:x}", Sha256::digest(urdf.as_bytes())),
        "reference schema or URDF checksum mismatch",
    )?;
    if let Some(expected) = &manifest.trajectory_sha256 {
        require(
            *expected == format!("{:x}", Sha256::digest(&trajectory_bytes)),
            "trajectory checksum mismatch",
        )?;
    }
    let nv = model.nv();
    require(
        trajectory.world_up == "Z"
            && !trajectory.nodes.is_empty()
            && trajectory.joint_names.len() == nv - 6,
        "invalid reference dimensions or world convention",
    )?;
    let mut indices = Vec::new();
    for name in &trajectory.joint_names {
        let entity = spawned.joints.get(name).ok_or("unknown reference joint")?;
        indices.push(
            model
                .kinematic()
                .dof_index_of_joint(*entity)
                .ok_or("reference joint is fixed")?
                + 6,
        );
    }
    require(
        indices.iter().collect::<BTreeSet<_>>().len() == nv - 6,
        "duplicate reference joints",
    )?;
    let mut base_force_n = 0.0_f64;
    let mut base_torque_nm = 0.0_f64;
    let mut joint_torque_difference_nm = 0.0_f64;
    for node in &trajectory.nodes {
        require(
            node.q_xyzw.len() == nv + 1
                && node.v_body.len() == nv
                && node.a_body.len() == nv
                && node.tau_nm.len() == nv - 6,
            "invalid reference node dimensions",
        )?;
        require(
            node.q_xyzw
                .iter()
                .chain(&node.v_body)
                .chain(&node.a_body)
                .chain(&node.tau_nm)
                .chain(node.contact_forces_world_n.values().flatten())
                .all(|v| v.is_finite()),
            "non-finite reference value",
        )?;
        let rotation: [f64; 4] = node.q_xyzw[3..7].try_into()?;
        require(
            (Quat::from_array(rotation).length_squared() - 1.0).abs() < 1.0e-8,
            "non-unit quaternion",
        )?;
        let mut q = vec![0.0; nv];
        let mut v = vec![0.0; nv];
        let mut a = vec![0.0; nv];
        q[..6].copy_from_slice(&base_configuration(node.q_xyzw[..3].try_into()?, rotation));
        // These are local body components: changing world axes does not rotate them.
        v[..6].copy_from_slice(&node.v_body[..6]);
        a[..6].copy_from_slice(&node.a_body[..6]);
        for (source, &target) in indices.iter().enumerate() {
            q[target] = node.q_xyzw[7 + source];
            v[target] = node.v_body[6 + source];
            a[target] = node.a_body[6 + source];
        }
        let mut torque = rnea(model, &q, &v, &a)?;
        for (name, force) in &node.contact_forces_world_n {
            let (side, index) = name
                .strip_prefix("rne_")
                .and_then(|s| s.split_once("_contact_"))
                .ok_or("invalid contact name")?;
            require(side == "left" || side == "right", "invalid contact side")?;
            let offset = manifest
                .contact_offsets_local_m
                .get(index.parse::<usize>()?)
                .ok_or("invalid contact offset index")?;
            require(
                offset.iter().all(|v| v.is_finite()),
                "non-finite contact offset",
            )?;
            let link = spawned
                .links
                .get(&format!("{side}_ankle_roll_link"))
                .ok_or("missing foot link")?;
            let jacobian = frame_jacobian(model, &q, *link, Vec3::from_array(*offset))?;
            let force = world_vector(*force);
            for (column, value) in torque.iter_mut().enumerate() {
                *value -= jacobian.get(0, column) * force.x
                    + jacobian.get(1, column) * force.y
                    + jacobian.get(2, column) * force.z;
            }
        }
        require(
            torque.iter().all(|v| v.is_finite()),
            "non-finite native inverse dynamics",
        )?;
        for value in &torque[..3] {
            base_force_n = base_force_n.max(value.abs());
        }
        for value in &torque[3..6] {
            base_torque_nm = base_torque_nm.max(value.abs());
        }
        for (source, &target) in indices.iter().enumerate() {
            joint_torque_difference_nm =
                joint_torque_difference_nm.max((torque[target] - node.tau_nm[source]).abs());
        }
    }
    let native_mass_kg: f64 = (0..model.link_count())
        .map(|index| model.link_inertia(index).expect("link inertia").mass_kg)
        .sum();
    let mass_difference_kg = (native_mass_kg - manifest.mass_kg).abs();
    let passed = mass_difference_kg.is_finite()
        && mass_difference_kg < 1.0e-8
        && base_force_n
            .max(base_torque_nm)
            .max(joint_torque_difference_nm)
            < 1.0e-4;
    println!(
        "{}",
        serde_json::to_string_pretty(&serde_json::json!({
            "schema": "rne.g1.external_dynamics_check.v1",
            "nodes": trajectory.nodes.len(),
            "base_force_residual_n": base_force_n,
            "base_torque_residual_nm": base_torque_nm,
            "joint_torque_difference_nm": joint_torque_difference_nm,
            "native_mass_kg": native_mass_kg,
            "mass_difference_kg": mass_difference_kg,
            "tolerance": 1.0e-4,
            "passed": passed,
            "plant_validated": false
        }))?
    );
    require(passed, "native and external dynamics check failed")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn external_rotation_reconstructs_the_native_root_pose() {
        let axes = Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2);
        for angle in [0.0, -1.0, -3.0, 2.0] {
            let external = Quat::from_rotation_y(angle) * Quat::from_rotation_x(0.1);
            let q = base_configuration([1.0, 2.0, 3.0], external.to_array());
            assert_eq!(&q[..3], &[1.0, 3.0, -2.0]);
            let actual = Quat::from_rotation_z(q[5])
                * Quat::from_rotation_y(q[4])
                * Quat::from_rotation_x(q[3])
                * axes;
            assert!(actual.dot(axes * external).abs() > 1.0 - 1.0e-12);
        }
    }
}
