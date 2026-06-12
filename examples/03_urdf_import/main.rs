//! Imports a minimal URDF diff drive model into the ECS.

use rne_urdf_import::{parse_urdf, spawn_urdf_robot};

fn main() {
    let xml =
        include_str!("../../adapters/ros2/rne_urdf_import/tests/fixtures/minimal_diff_drive.urdf");
    let urdf = parse_urdf(xml).expect("parse URDF");
    let mut world = rne_ecs::World::new();
    let spawned = spawn_urdf_robot(&mut world, &urdf).expect("spawn URDF robot");

    println!(
        "imported robot={} links={} joints={} colliders={} visuals={}",
        urdf.name,
        spawned.links.len(),
        spawned.joints.len(),
        spawned.collider_count,
        spawned.visual_count
    );
    println!("base_link entity index = {}", spawned.base_link.index());
}
