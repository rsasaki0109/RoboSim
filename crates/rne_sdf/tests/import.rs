//! SDF to URDF import integration tests.

use rne_sdf::{sdf_to_urdf, sdf_to_urdf_file, SdfError};
use std::fs;
use std::path::Path;

const FIXTURE_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures");

fn fixture(name: &str) -> String {
    fs::read_to_string(Path::new(FIXTURE_DIR).join(name)).expect("read fixture")
}

#[test]
fn converts_two_link_arm_to_urdf() {
    let urdf = sdf_to_urdf(&fixture("two_link_arm.sdf")).expect("convert");
    assert!(urdf.starts_with("<?xml version=\"1.0\"?>\n"));
    assert!(urdf.contains(r#"<robot name="two_link_arm">"#));
    assert!(urdf.contains(r#"<link name="base_link">"#));
    assert!(urdf.contains(r#"<link name="upper_link">"#));
    assert!(urdf.contains(r#"<link name="tip_link">"#));
    assert!(urdf.contains(r#"<joint name="shoulder_joint" type="revolute">"#));
    assert!(urdf.contains(r#"<parent link="base_link"/>"#));
    assert!(urdf.contains(r#"<child link="upper_link"/>"#));
    assert!(urdf.contains(r#"<axis xyz="0 0 1"/>"#));
    assert!(urdf.contains(r#"<limit lower="-3.14" upper="3.14" effort="10" velocity="2"/>"#));
    assert!(urdf.contains(r#"<mass value="2"/>"#));
    assert!(urdf.contains(r#"<box size="0.4 0.2 0.1"/>"#));
    assert!(urdf.contains(r#"<cylinder radius="0.05" length="0.5"/>"#));
    assert!(urdf.contains(r#"<sphere radius="0.03"/>"#));
    assert!(urdf.contains(r#"<color rgba="0.8 0.2 0.1 1"/>"#));
    assert!(urdf.contains(r#"<origin xyz="0 0 0.05" rpy="0 0 0"/>"#));
}

#[test]
fn converted_urdf_parses_with_rne_urdf_import() {
    let urdf = sdf_to_urdf(&fixture("two_link_arm.sdf")).expect("convert");
    let robot = rne_urdf_import::parse_urdf(&urdf).expect("URDF must parse");
    assert_eq!(robot.name, "two_link_arm");
}

#[test]
fn converts_from_file() {
    let urdf = sdf_to_urdf_file(Path::new(FIXTURE_DIR).join("two_link_arm.sdf").as_path())
        .expect("convert from file");
    assert!(urdf.contains(r#"<robot name="two_link_arm">"#));
}

#[test]
fn canonical_urdf_is_stable() {
    let actual = sdf_to_urdf(&fixture("two_link_arm.sdf")).expect("convert");
    let expected = fs::read_to_string(Path::new(FIXTURE_DIR).join("two_link_arm.urdf"))
        .expect("read golden")
        .replace("\r\n", "\n");
    assert_eq!(actual.trim_end(), expected.trim_end());
}

#[test]
fn rejects_world_wrapper() {
    let text = fixture("two_link_arm.sdf").replace(
        "<model name=\"two_link_arm\">",
        "<world name=\"default\"><model name=\"two_link_arm\">",
    );
    let error = sdf_to_urdf(&text).expect_err("world must be rejected");
    assert!(error.to_string().contains("world"));
}

#[test]
fn rejects_unsupported_joint_type() {
    let text = fixture("two_link_arm.sdf").replace("type=\"revolute\"", "type=\"universal\"");
    let error = sdf_to_urdf(&text).expect_err("universal joint must be rejected");
    assert!(matches!(error, SdfError::Unsupported { .. }));
}
