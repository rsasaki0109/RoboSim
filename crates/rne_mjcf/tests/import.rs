//! MJCF to URDF import integration tests.

use rne_mjcf::{mjcf_to_urdf, mjcf_to_urdf_file, MjcfError};
use std::fs;
use std::path::Path;

const FIXTURE_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures");

fn fixture(name: &str) -> String {
    fs::read_to_string(Path::new(FIXTURE_DIR).join(name)).expect("read fixture")
}

#[test]
fn converts_two_link_arm_to_urdf() {
    let urdf = mjcf_to_urdf(&fixture("two_link_arm.xml")).expect("convert");
    assert!(urdf.starts_with("<?xml version=\"1.0\"?>\n"));
    assert!(urdf.contains(r#"<robot name="two_link_arm">"#));
    assert!(urdf.contains(r#"<link name="base_link">"#));
    assert!(urdf.contains(r#"<link name="upper_link">"#));
    assert!(urdf.contains(r#"<link name="tip_link">"#));
    assert!(urdf.contains(r#"<joint name="shoulder" type="revolute">"#));
    assert!(urdf.contains(r#"<parent link="base_link"/>"#));
    assert!(urdf.contains(r#"<child link="upper_link"/>"#));
    assert!(urdf.contains(r#"<origin xyz="0 0.05 0" rpy="0 0 0"/>"#));
    assert!(urdf.contains(r#"<axis xyz="0 0 1"/>"#));
    assert!(urdf.contains(r#"<limit lower="-3.14" upper="3.14" effort="0" velocity="0"/>"#));
    // MJCF box size is half-extents; the URDF box must be full extents.
    assert!(urdf.contains(r#"<box size="0.4 0.2 0.1"/>"#));
    assert!(urdf.contains(r#"<cylinder radius="0.05" length="0.5"/>"#));
    assert!(urdf.contains(r#"<sphere radius="0.03"/>"#));
    assert!(urdf.contains(r#"<color rgba="0.8 0.2 0.1 1"/>"#));
}

#[test]
fn converted_urdf_parses_with_rne_urdf_import() {
    let urdf = mjcf_to_urdf(&fixture("two_link_arm.xml")).expect("convert");
    let robot = rne_urdf_import::parse_urdf(&urdf).expect("URDF must parse");
    assert_eq!(robot.name, "two_link_arm");
}

#[test]
fn converts_from_file() {
    let urdf = mjcf_to_urdf_file(Path::new(FIXTURE_DIR).join("two_link_arm.xml").as_path())
        .expect("convert from file");
    assert!(urdf.contains(r#"<robot name="two_link_arm">"#));
}

#[test]
fn degree_compiler_converts_hinge_ranges_to_radians() {
    let text = fixture("two_link_arm.xml")
        .replace(
            "<compiler angle=\"radian\"/>",
            "<compiler angle=\"degree\"/>",
        )
        .replace("range=\"-3.14 3.14\"", "range=\"-180 180\"")
        .replace("range=\"-1.57 1.57\"", "range=\"-90 90\"");
    let urdf = mjcf_to_urdf(&text).expect("convert");
    assert!(urdf.contains(
        r#"<limit lower="-3.141592653589793" upper="3.141592653589793" effort="0" velocity="0"/>"#
    ));
    assert!(urdf.contains(
        r#"<limit lower="-1.5707963267948966" upper="1.5707963267948966" effort="0" velocity="0"/>"#
    ));
}

#[test]
fn rejects_body_rotation() {
    let text = fixture("two_link_arm.xml").replace(
        "<body name=\"upper_link\" pos=\"0 0.05 0\">",
        "<body name=\"upper_link\" pos=\"0 0.05 0\" quat=\"1 0 0 0\">",
    );
    let error = mjcf_to_urdf(&text).expect_err("body quat must be rejected");
    assert!(matches!(error, MjcfError::Unsupported { .. }));
}

#[test]
fn rejects_unsupported_joint_type() {
    let text = fixture("two_link_arm.xml").replace("type=\"hinge\"", "type=\"ball\"");
    let error = mjcf_to_urdf(&text).expect_err("ball joint must be rejected");
    assert!(matches!(error, MjcfError::Unsupported { .. }));
}

#[test]
fn canonical_urdf_is_stable() {
    let actual = mjcf_to_urdf(&fixture("two_link_arm.xml")).expect("convert");
    let expected = fs::read_to_string(Path::new(FIXTURE_DIR).join("two_link_arm.urdf"))
        .expect("read golden")
        .replace("\r\n", "\n");
    assert_eq!(actual.trim_end(), expected.trim_end());
}
