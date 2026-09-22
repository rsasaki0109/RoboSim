//! Minimal MuJoCo MJCF model to URDF conversion.

use crate::MjcfError;
use roxmltree::{Document, Node};
use std::collections::BTreeMap;
use std::io::Read;
use std::path::Path;

const DEG_TO_RAD: f64 = std::f64::consts::PI / 180.0;
const MJCF_MAX_BODY_DEPTH: usize = 128;
const MJCF_MAX_OUTPUT_BYTES: usize = 64 * 1024 * 1024;

/// Maximum accepted MJCF XML input size.
pub const MJCF_MAX_INPUT_BYTES: usize = 16 * 1024 * 1024;

/// Angular unit convention from the MJCF `<compiler>`.
#[derive(Clone, Copy, Debug, PartialEq)]
enum AngleConvention {
    Degree,
    Radian,
}

/// Converts a minimal MJCF model document into a URDF XML string.
pub fn mjcf_to_urdf(text: &str) -> Result<String, MjcfError> {
    ensure_input_len(text.len())?;
    let document = Document::parse(text).map_err(|error| MjcfError::Xml(error.to_string()))?;
    let root = document.root_element();
    if root.tag_name().name() != "mujoco" {
        return Err(MjcfError::Invalid(
            "root element must be `mujoco`".to_string(),
        ));
    }
    let angle = first_child_element(root, "compiler")
        .and_then(|compiler| compiler.attribute("angle"))
        .map(|value| match value {
            "radian" => Ok(AngleConvention::Radian),
            "degree" => Ok(AngleConvention::Degree),
            other => Err(MjcfError::Invalid(format!(
                "unsupported compiler angle `{other}`"
            ))),
        })
        .transpose()?
        .unwrap_or(AngleConvention::Degree);

    let worldbody = first_child_element(root, "worldbody")
        .ok_or_else(|| MjcfError::Invalid("missing `<worldbody>`".to_string()))?;
    let root_bodies = child_elements(worldbody)
        .filter(|node| node.tag_name().name() == "body")
        .collect::<Vec<_>>();
    if root_bodies.len() != 1 {
        return Err(MjcfError::Invalid(format!(
            "expected exactly one root `<body>`, found {}",
            root_bodies.len()
        )));
    }
    let root_body = root_bodies[0];
    if first_child_element(root_body, "joint").is_some() {
        return Err(MjcfError::Unsupported {
            element: "joint".to_string(),
            reason: "a root body joint (free/movable base) is not supported".to_string(),
        });
    }
    reject_body_rotation(root_body)?;

    let meshes = parse_asset_meshes(root)?;
    let model_name = root.attribute("model").unwrap_or("model");
    let mut out = String::from("<?xml version=\"1.0\"?>\n");
    out.push_str(&format!("<robot name=\"{}\">\n", escape_attr(model_name)));
    render_body(root_body, None, angle, &meshes, 0, &mut out)?;
    out.push_str("</robot>\n");
    Ok(out)
}

/// Reads an MJCF model file and converts it.
pub fn mjcf_to_urdf_file(path: &Path) -> Result<String, MjcfError> {
    let mut bytes = Vec::new();
    std::fs::File::open(path)?
        .take((MJCF_MAX_INPUT_BYTES as u64) + 1)
        .read_to_end(&mut bytes)?;
    ensure_input_len(bytes.len())?;
    let text =
        String::from_utf8(bytes).map_err(|error| MjcfError::Xml(error.utf8_error().to_string()))?;
    mjcf_to_urdf(&text)
}

fn ensure_input_len(actual: usize) -> Result<(), MjcfError> {
    if actual > MJCF_MAX_INPUT_BYTES {
        return Err(MjcfError::Invalid(format!(
            "input is {actual} bytes, limit is {MJCF_MAX_INPUT_BYTES} bytes"
        )));
    }
    Ok(())
}

fn render_body(
    body: Node<'_, '_>,
    parent_link: Option<&str>,
    angle: AngleConvention,
    meshes: &BTreeMap<String, MeshAsset>,
    depth: usize,
    out: &mut String,
) -> Result<(), MjcfError> {
    if depth > MJCF_MAX_BODY_DEPTH {
        return Err(MjcfError::Invalid(format!(
            "body nesting exceeds {MJCF_MAX_BODY_DEPTH} levels"
        )));
    }
    let name = required_attr(&body, "body", "name")?;
    if let Some(parent_link) = parent_link {
        render_joint(body, parent_link, name, angle, out)?;
    }
    render_link(body, name, angle, meshes, out)?;
    if out.len() > MJCF_MAX_OUTPUT_BYTES {
        return Err(MjcfError::Invalid(format!(
            "converted URDF exceeds {MJCF_MAX_OUTPUT_BYTES} bytes"
        )));
    }
    for child in child_elements(body).filter(|node| node.tag_name().name() == "body") {
        render_body(child, Some(name), angle, meshes, depth + 1, out)?;
    }
    Ok(())
}

/// A referenced mesh asset from `<asset>`.
#[derive(Clone, Debug, PartialEq)]
struct MeshAsset {
    file: String,
    scale: [f64; 3],
}

fn parse_asset_meshes(root: Node<'_, '_>) -> Result<BTreeMap<String, MeshAsset>, MjcfError> {
    let mut meshes = BTreeMap::new();
    let Some(asset) = first_child_element(root, "asset") else {
        return Ok(meshes);
    };
    for mesh in child_elements(asset).filter(|node| node.tag_name().name() == "mesh") {
        let name = required_attr(&mesh, "mesh", "name")?;
        let file = required_attr(&mesh, "mesh", "file")?;
        let scale = mesh
            .attribute("scale")
            .map(|value| parse_vec3(value, "mesh@scale"))
            .transpose()?
            .unwrap_or([1.0, 1.0, 1.0]);
        meshes.insert(
            name.to_string(),
            MeshAsset {
                file: file.to_string(),
                scale,
            },
        );
    }
    Ok(meshes)
}

/// Converts a node's MJCF rotation attributes to URDF `rpy` (radians).
fn node_rotation_rpy(node: Node<'_, '_>, angle: AngleConvention) -> Result<[f64; 3], MjcfError> {
    if let Some(quat) = node.attribute("quat") {
        let [w, x, y, z] = parse_vec4(quat, "quat")?;
        return quat_to_rpy([w, x, y, z]);
    }
    if let Some(euler) = node.attribute("euler") {
        let euler = parse_vec3(euler, "euler")?;
        let scale = if angle == AngleConvention::Degree {
            DEG_TO_RAD
        } else {
            1.0
        };
        return Ok([euler[0] * scale, euler[1] * scale, euler[2] * scale]);
    }
    if let Some(axisangle) = node.attribute("axisangle") {
        let [x, y, z, a] = parse_vec4(axisangle, "axisangle")?;
        return axis_angle_to_rpy([x, y, z], a);
    }
    if node.attribute("zaxis").is_some() {
        return Err(MjcfError::Unsupported {
            element: node.tag_name().name().to_string(),
            reason: "`@zaxis` rotation is not supported".to_string(),
        });
    }
    Ok([0.0, 0.0, 0.0])
}

fn quat_to_rpy(quat: [f64; 4]) -> Result<[f64; 3], MjcfError> {
    let [w, x, y, z] = quat;
    let length = (w * w + x * x + y * y + z * z).sqrt();
    if !length.is_finite() || length < 1.0e-9 {
        return Err(MjcfError::Invalid("quaternion is degenerate".to_string()));
    }
    let (w, x, y, z) = (w / length, x / length, y / length, z / length);
    // Extrinsic XYZ (URDF roll-pitch-yaw).
    let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
    let pitch = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0).asin();
    let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
    Ok([roll, pitch, yaw])
}

fn axis_angle_to_rpy(axis: [f64; 3], angle: f64) -> Result<[f64; 3], MjcfError> {
    let length = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    if !length.is_finite() || length < 1.0e-9 {
        return Err(MjcfError::Invalid(
            "axisangle axis is degenerate".to_string(),
        ));
    }
    let half = angle * 0.5;
    let s = half.sin() / length;
    quat_to_rpy([half.cos(), axis[0] * s, axis[1] * s, axis[2] * s])
}

fn reject_body_rotation(body: Node<'_, '_>) -> Result<(), MjcfError> {
    for attribute in ["quat", "euler", "zaxis"] {
        if body.attribute(attribute).is_some() {
            return Err(MjcfError::Unsupported {
                element: "body".to_string(),
                reason: format!("body `@{attribute}` rotation is not supported"),
            });
        }
    }
    Ok(())
}

fn render_joint(
    body: Node<'_, '_>,
    parent_link: &str,
    child_link: &str,
    angle: AngleConvention,
    out: &mut String,
) -> Result<(), MjcfError> {
    let joint = first_child_element(body, "joint").ok_or_else(|| {
        MjcfError::Invalid(format!(
            "body `{child_link}` must declare a `<joint>` or be the root body"
        ))
    })?;
    let joint_name = joint.attribute("name").unwrap_or(child_link);
    let joint_type = joint.attribute("type").unwrap_or("hinge");
    let urdf_type = match joint_type {
        "hinge" => "revolute",
        "slide" => "prismatic",
        other => {
            return Err(MjcfError::Unsupported {
                element: "joint".to_string(),
                reason: format!("joint type `{other}` is not supported"),
            })
        }
    };
    let pos = joint
        .attribute("pos")
        .map(|value| parse_vec3(value, "joint@pos"))
        .transpose()?
        .unwrap_or(vec_attr(body, "pos", [0.0, 0.0, 0.0]));
    let axis = joint
        .attribute("axis")
        .map(|value| parse_vec3(value, "joint@axis"))
        .transpose()?
        .unwrap_or([0.0, 0.0, 1.0]);
    let (lower, upper) = joint
        .attribute("range")
        .map(|value| parse_range(value, joint_type, angle))
        .transpose()?
        .unwrap_or((0.0, 0.0));

    out.push_str(&format!(
        "  <joint name=\"{}\" type=\"{}\">\n",
        escape_attr(joint_name),
        urdf_type
    ));
    out.push_str(&format!(
        "    <parent link=\"{}\"/>\n    <child link=\"{}\"/>\n",
        escape_attr(parent_link),
        escape_attr(child_link)
    ));
    let body_rpy = node_rotation_rpy(body, angle)?;
    out.push_str(&format!(
        "    <origin xyz=\"{}\" rpy=\"{}\"/>\n",
        vec3_string(&pos),
        vec3_string(&body_rpy)
    ));
    out.push_str(&format!("    <axis xyz=\"{}\"/>\n", vec3_string(&axis)));
    out.push_str(&format!(
        "    <limit lower=\"{}\" upper=\"{}\" effort=\"0\" velocity=\"0\"/>\n",
        num(lower),
        num(upper)
    ));
    out.push_str("  </joint>\n");
    Ok(())
}

fn render_link(
    body: Node<'_, '_>,
    name: &str,
    angle: AngleConvention,
    meshes: &BTreeMap<String, MeshAsset>,
    out: &mut String,
) -> Result<(), MjcfError> {
    out.push_str(&format!("  <link name=\"{}\">\n", escape_attr(name)));
    let mut has_geom = false;
    for geom in child_elements(body).filter(|node| node.tag_name().name() == "geom") {
        let pos = vec_attr(geom, "pos", [0.0, 0.0, 0.0]);
        let geom_rpy = node_rotation_rpy(geom, angle)?;
        let geometry = render_geom_geometry(&geom, meshes)?;
        out.push_str("    <visual>\n");
        out.push_str(&format!(
            "      <origin xyz=\"{}\" rpy=\"{}\"/>\n",
            vec3_string(&pos),
            vec3_string(&geom_rpy)
        ));
        out.push_str(&format!("      {geometry}\n"));
        if let Some(rgba) = geom.attribute("rgba") {
            let rgba = parse_vec4(rgba, "geom@rgba")?;
            out.push_str(&format!(
                "      <material name=\"material\"><color rgba=\"{}\"/></material>\n",
                vec4_string(&rgba)
            ));
        }
        out.push_str("    </visual>\n");
        out.push_str("    <collision>\n");
        out.push_str(&format!(
            "      <origin xyz=\"{}\" rpy=\"{}\"/>\n",
            vec3_string(&pos),
            vec3_string(&geom_rpy)
        ));
        out.push_str(&format!("      {geometry}\n"));
        out.push_str("    </collision>\n");
        has_geom = true;
    }
    if !has_geom {
        return Err(MjcfError::Invalid(format!(
            "body `{name}` has no `<geom>` children"
        )));
    }
    out.push_str("  </link>\n");
    Ok(())
}

fn render_geom_geometry(
    geom: &Node<'_, '_>,
    meshes: &BTreeMap<String, MeshAsset>,
) -> Result<String, MjcfError> {
    let geom_type = geom.attribute("type").unwrap_or("sphere");
    let size = geom
        .attribute("size")
        .map(|value| parse_vec_any(value, "geom@size"))
        .transpose()?
        .unwrap_or_default();
    match geom_type {
        "box" => {
            let [x, y, z] = three(size, "box geom size")?;
            // MJCF box size is half-extents; URDF box size is full extents.
            Ok(format!(
                "<geometry><box size=\"{}\"/></geometry>",
                vec3_string(&[2.0 * x, 2.0 * y, 2.0 * z])
            ))
        }
        "sphere" => {
            let radius = first(size, "sphere geom size")?;
            Ok(format!(
                "<geometry><sphere radius=\"{}\"/></geometry>",
                num(radius)
            ))
        }
        "cylinder" => {
            let [radius, length] = two(size, "cylinder geom size")?;
            Ok(format!(
                "<geometry><cylinder radius=\"{}\" length=\"{}\"/></geometry>",
                num(radius),
                num(length)
            ))
        }
        "capsule" => {
            // URDF has no capsule primitive; approximate with a cylinder.
            let [radius, half_length] = two(size, "capsule geom size")?;
            Ok(format!(
                "<geometry><cylinder radius=\"{}\" length=\"{}\"/></geometry>",
                num(radius),
                num(2.0 * half_length)
            ))
        }
        "mesh" => {
            let name = required_attr(geom, "geom", "mesh")?;
            let asset = meshes.get(name).ok_or_else(|| {
                MjcfError::Invalid(format!("geom references unknown mesh asset `{name}`"))
            })?;
            Ok(format!(
                "<geometry><mesh filename=\"{}\" scale=\"{}\"/></geometry>",
                escape_attr(&asset.file),
                vec3_string(&asset.scale)
            ))
        }
        other => Err(MjcfError::Unsupported {
            element: "geom".to_string(),
            reason: format!("geom type `{other}` is not supported"),
        }),
    }
}

fn parse_range(
    text: &str,
    joint_type: &str,
    angle: AngleConvention,
) -> Result<(f64, f64), MjcfError> {
    let [lower, upper] = two(parse_vec_any(text, "joint@range")?, "joint@range")?;
    let (lower, upper) = if joint_type == "hinge" && angle == AngleConvention::Degree {
        (lower * DEG_TO_RAD, upper * DEG_TO_RAD)
    } else {
        (lower, upper)
    };
    Ok((lower, upper))
}

fn parse_vec3(text: &str, field: &str) -> Result<[f64; 3], MjcfError> {
    let values = text
        .split_whitespace()
        .map(|value| parse_scalar(value, field))
        .collect::<Result<Vec<_>, _>>()?;
    three(values, field)
}

fn parse_vec4(text: &str, field: &str) -> Result<[f64; 4], MjcfError> {
    let values = text
        .split_whitespace()
        .map(|value| parse_scalar(value, field))
        .collect::<Result<Vec<_>, _>>()?;
    if values.len() != 4 {
        return Err(MjcfError::Invalid(format!(
            "`{field}` must contain exactly four numbers"
        )));
    }
    Ok([values[0], values[1], values[2], values[3]])
}

fn parse_vec_any(text: &str, field: &str) -> Result<Vec<f64>, MjcfError> {
    text.split_whitespace()
        .map(|value| parse_scalar(value, field))
        .collect()
}

fn parse_scalar(text: &str, field: &str) -> Result<f64, MjcfError> {
    let value = text
        .trim()
        .parse::<f64>()
        .map_err(|_| MjcfError::Invalid(format!("`{field}` must be a number")))?;
    if !value.is_finite() {
        return Err(MjcfError::Invalid(format!("`{field}` must be finite")));
    }
    Ok(value)
}

fn three(values: Vec<f64>, field: &str) -> Result<[f64; 3], MjcfError> {
    if values.len() != 3 {
        return Err(MjcfError::Invalid(format!(
            "`{field}` must contain exactly three numbers"
        )));
    }
    Ok([values[0], values[1], values[2]])
}

fn two(values: Vec<f64>, field: &str) -> Result<[f64; 2], MjcfError> {
    if values.len() != 2 {
        return Err(MjcfError::Invalid(format!(
            "`{field}` must contain exactly two numbers"
        )));
    }
    Ok([values[0], values[1]])
}

fn first(values: Vec<f64>, field: &str) -> Result<f64, MjcfError> {
    values
        .first()
        .copied()
        .ok_or_else(|| MjcfError::Invalid(format!("`{field}` must not be empty")))
}

fn required_attr<'a, 'input>(
    node: &Node<'a, 'input>,
    element: &str,
    attribute: &str,
) -> Result<&'a str, MjcfError> {
    node.attribute(attribute)
        .ok_or_else(|| MjcfError::Invalid(format!("`{element}` requires `@{attribute}`")))
}

fn vec_attr(node: Node<'_, '_>, attribute: &str, default: [f64; 3]) -> [f64; 3] {
    node.attribute(attribute)
        .and_then(|text| parse_vec3(text, attribute).ok())
        .unwrap_or(default)
}

fn vec3_string(values: &[f64; 3]) -> String {
    format!("{} {} {}", num(values[0]), num(values[1]), num(values[2]))
}

fn vec4_string(values: &[f64; 4]) -> String {
    format!(
        "{} {} {} {}",
        num(values[0]),
        num(values[1]),
        num(values[2]),
        num(values[3])
    )
}

fn num(value: f64) -> String {
    if value == value.trunc() && value.abs() < 1e15 {
        format!("{}", value as i64)
    } else {
        format!("{value}")
    }
}

fn escape_attr(value: &str) -> String {
    value
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

fn child_elements<'a, 'input>(node: Node<'a, 'input>) -> impl Iterator<Item = Node<'a, 'input>> {
    node.children().filter(|child| child.is_element())
}

fn first_child_element<'a, 'input>(node: Node<'a, 'input>, name: &str) -> Option<Node<'a, 'input>> {
    child_elements(node).find(|child| child.tag_name().name() == name)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_declared_input_size_before_parsing() {
        assert!(ensure_input_len(MJCF_MAX_INPUT_BYTES + 1).is_err());
    }

    #[test]
    fn rejects_excessive_body_nesting_without_recursing_unboundedly() {
        let mut xml = String::from("<mujoco><worldbody>");
        for index in 0..(MJCF_MAX_BODY_DEPTH + 3) {
            xml.push_str(&format!("<body name=\"b{index}\">"));
        }
        for _ in 0..(MJCF_MAX_BODY_DEPTH + 3) {
            xml.push_str("</body>");
        }
        xml.push_str("</worldbody></mujoco>");
        assert!(matches!(mjcf_to_urdf(&xml), Err(MjcfError::Invalid(_))));
    }

    #[test]
    fn converts_mesh_assets() {
        let xml = r#"
<mujoco model="mesh_model">
  <asset>
    <mesh name="part" file="meshes/part.stl" scale="0.5 0.5 0.5"/>
  </asset>
  <worldbody>
    <body name="base">
      <geom type="mesh" mesh="part"/>
    </body>
  </worldbody>
</mujoco>
"#;
        let urdf = mjcf_to_urdf(xml).expect("convert");
        assert!(urdf.contains(r#"<mesh filename="meshes/part.stl" scale="0.5 0.5 0.5"/>"#));
    }

    #[test]
    fn approximates_capsule_with_a_cylinder() {
        let xml = r#"
<mujoco model="capsule_model">
  <worldbody>
    <body name="base">
      <geom type="capsule" size="0.1 0.2"/>
    </body>
  </worldbody>
</mujoco>
"#;
        let urdf = mjcf_to_urdf(xml).expect("convert");
        assert!(urdf.contains(r#"<cylinder radius="0.1" length="0.4"/>"#));
    }

    #[test]
    fn converts_degree_euler_to_rpy() {
        let xml = r#"
<mujoco model="euler_model">
  <worldbody>
    <body name="base">
      <geom type="sphere" size="0.1"/>
      <body name="arm" pos="0 0.1 0" euler="0 0 90">
        <joint name="j" type="hinge"/>
        <geom type="sphere" size="0.05"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"#;
        let urdf = mjcf_to_urdf(xml).expect("convert");
        assert!(urdf.contains(r#"rpy="0 0 1.5707963267948966""#));
    }

    #[test]
    fn rejects_unknown_mesh_reference() {
        let xml = r#"
<mujoco model="bad_mesh">
  <worldbody>
    <body name="base">
      <geom type="mesh" mesh="missing"/>
    </body>
  </worldbody>
</mujoco>
"#;
        assert!(matches!(mjcf_to_urdf(xml), Err(MjcfError::Invalid(_))));
    }
}
