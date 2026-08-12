//! Minimal MuJoCo MJCF model to URDF conversion.

use crate::MjcfError;
use roxmltree::{Document, Node};
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

    let model_name = root.attribute("model").unwrap_or("model");
    let mut out = String::from("<?xml version=\"1.0\"?>\n");
    out.push_str(&format!("<robot name=\"{}\">\n", escape_attr(model_name)));
    render_body(root_body, None, angle, 0, &mut out)?;
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
    depth: usize,
    out: &mut String,
) -> Result<(), MjcfError> {
    if depth > MJCF_MAX_BODY_DEPTH {
        return Err(MjcfError::Invalid(format!(
            "body nesting exceeds {MJCF_MAX_BODY_DEPTH} levels"
        )));
    }
    reject_body_rotation(body)?;
    let name = required_attr(&body, "body", "name")?;
    if let Some(parent_link) = parent_link {
        render_joint(body, parent_link, name, angle, out)?;
    }
    render_link(body, name, out)?;
    if out.len() > MJCF_MAX_OUTPUT_BYTES {
        return Err(MjcfError::Invalid(format!(
            "converted URDF exceeds {MJCF_MAX_OUTPUT_BYTES} bytes"
        )));
    }
    for child in child_elements(body).filter(|node| node.tag_name().name() == "body") {
        render_body(child, Some(name), angle, depth + 1, out)?;
    }
    Ok(())
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
    out.push_str(&format!(
        "    <origin xyz=\"{}\" rpy=\"0 0 0\"/>\n",
        vec3_string(&pos)
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

fn render_link(body: Node<'_, '_>, name: &str, out: &mut String) -> Result<(), MjcfError> {
    out.push_str(&format!("  <link name=\"{}\">\n", escape_attr(name)));
    let mut has_geom = false;
    for geom in child_elements(body).filter(|node| node.tag_name().name() == "geom") {
        for attribute in ["quat", "euler", "zaxis"] {
            if geom.attribute(attribute).is_some() {
                return Err(MjcfError::Unsupported {
                    element: "geom".to_string(),
                    reason: format!("geom `@{attribute}` rotation is not supported"),
                });
            }
        }
        let pos = vec_attr(geom, "pos", [0.0, 0.0, 0.0]);
        let geometry = render_geom_geometry(&geom)?;
        out.push_str("    <visual>\n");
        out.push_str(&format!(
            "      <origin xyz=\"{}\" rpy=\"0 0 0\"/>\n",
            vec3_string(&pos)
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
            "      <origin xyz=\"{}\" rpy=\"0 0 0\"/>\n",
            vec3_string(&pos)
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

fn render_geom_geometry(geom: &Node<'_, '_>) -> Result<String, MjcfError> {
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
}
