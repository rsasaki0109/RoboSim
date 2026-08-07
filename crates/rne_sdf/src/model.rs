//! Minimal SDF model to URDF conversion.

use crate::SdfError;
use roxmltree::{Document, Node};
use std::path::Path;

/// Converts a minimal SDF model document into a URDF XML string.
pub fn sdf_to_urdf(text: &str) -> Result<String, SdfError> {
    let document = Document::parse(text).map_err(|error| SdfError::Xml(error.to_string()))?;
    let root = document.root_element();
    if root.tag_name().name() != "sdf" {
        return Err(SdfError::Invalid("root element must be `sdf`".to_string()));
    }
    if first_child_element(root, "world").is_some() {
        return Err(SdfError::Unsupported {
            element: "world".to_string(),
            reason: "only a top-level `<model>` is supported".to_string(),
        });
    }
    let models = child_elements(root)
        .filter(|node| node.tag_name().name() == "model")
        .collect::<Vec<_>>();
    if models.len() != 1 {
        return Err(SdfError::Invalid(format!(
            "expected exactly one `<model>`, found {}",
            models.len()
        )));
    }
    let model = models[0];
    if first_child_element(model, "pose").is_some() {
        return Err(SdfError::Unsupported {
            element: "pose".to_string(),
            reason: "model-level `<pose>` is not supported".to_string(),
        });
    }

    let robot_name = model.attribute("name").unwrap_or("model");
    let mut out = String::from("<?xml version=\"1.0\"?>\n");
    out.push_str(&format!("<robot name=\"{}\">\n", escape_attr(robot_name)));
    for link in child_elements(model).filter(|node| node.tag_name().name() == "link") {
        out.push_str(&render_link(link)?);
    }
    for joint in child_elements(model).filter(|node| node.tag_name().name() == "joint") {
        out.push_str(&render_joint(joint)?);
    }
    out.push_str("</robot>\n");
    Ok(out)
}

/// Reads an SDF model file and converts it, keeping the model name.
pub fn sdf_to_urdf_file(path: &Path) -> Result<String, SdfError> {
    let text = std::fs::read_to_string(path)?;
    sdf_to_urdf(&text)
}

fn render_link(link: Node<'_, '_>) -> Result<String, SdfError> {
    let name = required_attr(&link, "link", "name")?;
    if first_child_element(link, "pose").is_some() {
        return Err(SdfError::Unsupported {
            element: "pose".to_string(),
            reason: format!("link `{name}` `<pose>` is not supported"),
        });
    }
    let mut out = String::new();
    out.push_str(&format!("  <link name=\"{}\">\n", escape_attr(name)));
    if let Some(inertial) = first_child_element(link, "inertial") {
        let mass = first_child_element(inertial, "mass")
            .and_then(|mass| mass.attribute("value"))
            .map(|value| parse_scalar(value, "inertial/mass@value"))
            .transpose()?
            .ok_or_else(|| {
                SdfError::Invalid(format!("link `{name}` inertial requires `<mass value>`"))
            })?;
        let inertia = first_child_element(inertial, "inertia").ok_or_else(|| {
            SdfError::Invalid(format!("link `{name}` inertial requires `<inertia>`"))
        })?;
        let mut inertia_values = [0.0; 6];
        for (index, attribute) in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]
            .into_iter()
            .enumerate()
        {
            let value = inertia.attribute(attribute).ok_or_else(|| {
                SdfError::Invalid(format!("link `{name}` inertia@{attribute} is required"))
            })?;
            inertia_values[index] = parse_scalar(value, &format!("inertia@{attribute}"))?;
        }
        let [ixx, ixy, ixz, iyy, iyz, izz] = inertia_values;
        out.push_str("    <inertial>\n");
        out.push_str(&format!("      <mass value=\"{}\"/>\n", num(mass)));
        out.push_str(&format!(
            "      <inertia ixx=\"{}\" ixy=\"{}\" ixz=\"{}\" iyy=\"{}\" iyz=\"{}\" izz=\"{}\"/>\n",
            num(ixx),
            num(ixy),
            num(ixz),
            num(iyy),
            num(iyz),
            num(izz)
        ));
        out.push_str("    </inertial>\n");
    }
    for visual in child_elements(link).filter(|node| node.tag_name().name() == "visual") {
        out.push_str(&render_visual(visual)?);
    }
    for collision in child_elements(link).filter(|node| node.tag_name().name() == "collision") {
        out.push_str(&render_collision(collision)?);
    }
    out.push_str("  </link>\n");
    Ok(out)
}

fn render_visual(visual: Node<'_, '_>) -> Result<String, SdfError> {
    let mut out = String::from("    <visual>\n");
    out.push_str(&format!("      {}\n", render_origin(visual)));
    out.push_str(&format!("      {}\n", render_geometry(visual)?));
    if let Some(material) = first_child_element(visual, "material") {
        out.push_str(&format!("      {}\n", render_material(material)?));
    }
    out.push_str("    </visual>\n");
    Ok(out)
}

fn render_collision(collision: Node<'_, '_>) -> Result<String, SdfError> {
    let mut out = String::from("    <collision>\n");
    out.push_str(&format!("      {}\n", render_origin(collision)));
    out.push_str(&format!("      {}\n", render_geometry(collision)?));
    out.push_str("    </collision>\n");
    Ok(out)
}

fn render_origin(node: Node<'_, '_>) -> String {
    let (xyz, rpy) = if let Some(pose) = first_child_element(node, "pose") {
        let values = pose
            .text()
            .unwrap_or_default()
            .split_whitespace()
            .filter_map(|value| value.parse::<f64>().ok())
            .collect::<Vec<_>>();
        if values.len() == 6 {
            (
                [values[0], values[1], values[2]],
                [values[3], values[4], values[5]],
            )
        } else {
            ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        }
    } else {
        (
            vec_attr(node, "xyz", [0.0, 0.0, 0.0]),
            vec_attr(node, "rpy", [0.0, 0.0, 0.0]),
        )
    };
    format!(
        "<origin xyz=\"{}\" rpy=\"{}\"/>",
        vec3_string(&xyz),
        vec3_string(&rpy)
    )
}

fn render_geometry(node: Node<'_, '_>) -> Result<String, SdfError> {
    let geometry = first_child_element(node, "geometry").ok_or_else(|| {
        SdfError::Invalid("`visual`/`collision` requires a `<geometry>`".to_string())
    })?;
    if let Some(box_node) = first_child_element(geometry, "box") {
        let size = first_child_element(box_node, "size")
            .and_then(|size| size.text())
            .ok_or_else(|| SdfError::Invalid("`<box>` requires a `<size>`".to_string()))?;
        let size = parse_vec3(size, "box/size")?;
        return Ok(format!(
            "<geometry><box size=\"{}\"/></geometry>",
            vec3_string(&size)
        ));
    }
    if let Some(sphere) = first_child_element(geometry, "sphere") {
        let radius = first_child_element(sphere, "radius")
            .and_then(|radius| radius.text())
            .map(|value| parse_scalar(value, "sphere/radius"))
            .transpose()?
            .ok_or_else(|| SdfError::Invalid("`<sphere>` requires a `<radius>`".to_string()))?;
        return Ok(format!(
            "<geometry><sphere radius=\"{}\"/></geometry>",
            num(radius)
        ));
    }
    if let Some(cylinder) = first_child_element(geometry, "cylinder") {
        let radius = first_child_element(cylinder, "radius")
            .and_then(|radius| radius.text())
            .map(|value| parse_scalar(value, "cylinder/radius"))
            .transpose()?
            .ok_or_else(|| SdfError::Invalid("`<cylinder>` requires a `<radius>`".to_string()))?;
        let length = first_child_element(cylinder, "length")
            .and_then(|length| length.text())
            .map(|value| parse_scalar(value, "cylinder/length"))
            .transpose()?
            .ok_or_else(|| SdfError::Invalid("`<cylinder>` requires a `<length>`".to_string()))?;
        return Ok(format!(
            "<geometry><cylinder radius=\"{}\" length=\"{}\"/></geometry>",
            num(radius),
            num(length)
        ));
    }
    if let Some(mesh) = first_child_element(geometry, "mesh") {
        let uri = first_child_element(mesh, "uri")
            .and_then(|uri| uri.text())
            .ok_or_else(|| SdfError::Invalid("`<mesh>` requires a `<uri>`".to_string()))?;
        return Ok(format!(
            "<geometry><mesh filename=\"{}\"/></geometry>",
            escape_attr(uri.trim())
        ));
    }
    Err(SdfError::Unsupported {
        element: "geometry".to_string(),
        reason: "only box, sphere, cylinder, and mesh geometry are supported".to_string(),
    })
}

fn render_material(material: Node<'_, '_>) -> Result<String, SdfError> {
    let color = ["diffuse", "ambient"].into_iter().find_map(|name| {
        first_child_element(material, name)
            .and_then(|node| node.text())
            .map(|text| (name, text))
    });
    let Some((_, text)) = color else {
        return Ok(String::new());
    };
    let rgba = parse_vec4(text, "material color")?;
    Ok(format!(
        "<material name=\"material\"><color rgba=\"{}\"/></material>",
        vec4_string(&rgba)
    ))
}

fn render_joint(joint: Node<'_, '_>) -> Result<String, SdfError> {
    let name = required_attr(&joint, "joint", "name")?;
    let joint_type = required_attr(&joint, "joint", "type")?;
    let urdf_type = match joint_type {
        "revolute" => "revolute",
        "continuous" => "continuous",
        "prismatic" => "prismatic",
        "fixed" => "fixed",
        other => {
            return Err(SdfError::Unsupported {
                element: "joint".to_string(),
                reason: format!("joint type `{other}` is not supported"),
            })
        }
    };
    let parent = first_child_element(joint, "parent")
        .and_then(|parent| parent.attribute("link"))
        .ok_or_else(|| SdfError::Invalid(format!("joint `{name}` requires `<parent link>`")))?;
    let child = first_child_element(joint, "child")
        .and_then(|child| child.attribute("link"))
        .ok_or_else(|| SdfError::Invalid(format!("joint `{name}` requires `<child link>`")))?;

    let mut out = String::new();
    out.push_str(&format!(
        "  <joint name=\"{}\" type=\"{}\">\n",
        escape_attr(name),
        urdf_type
    ));
    out.push_str(&format!(
        "    <parent link=\"{}\"/>\n    <child link=\"{}\"/>\n",
        escape_attr(parent),
        escape_attr(child)
    ));
    out.push_str(&format!("    {}\n", render_origin(joint)));
    if urdf_type != "fixed" {
        let axis = first_child_element(joint, "axis")
            .and_then(|axis| axis.attribute("xyz"))
            .ok_or_else(|| {
                SdfError::Invalid(format!(
                    "joint `{name}` requires `<axis xyz>` for type `{joint_type}`"
                ))
            })?;
        let axis = parse_vec3(axis, "axis@xyz")?;
        out.push_str(&format!("    <axis xyz=\"{}\"/>\n", vec3_string(&axis)));
        let limit = first_child_element(joint, "limit")
            .ok_or_else(|| SdfError::Invalid(format!("joint `{name}` requires a `<limit>`")))?;
        let lower = limit
            .attribute("lower")
            .map(|value| parse_scalar(value, "limit@lower"))
            .transpose()?
            .unwrap_or(0.0);
        let upper = limit
            .attribute("upper")
            .map(|value| parse_scalar(value, "limit@upper"))
            .transpose()?
            .unwrap_or(0.0);
        let effort = limit
            .attribute("effort")
            .map(|value| parse_scalar(value, "limit@effort"))
            .transpose()?
            .unwrap_or(0.0);
        let velocity = limit
            .attribute("velocity")
            .map(|value| parse_scalar(value, "limit@velocity"))
            .transpose()?
            .unwrap_or(0.0);
        out.push_str(&format!(
            "    <limit lower=\"{}\" upper=\"{}\" effort=\"{}\" velocity=\"{}\"/>\n",
            num(lower),
            num(upper),
            num(effort),
            num(velocity)
        ));
    }
    out.push_str("  </joint>\n");
    Ok(out)
}

fn required_attr<'a, 'input>(
    node: &Node<'a, 'input>,
    element: &str,
    attribute: &str,
) -> Result<&'a str, SdfError> {
    node.attribute(attribute)
        .ok_or_else(|| SdfError::Invalid(format!("`{element}` requires `@{attribute}`")))
}

fn parse_scalar(text: &str, field: &str) -> Result<f64, SdfError> {
    let value = text
        .trim()
        .parse::<f64>()
        .map_err(|_| SdfError::Invalid(format!("`{field}` must be a number")))?;
    if !value.is_finite() {
        return Err(SdfError::Invalid(format!("`{field}` must be finite")));
    }
    Ok(value)
}

fn parse_vec3(text: &str, field: &str) -> Result<[f64; 3], SdfError> {
    let values = text
        .split_whitespace()
        .map(|value| parse_scalar(value, field))
        .collect::<Result<Vec<_>, _>>()?;
    if values.len() != 3 {
        return Err(SdfError::Invalid(format!(
            "`{field}` must contain exactly three numbers"
        )));
    }
    Ok([values[0], values[1], values[2]])
}

fn parse_vec4(text: &str, field: &str) -> Result<[f64; 4], SdfError> {
    let values = text
        .split_whitespace()
        .map(|value| parse_scalar(value, field))
        .collect::<Result<Vec<_>, _>>()?;
    if values.len() != 4 {
        return Err(SdfError::Invalid(format!(
            "`{field}` must contain exactly four numbers"
        )));
    }
    Ok([values[0], values[1], values[2], values[3]])
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
