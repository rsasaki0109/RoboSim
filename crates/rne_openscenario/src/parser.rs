//! Minimal OpenSCENARIO 1.0 XML parser.

use crate::scenario::check_revision;
use crate::{
    ScenarioAction, ScenarioDocument, ScenarioEntity, ScenarioEntityKind, ScenarioError,
    ScenarioTimedAction,
};
use roxmltree::{Document, Node};
use std::path::Path;

/// Parses a minimal OpenSCENARIO 1.0 XML file into a validated scenario document.
///
/// The document's [`ScenarioDocument::source`] is set to `imported.xosc`; use
/// [`parse_openscenario_xml_with_source`] or [`parse_openscenario_xml_file`] to
/// record the real file path.
pub fn parse_openscenario_xml(text: &str) -> Result<ScenarioDocument, ScenarioError> {
    parse_openscenario_xml_with_source("imported.xosc", text)
}

/// Parses a minimal OpenSCENARIO 1.0 XML file with an explicit source path.
pub fn parse_openscenario_xml_with_source(
    source: &str,
    text: &str,
) -> Result<ScenarioDocument, ScenarioError> {
    let document = Document::parse(text)
        .map_err(|error| ScenarioError::Invalid(format!("XML syntax: {error}")))?;
    let root = document.root_element();
    if root.tag_name().name() != "OpenSCENARIO" {
        return Err(ScenarioError::Invalid(
            "root element must be `OpenSCENARIO`".to_string(),
        ));
    }

    let header = first_child_element(root, "FileHeader")
        .ok_or_else(|| ScenarioError::Invalid("missing `FileHeader`".to_string()))?;
    let rev_major = parse_u32_attribute(&header, "revMajor", "FileHeader")?;
    let rev_minor = parse_u32_attribute(&header, "revMinor", "FileHeader")?;
    check_revision(rev_major, rev_minor)?;

    let road_network_logic_file = descendant_element(root, "LogicFile")
        .and_then(|logic_file| logic_file.attribute("filepath"))
        .map(str::to_string)
        .ok_or_else(|| {
            ScenarioError::Invalid("missing `RoadNetwork/LogicFile@filepath`".to_string())
        })?;

    let mut entities = parse_entities(root)?;
    let initial_poses = parse_init_poses(root)?;
    for entity in &mut entities {
        if let Some((position, heading)) = initial_poses.get(&entity.name) {
            entity.initial_world_position_m = Some(*position);
            entity.initial_heading_rad = *heading;
        }
    }

    let actions = parse_storyboard_actions(root)?;

    let document = ScenarioDocument::new(source, road_network_logic_file, entities, actions);
    document.validate()?;
    Ok(document)
}

/// Reads an OpenSCENARIO file from disk and parses it with its path recorded.
pub fn parse_openscenario_xml_file(path: &Path) -> Result<ScenarioDocument, ScenarioError> {
    let text = std::fs::read_to_string(path)?;
    parse_openscenario_xml_with_source(&path.display().to_string(), &text)
}

fn parse_entities<'a, 'input>(
    root: Node<'a, 'input>,
) -> Result<Vec<ScenarioEntity>, ScenarioError> {
    let mut entities = Vec::new();
    for scenario_object in descendant_elements(root, "ScenarioObject") {
        let name = parse_string_attribute(&scenario_object, "name", "ScenarioObject")?;
        let kind = if first_child_element(scenario_object, "Vehicle").is_some() {
            ScenarioEntityKind::MotorVehicle
        } else if first_child_element(scenario_object, "Bicycle").is_some() {
            ScenarioEntityKind::Bicycle
        } else if first_child_element(scenario_object, "Pedestrian").is_some() {
            ScenarioEntityKind::Pedestrian
        } else {
            return Err(ScenarioError::UnsupportedElement {
                element: "ScenarioObject".to_string(),
                reason: format!(
                    "entity `{name}` must declare a Vehicle, Bicycle, or Pedestrian child"
                ),
            });
        };
        entities.push(ScenarioEntity {
            name,
            kind,
            initial_world_position_m: None,
            initial_heading_rad: None,
        });
    }
    Ok(entities)
}

/// Initial world pose decoded from a `TeleportAction` `WorldPosition`.
type InitialPose = ([f64; 3], Option<f64>);

fn parse_init_poses<'a, 'input>(
    root: Node<'a, 'input>,
) -> Result<std::collections::HashMap<String, InitialPose>, ScenarioError> {
    let mut poses = std::collections::HashMap::new();
    for private in descendant_elements(root, "Private") {
        let entity_ref = first_child_element(private, "EntityRef")
            .and_then(|entity_ref| entity_ref.attribute("entityRef"))
            .ok_or_else(|| {
                ScenarioError::Invalid(
                    "`Init` `Private` requires an `EntityRef@entityRef`".to_string(),
                )
            })?;
        for teleport in descendant_elements(private, "TeleportAction") {
            let world_position = first_child_element(teleport, "Position")
                .and_then(|position| first_child_element(position, "WorldPosition"))
                .ok_or_else(|| {
                    ScenarioError::Invalid(format!(
                        "`TeleportAction` for entity `{entity_ref}` requires a `WorldPosition`"
                    ))
                })?;
            let x = parse_f64_attribute(&world_position, "x", "WorldPosition")?;
            let y = parse_f64_attribute(&world_position, "y", "WorldPosition")?;
            let z = parse_f64_attribute(&world_position, "z", "WorldPosition")?;
            let heading = world_position
                .attribute("h")
                .map(|value| parse_f64(value, "WorldPosition@h"))
                .transpose()?;
            poses.insert(
                entity_ref.to_string(),
                ([x, y, z], heading.map(|heading| heading.to_radians())),
            );
        }
    }
    Ok(poses)
}

fn parse_storyboard_actions<'a, 'input>(
    root: Node<'a, 'input>,
) -> Result<Vec<ScenarioTimedAction>, ScenarioError> {
    let mut actions = Vec::new();
    for maneuver_group in descendant_elements(root, "ManeuverGroup") {
        let actor_refs = first_child_element(maneuver_group, "Actors")
            .map(|actors| {
                child_elements(actors)
                    .filter(|node| node.tag_name().name() == "EntityRef")
                    .filter_map(|entity_ref| entity_ref.attribute("entityRef"))
                    .map(str::to_string)
                    .collect::<Vec<_>>()
            })
            .unwrap_or_default();
        if actor_refs.len() != 1 {
            return Err(ScenarioError::UnsupportedElement {
                element: "ManeuverGroup".to_string(),
                reason: "actor sets must contain exactly one EntityRef".to_string(),
            });
        }
        let entity = actor_refs.into_iter().next().expect("exactly one actor");

        for event in descendant_elements(maneuver_group, "Event") {
            let start_time_s = descendant_element(event, "SimulationTimeCondition")
                .ok_or_else(|| {
                    ScenarioError::UnsupportedElement {
                        element: "Event".to_string(),
                        reason: format!(
                            "event for entity `{entity}` requires a `SimulationTimeCondition` start time"
                        ),
                    }
                })?
                .attribute("value")
                .map(|value| parse_f64(value, "SimulationTimeCondition@value"))
                .transpose()?
                .ok_or_else(|| {
                    ScenarioError::Invalid(
                        "`SimulationTimeCondition@value` must not be empty".to_string(),
                    )
                })?;
            let action = if let Some(speed_action) =
                descendant_element(event, "AbsoluteTargetSpeed")
            {
                let target_m_s =
                    parse_f64_attribute(&speed_action, "value", "AbsoluteTargetSpeed")?;
                ScenarioAction::AbsoluteSpeed { target_m_s }
            } else if let Some(lane_change) = descendant_element(event, "RelativeTargetLane") {
                let target_lane_offset =
                    parse_i64_attribute(&lane_change, "value", "RelativeTargetLane")?;
                ScenarioAction::LaneChange { target_lane_offset }
            } else {
                return Err(ScenarioError::UnsupportedElement {
                    element: "Event".to_string(),
                    reason: format!(
                        "event for entity `{entity}` requires an `AbsoluteTargetSpeed` or `RelativeTargetLane` action"
                    ),
                });
            };
            actions.push(ScenarioTimedAction {
                entity: entity.clone(),
                start_time_s,
                action,
            });
        }
    }
    Ok(actions)
}

fn child_elements<'a, 'input>(node: Node<'a, 'input>) -> impl Iterator<Item = Node<'a, 'input>> {
    node.children().filter(|child| child.is_element())
}

fn first_child_element<'a, 'input>(node: Node<'a, 'input>, name: &str) -> Option<Node<'a, 'input>> {
    child_elements(node).find(|child| child.tag_name().name() == name)
}

fn descendant_element<'a, 'input>(node: Node<'a, 'input>, name: &str) -> Option<Node<'a, 'input>> {
    descendant_elements(node, name).into_iter().next()
}

fn descendant_elements<'a, 'input>(node: Node<'a, 'input>, name: &str) -> Vec<Node<'a, 'input>> {
    node.descendants()
        .filter(|descendant| descendant.is_element() && descendant.tag_name().name() == name)
        .collect()
}

fn parse_string_attribute<'a, 'input>(
    node: &Node<'a, 'input>,
    attribute: &str,
    element: &str,
) -> Result<String, ScenarioError> {
    node.attribute(attribute)
        .map(str::to_string)
        .ok_or_else(|| ScenarioError::Invalid(format!("`{element}@{attribute}` must not be empty")))
}

fn parse_u32_attribute<'a, 'input>(
    node: &Node<'a, 'input>,
    attribute: &str,
    element: &str,
) -> Result<u32, ScenarioError> {
    let value = node
        .attribute(attribute)
        .ok_or_else(|| ScenarioError::Invalid(format!("missing `{element}@{attribute}`")))?;
    value.parse::<u32>().map_err(|_| {
        ScenarioError::Invalid(format!(
            "`{element}@{attribute}` must be an unsigned integer"
        ))
    })
}

fn parse_i64_attribute<'a, 'input>(
    node: &Node<'a, 'input>,
    attribute: &str,
    element: &str,
) -> Result<i64, ScenarioError> {
    let value = node
        .attribute(attribute)
        .ok_or_else(|| ScenarioError::Invalid(format!("missing `{element}@{attribute}`")))?;
    value
        .parse::<i64>()
        .map_err(|_| ScenarioError::Invalid(format!("`{element}@{attribute}` must be an integer")))
}

fn parse_f64_attribute<'a, 'input>(
    node: &Node<'a, 'input>,
    attribute: &str,
    element: &str,
) -> Result<f64, ScenarioError> {
    let value = node
        .attribute(attribute)
        .ok_or_else(|| ScenarioError::Invalid(format!("missing `{element}@{attribute}`")))?;
    parse_f64(value, &format!("{element}@{attribute}"))
}

fn parse_f64(value: &str, field: &str) -> Result<f64, ScenarioError> {
    let parsed = value
        .parse::<f64>()
        .map_err(|_| ScenarioError::Invalid(format!("`{field}` must be a finite number")))?;
    if !parsed.is_finite() {
        return Err(ScenarioError::Invalid(format!("`{field}` must be finite")));
    }
    Ok(parsed)
}
