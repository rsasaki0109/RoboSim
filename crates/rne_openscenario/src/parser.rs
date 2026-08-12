//! Minimal OpenSCENARIO 1.0 XML parser.

use crate::scenario::{
    check_revision, ensure_scenario_input_len, read_bounded_utf8, read_bounded_utf8_with_limit,
};
use crate::{
    ScenarioAction, ScenarioDocument, ScenarioEntity, ScenarioEntityKind, ScenarioError,
    ScenarioTimedAction,
};
use roxmltree::{Document, Node};
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Component, Path, PathBuf};

const OPENSCENARIO_MAX_CATALOG_DIRECTORIES: usize = 64;
const OPENSCENARIO_MAX_CATALOG_FILES: usize = 1_024;
const OPENSCENARIO_MAX_CATALOG_FILE_BYTES: usize = 8 * 1024 * 1024;
const OPENSCENARIO_MAX_CATALOG_BYTES: usize = 64 * 1024 * 1024;

#[derive(Default)]
struct CatalogBudget {
    files: usize,
    bytes: usize,
}

/// Parses a minimal OpenSCENARIO 1.0 XML file into a validated scenario document.
///
/// The document's [`ScenarioDocument::source`] is set to `imported.xosc`; use
/// [`parse_openscenario_xml_with_source`] or [`parse_openscenario_xml_file`] to
/// record the real file path.
pub fn parse_openscenario_xml(text: &str) -> Result<ScenarioDocument, ScenarioError> {
    parse_openscenario_xml_with_source("imported.xosc", text)
}

/// Parses a minimal OpenSCENARIO 1.0 XML file with an explicit source path.
///
/// Vehicle `CatalogReference` entities are only resolvable when a base
/// directory is provided (see [`parse_openscenario_xml_file`]); without one, a
/// catalog reference is rejected.
pub fn parse_openscenario_xml_with_source(
    source: &str,
    text: &str,
) -> Result<ScenarioDocument, ScenarioError> {
    parse_inner(source, text, None)
}

/// Parses a minimal OpenSCENARIO 1.0 XML file with an explicit source path and
/// base directory used to resolve `CatalogLocations`.
pub fn parse_openscenario_xml_with_source_at(
    source: &str,
    text: &str,
    base_dir: &Path,
) -> Result<ScenarioDocument, ScenarioError> {
    parse_inner(source, text, Some(base_dir))
}

fn parse_inner(
    source: &str,
    text: &str,
    base_dir: Option<&Path>,
) -> Result<ScenarioDocument, ScenarioError> {
    ensure_scenario_input_len(text.len())?;
    let parameters = extract_parameters(text)?;
    let text = substitute_parameters(text, &parameters)?;
    let document = Document::parse(&text)
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

    let mut entities = parse_entities(root, base_dir)?;
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
///
/// Vehicle catalog directories in `CatalogLocations` are resolved relative to
/// the file's directory.
pub fn parse_openscenario_xml_file(path: &Path) -> Result<ScenarioDocument, ScenarioError> {
    let text = read_bounded_utf8(path)?;
    parse_openscenario_xml_with_source_at(
        &path.display().to_string(),
        &text,
        path.parent().unwrap_or_else(|| Path::new(".")),
    )
}

fn parse_entities<'a, 'input>(
    root: Node<'a, 'input>,
    base_dir: Option<&Path>,
) -> Result<Vec<ScenarioEntity>, ScenarioError> {
    let catalog_dirs = catalog_directories(root)?;
    let mut entities = Vec::new();
    let mut catalog_budget = CatalogBudget::default();
    let mut catalog_cache = BTreeMap::new();
    for scenario_object in descendant_elements(root, "ScenarioObject") {
        let name = parse_string_attribute(&scenario_object, "name", "ScenarioObject")?;
        let kind = if first_child_element(scenario_object, "Vehicle").is_some() {
            ScenarioEntityKind::MotorVehicle
        } else if first_child_element(scenario_object, "Bicycle").is_some() {
            ScenarioEntityKind::Bicycle
        } else if first_child_element(scenario_object, "Pedestrian").is_some() {
            ScenarioEntityKind::Pedestrian
        } else if let Some(catalog_reference) =
            first_child_element(scenario_object, "CatalogReference")
        {
            let catalog_name = catalog_reference.attribute("catalogName").ok_or_else(|| {
                ScenarioError::Invalid("`CatalogReference` requires `@catalogName`".to_string())
            })?;
            let entry_name = catalog_reference.attribute("entryName").ok_or_else(|| {
                ScenarioError::Invalid("`CatalogReference` requires `@entryName`".to_string())
            })?;
            let cache_key = (catalog_name.to_string(), entry_name.to_string());
            if let Some(kind) = catalog_cache.get(&cache_key) {
                *kind
            } else {
                let kind = resolve_catalog_entity(
                    catalog_name,
                    entry_name,
                    &catalog_dirs,
                    base_dir,
                    &mut catalog_budget,
                )?;
                catalog_cache.insert(cache_key, kind);
                kind
            }
        } else {
            return Err(ScenarioError::UnsupportedElement {
                element: "ScenarioObject".to_string(),
                reason: format!(
                    "entity `{name}` must declare a Vehicle, Bicycle, or Pedestrian child or a CatalogReference"
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

/// Collects the `CatalogLocations` directory paths in document order.
fn catalog_directories(root: Node<'_, '_>) -> Result<Vec<PathBuf>, ScenarioError> {
    let directories = descendant_elements(root, "Directory")
        .into_iter()
        .filter_map(|directory| directory.attribute("path"))
        .map(PathBuf::from)
        .collect::<Vec<_>>();
    if directories.len() > OPENSCENARIO_MAX_CATALOG_DIRECTORIES {
        return Err(ScenarioError::Invalid(format!(
            "CatalogLocations contains {} directories, limit is {OPENSCENARIO_MAX_CATALOG_DIRECTORIES}",
            directories.len()
        )));
    }
    for directory in &directories {
        validate_catalog_relative_path(directory)?;
    }
    Ok(directories)
}

fn validate_catalog_relative_path(path: &Path) -> Result<(), ScenarioError> {
    if path.as_os_str().is_empty()
        || path.is_absolute()
        || path.components().any(|component| {
            matches!(
                component,
                Component::ParentDir | Component::RootDir | Component::Prefix(_)
            )
        })
    {
        return Err(ScenarioError::Invalid(format!(
            "catalog directory must stay below the scenario directory: {}",
            path.display()
        )));
    }
    Ok(())
}

/// Resolves a `CatalogReference` entity kind by scanning the catalog files.
fn resolve_catalog_entity(
    catalog_name: &str,
    entry_name: &str,
    catalog_dirs: &[std::path::PathBuf],
    base_dir: Option<&Path>,
    budget: &mut CatalogBudget,
) -> Result<ScenarioEntityKind, ScenarioError> {
    if catalog_name != "VehicleCatalog" {
        return Err(ScenarioError::UnsupportedElement {
            element: "CatalogReference".to_string(),
            reason: format!("catalog `{catalog_name}` is not supported (only VehicleCatalog)"),
        });
    }
    let Some(base_dir) = base_dir else {
        return Err(ScenarioError::Invalid(
            "catalog resolution requires a base directory".to_string(),
        ));
    };
    let canonical_base = base_dir.canonicalize().map_err(|error| {
        ScenarioError::Invalid(format!(
            "resolve scenario directory {}: {error}",
            base_dir.display()
        ))
    })?;
    if budget.files >= OPENSCENARIO_MAX_CATALOG_FILES
        || budget.bytes >= OPENSCENARIO_MAX_CATALOG_BYTES
    {
        return Err(ScenarioError::Invalid(
            "OpenSCENARIO catalog scan budget is exhausted".to_string(),
        ));
    }
    for directory in catalog_dirs {
        validate_catalog_relative_path(directory)?;
        let directory_path = base_dir.join(directory);
        let canonical_directory = directory_path.canonicalize().map_err(|error| {
            ScenarioError::Invalid(format!(
                "resolve catalog directory {}: {error}",
                directory_path.display()
            ))
        })?;
        if !canonical_directory.starts_with(&canonical_base) {
            return Err(ScenarioError::Invalid(format!(
                "catalog directory escapes scenario directory: {}",
                directory_path.display()
            )));
        }
        let mut files = std::fs::read_dir(&canonical_directory)
            .map_err(|error| {
                ScenarioError::Invalid(format!(
                    "read catalog directory {}: {error}",
                    canonical_directory.display()
                ))
            })?
            .filter_map(Result::ok)
            .filter_map(|entry| {
                let path = entry.path();
                path.extension()
                    .is_some_and(|extension| extension == "xosc" || extension == "xml")
                    .then_some(path)
            })
            .take(OPENSCENARIO_MAX_CATALOG_FILES.saturating_sub(budget.files) + 1)
            .collect::<Vec<_>>();
        if files.len() > OPENSCENARIO_MAX_CATALOG_FILES.saturating_sub(budget.files) {
            return Err(ScenarioError::Invalid(format!(
                "catalog scan exceeds {OPENSCENARIO_MAX_CATALOG_FILES} XML files at {}",
                canonical_directory.display()
            )));
        }
        files.sort();
        for file in files {
            let canonical_file = file.canonicalize().map_err(|error| {
                ScenarioError::Invalid(format!("resolve catalog file {}: {error}", file.display()))
            })?;
            if !canonical_file.starts_with(&canonical_directory) {
                return Err(ScenarioError::Invalid(format!(
                    "catalog file escapes catalog directory: {}",
                    file.display()
                )));
            }
            let file_bytes =
                usize::try_from(canonical_file.metadata().map_err(ScenarioError::Io)?.len())
                    .unwrap_or(usize::MAX);
            if file_bytes > OPENSCENARIO_MAX_CATALOG_FILE_BYTES {
                return Err(ScenarioError::Invalid(format!(
                    "catalog file is {file_bytes} bytes, per-file limit is {OPENSCENARIO_MAX_CATALOG_FILE_BYTES}: {}",
                    canonical_file.display()
                )));
            }
            let next_total = budget.bytes.saturating_add(file_bytes);
            if next_total > OPENSCENARIO_MAX_CATALOG_BYTES {
                return Err(ScenarioError::Invalid(format!(
                    "catalog scan exceeds {OPENSCENARIO_MAX_CATALOG_BYTES} total bytes"
                )));
            }
            budget.files += 1;
            budget.bytes = next_total;
            let text =
                read_bounded_utf8_with_limit(&canonical_file, OPENSCENARIO_MAX_CATALOG_FILE_BYTES)?;
            let document = Document::parse(&text)
                .map_err(|error| ScenarioError::Invalid(format!("XML syntax: {error}")))?;
            let found = descendant_elements(document.root_element(), "Vehicle")
                .into_iter()
                .any(|vehicle| vehicle.attribute("name") == Some(entry_name));
            if found {
                return Ok(ScenarioEntityKind::MotorVehicle);
            }
        }
    }
    Err(ScenarioError::Invalid(format!(
        "catalog entry `{entry_name}` not found in VehicleCatalog"
    )))
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
        if actor_refs.is_empty() {
            return Err(ScenarioError::UnsupportedElement {
                element: "ManeuverGroup".to_string(),
                reason: "actor sets must contain at least one EntityRef".to_string(),
            });
        }
        let unique_actor_refs = actor_refs.iter().cloned().collect::<BTreeSet<_>>();
        if unique_actor_refs.len() != actor_refs.len() {
            return Err(ScenarioError::Invalid(
                "ManeuverGroup actor sets must not contain duplicate EntityRef values".to_string(),
            ));
        }
        let actor_refs = unique_actor_refs.into_iter().collect::<Vec<_>>();
        let actor_names = actor_refs.join(", ");

        for event in descendant_elements(maneuver_group, "Event") {
            let start_time_s = descendant_element(event, "SimulationTimeCondition")
                .ok_or_else(|| {
                    ScenarioError::UnsupportedElement {
                        element: "Event".to_string(),
                        reason: format!(
                            "event for actors `{actor_names}` requires a `SimulationTimeCondition` start time"
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
            } else if descendant_element(event, "AssignRouteAction").is_some() {
                let waypoints = parse_assigned_route(event)?;
                ScenarioAction::AssignRoute { waypoints }
            } else {
                return Err(ScenarioError::UnsupportedElement {
                    element: "Event".to_string(),
                    reason: format!(
                        "event for actors `{actor_names}` requires an `AbsoluteTargetSpeed`, `RelativeTargetLane`, or `AssignRouteAction` action"
                    ),
                });
            };
            actions.extend(actor_refs.iter().map(|entity| ScenarioTimedAction {
                entity: entity.clone(),
                start_time_s,
                action: action.clone(),
            }));
        }
    }
    Ok(actions)
}

fn parse_assigned_route(event: Node<'_, '_>) -> Result<Vec<[f64; 3]>, ScenarioError> {
    let route = descendant_element(event, "Route").ok_or_else(|| {
        ScenarioError::Invalid("`AssignRouteAction` requires a `<Route>`".to_string())
    })?;
    let mut waypoints = Vec::new();
    for waypoint in descendant_elements(route, "Waypoint") {
        let world_position = descendant_element(waypoint, "WorldPosition").ok_or_else(|| {
            ScenarioError::Invalid(format!(
                "`Waypoint` requires a `WorldPosition` (route has {} waypoints so far)",
                waypoints.len()
            ))
        })?;
        let x = parse_f64_attribute(&world_position, "x", "WorldPosition")?;
        let y = parse_f64_attribute(&world_position, "y", "WorldPosition")?;
        let z = parse_f64_attribute(&world_position, "z", "WorldPosition")?;
        waypoints.push([x, y, z]);
    }
    if waypoints.len() < 2 {
        return Err(ScenarioError::Invalid(
            "`AssignRouteAction` route requires at least two waypoints".to_string(),
        ));
    }
    Ok(waypoints)
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

/// Reads `ParameterDeclarations` into a name → value map.
///
/// Parameter values are substituted into `${name}` references before the
/// document is parsed, so declared values must be plain XML attribute tokens
/// (numbers for the numeric subset this importer accepts).
fn extract_parameters(text: &str) -> Result<BTreeMap<String, String>, ScenarioError> {
    let document = Document::parse(text)
        .map_err(|error| ScenarioError::Invalid(format!("XML syntax: {error}")))?;
    let mut parameters = BTreeMap::new();
    for declaration in descendant_elements(document.root_element(), "ParameterDeclaration") {
        let name = declaration.attribute("name").ok_or_else(|| {
            ScenarioError::Invalid("`ParameterDeclaration` requires `@name`".to_string())
        })?;
        let value = declaration.attribute("value").ok_or_else(|| {
            ScenarioError::Invalid("`ParameterDeclaration` requires `@value`".to_string())
        })?;
        if parameters
            .insert(name.to_string(), value.to_string())
            .is_some()
        {
            return Err(ScenarioError::Invalid(format!(
                "duplicate parameter declaration `{name}`"
            )));
        }
    }
    Ok(parameters)
}

/// Replaces every `$ {name}` reference with its declared value.
fn substitute_parameters(
    text: &str,
    parameters: &BTreeMap<String, String>,
) -> Result<String, ScenarioError> {
    substitute_parameters_with_limit(text, parameters, crate::SCENARIO_MAX_INPUT_BYTES)
}

fn substitute_parameters_with_limit(
    text: &str,
    parameters: &BTreeMap<String, String>,
    limit: usize,
) -> Result<String, ScenarioError> {
    let mut out = String::with_capacity(text.len().min(limit));
    let mut remaining = text;
    while let Some(start) = remaining.find("${") {
        push_bounded(&mut out, &remaining[..start], limit)?;
        let reference = &remaining[start + 2..];
        let Some(end) = reference.find('}') else {
            return Err(ScenarioError::Invalid(
                "unterminated parameter reference".to_string(),
            ));
        };
        let name = &reference[..end];
        let value = parameters.get(name).ok_or_else(|| {
            ScenarioError::Invalid(format!("unknown parameter reference `{name}`"))
        })?;
        push_bounded(&mut out, value, limit)?;
        remaining = &reference[end + 1..];
    }
    push_bounded(&mut out, remaining, limit)?;
    Ok(out)
}

fn push_bounded(out: &mut String, value: &str, limit: usize) -> Result<(), ScenarioError> {
    if out.len().saturating_add(value.len()) > limit {
        return Err(ScenarioError::Invalid(format!(
            "parameter substitution exceeds {limit} bytes"
        )));
    }
    out.push_str(value);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_catalog_paths_that_can_escape_the_scenario_directory() {
        assert!(validate_catalog_relative_path(Path::new("catalogs/vehicles")).is_ok());
        assert!(validate_catalog_relative_path(Path::new("../outside")).is_err());
        assert!(validate_catalog_relative_path(Path::new("/absolute")).is_err());
    }

    #[test]
    fn parameter_substitution_is_single_pass_and_bounded() {
        let parameters = BTreeMap::from([
            ("a".to_string(), "${b}".to_string()),
            ("b".to_string(), "123".to_string()),
        ]);
        assert_eq!(
            substitute_parameters_with_limit("x=${a};y=${b}", &parameters, 32).unwrap(),
            "x=${b};y=123"
        );
        assert!(substitute_parameters_with_limit("${b}${b}", &parameters, 5).is_err());
        assert!(substitute_parameters_with_limit("${missing}", &parameters, 32).is_err());
    }

    #[test]
    fn rejects_declared_scenario_size_before_parsing() {
        assert!(ensure_scenario_input_len(crate::SCENARIO_MAX_INPUT_BYTES + 1).is_err());
    }
}
