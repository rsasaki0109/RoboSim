//! Link-attached device helpers.
//!
//! This is the RNE analogue of Choreonoid's `Device` / `DeviceList` pair:
//! sensors, actuators, and controllers are regular ECS entities that point back
//! at a host link, while the link keeps an indexed [`LinkDevices`] list for
//! deterministic enumeration. The abstraction is intentionally free of any
//! concrete sensor or actuator type so that `rne_robot` stays independent of
//! `rne_sensor`.

use crate::components::{Device, DeviceKind, Link, LinkDevices};
use bevy_ecs::prelude::World;
use rne_ecs::{spawn_named, Entity};
use thiserror::Error;

/// Error returned by device attachment helpers.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Error)]
pub enum DeviceError {
    /// The target entity does not have a [`Link`] component.
    #[error("entity {0:?} is not a link")]
    NotALink(Entity),
    /// The entity is not a device because it lacks a [`Device`] component.
    #[error("entity {0:?} is not a device")]
    NotADevice(Entity),
    /// The device name was empty.
    #[error("device name must not be empty")]
    EmptyName,
}

/// Spawns a new device attached to `link`.
pub fn spawn_device(
    world: &mut World,
    link: Entity,
    name: impl Into<String>,
    kind: DeviceKind,
) -> Result<Entity, DeviceError> {
    if world.get::<Link>(link).is_none() {
        return Err(DeviceError::NotALink(link));
    }
    let name = name.into();
    if name.is_empty() {
        return Err(DeviceError::EmptyName);
    }
    let device = spawn_named(world, name.clone());
    world.entity_mut(device).insert(Device { link, name, kind });
    index_device(world, link, device);
    Ok(device)
}

/// Attaches an existing device entity to `link`.
///
/// The device entity must already carry a [`Device`] component; its `link`
/// field is rewritten to `link`.
pub fn attach_device(world: &mut World, link: Entity, device: Entity) -> Result<(), DeviceError> {
    let Some(existing) = world.get::<Device>(device).cloned() else {
        return Err(DeviceError::NotADevice(device));
    };
    if world.get::<Link>(link).is_none() {
        return Err(DeviceError::NotALink(link));
    }
    world.entity_mut(device).insert(Device { link, ..existing });
    index_device(world, link, device);
    Ok(())
}

/// Removes a device from its host link's index.
///
/// The device entity and its [`Device`] component are left in place so callers
/// can decide whether to despawn or re-attach it.
pub fn detach_device(world: &mut World, device: Entity) -> Option<Entity> {
    let link = world.get::<Device>(device)?.link;
    if let Some(mut devices) = world.get_mut::<LinkDevices>(link) {
        devices.0.retain(|candidate| *candidate != device);
    }
    Some(link)
}

/// Returns the host link of a device, if it is attached.
pub fn device_link(world: &World, device: Entity) -> Option<Entity> {
    world.get::<Device>(device).map(|device| device.link)
}

/// Returns the device entities currently indexed on `link`.
pub fn devices_of_link(world: &World, link: Entity) -> Vec<Entity> {
    world
        .get::<LinkDevices>(link)
        .map(|devices| devices.0.clone())
        .unwrap_or_default()
}

/// Returns the devices on `link` of a specific category.
pub fn devices_of_kind(world: &World, link: Entity, kind: DeviceKind) -> Vec<Entity> {
    devices_of_link(world, link)
        .into_iter()
        .filter(|device| world.get::<Device>(*device).map(|d| d.kind) == Some(kind))
        .collect()
}

fn index_device(world: &mut World, link: Entity, device: Entity) {
    let already_indexed = world
        .get::<LinkDevices>(link)
        .map(|devices| devices.0.contains(&device))
        .unwrap_or(false);
    if already_indexed {
        return;
    }
    if let Some(mut devices) = world.get_mut::<LinkDevices>(link) {
        devices.0.push(device);
    } else {
        world.entity_mut(link).insert(LinkDevices(vec![device]));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::Link;
    use rne_ecs::spawn_named;

    fn link_world() -> (World, Entity) {
        let mut world = World::new();
        let robot = spawn_named(&mut world, "robot");
        let link = spawn_named(&mut world, "base_link");
        world.entity_mut(link).insert(Link {
            robot,
            name: "base_link".into(),
        });
        (world, link)
    }

    #[test]
    fn spawn_device_indexes_on_link() {
        let (mut world, link) = link_world();
        let imu = spawn_device(&mut world, link, "imu", DeviceKind::Sensor).unwrap();
        let motor = spawn_device(&mut world, link, "motor", DeviceKind::Actuator).unwrap();

        assert_eq!(device_link(&world, imu), Some(link));
        assert_eq!(devices_of_link(&world, link), vec![imu, motor]);
        assert_eq!(devices_of_kind(&world, link, DeviceKind::Sensor), vec![imu]);
    }

    #[test]
    fn detach_removes_from_index_but_keeps_entity() {
        let (mut world, link) = link_world();
        let device = spawn_device(&mut world, link, "camera", DeviceKind::Sensor).unwrap();
        assert_eq!(detach_device(&mut world, device), Some(link));
        assert!(devices_of_link(&world, link).is_empty());
        assert!(world.get::<Device>(device).is_some());
    }

    #[test]
    fn attach_rejects_non_link_and_non_device() {
        let (mut world, link) = link_world();
        let plain = spawn_named(&mut world, "plain");
        assert_eq!(
            attach_device(&mut world, link, plain),
            Err(DeviceError::NotADevice(plain))
        );
        assert_eq!(
            spawn_device(&mut world, plain, "imu", DeviceKind::Sensor),
            Err(DeviceError::NotALink(plain))
        );
    }
}
