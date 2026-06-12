//! ECS wrapper and shared entity conventions for Robot Native Engine.

#![deny(missing_docs)]

pub mod components;
pub mod entity;

pub use bevy_ecs::prelude::{Bundle, Component, Entity, Query, Res, ResMut, Resource, World};
pub use components::{Children, EntityUuid, Name, Parent};
pub use entity::spawn_named;
