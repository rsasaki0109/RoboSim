//! Minimal Robot Native Engine bootstrap example.

use bevy_ecs::world::World;
use rne_core::{AppBuilder, Plugin, RneApp, SchedulePhase, SimDuration, SystemId};
use rne_ecs::{spawn_named, Children, Name, Parent};
use rne_math::{Quat, Seconds, Vec3};
use rne_world::{propagate_transforms, GlobalTransform3, Transform3, WorldEntity};

struct HelloWorldPlugin;

impl Plugin for HelloWorldPlugin {
    fn name(&self) -> &'static str {
        "hello_world"
    }

    fn build(&self, schedule: &mut rne_core::Schedule) {
        schedule.add_system(SchedulePhase::PreUpdate, SystemId::new("hello"), || {
            println!("Robot Native Engine: hello world tick");
        });
    }
}

fn main() {
    let mut ecs_world = World::new();
    let world_entity = rne_world::spawn_world(&mut ecs_world);
    let world = ecs_world
        .get::<WorldEntity>(world_entity)
        .expect("world entity");
    println!(
        "Spawned world entity with gravity = ({:.2}, {:.2}, {:.2}) m/s²",
        world.gravity_m_s2.x, world.gravity_m_s2.y, world.gravity_m_s2.z
    );

    let parent = spawn_named(&mut ecs_world, "robot_base");
    let child = spawn_named(&mut ecs_world, "sensor_mount");
    ecs_world.entity_mut(parent).insert((
        Transform3::from_translation_rotation(Vec3::new(0.0, 0.0, 0.0), Quat::IDENTITY),
        GlobalTransform3::default(),
        Children(Default::default()),
    ));
    ecs_world.entity_mut(child).insert((
        Parent(parent),
        Transform3::from_translation_rotation(Vec3::new(0.0, 0.5, 0.0), Quat::IDENTITY),
        GlobalTransform3::default(),
    ));
    ecs_world
        .entity_mut(parent)
        .get_mut::<Children>()
        .unwrap()
        .0
        .push(child);

    let mut schedule = bevy_ecs::schedule::Schedule::default();
    schedule.add_systems(propagate_transforms);
    schedule.run(&mut ecs_world);

    let sensor_name = ecs_world.get::<Name>(child).unwrap();
    let sensor_global = ecs_world.get::<GlobalTransform3>(child).unwrap();
    let point = sensor_global.matrix.transform_point3(Vec3::ZERO);
    println!(
        "Entity '{}' global position = ({:.2}, {:.2}, {:.2}) m",
        sensor_name.0, point.x, point.y, point.z
    );

    let mut app = AppBuilder::new().add_plugin(HelloWorldPlugin).build();
    app.step(SimDuration::from_seconds(Seconds::new(1.0 / 60.0)));
    println!("Simulation time = {}", app.clock().sim_time());

    let _app: RneApp = app;
}
