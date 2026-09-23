//! Local robot workbench application.

#[cfg(feature = "gpu")]
mod host;
mod project;
#[cfg(feature = "gpu")]
mod server;
mod simulation;

use anyhow::{bail, ensure, Context, Result};
use project::{JointPosition, Project, RobotFormat, RobotSource};
use std::io::Read;
#[cfg(feature = "gpu")]
use std::path::PathBuf;

fn read_bounded(path: &str) -> Result<Vec<u8>> {
    let mut bytes = Vec::new();
    std::fs::File::open(path)?
        .take(4 * 1024 * 1024 + 1)
        .read_to_end(&mut bytes)?;
    ensure!(bytes.len() <= 4 * 1024 * 1024, "file exceeds 4 MiB");
    Ok(bytes)
}

fn smoke() -> Result<()> {
    let mut project = Project::new(RobotSource {
        format: RobotFormat::Urdf,
        xml: include_str!("../../crates/rne_urdf_import/tests/fixtures/prismatic_slider.urdf")
            .into(),
    })?;
    project.targets.insert(
        "slider_joint".into(),
        JointPosition::Prismatic { position_m: 0.1 },
    );
    project.save_pose("extended")?;
    let saved = project.poses["extended"].clone();
    project.apply_pose(&saved)?;
    let roundtrip = Project::from_json(&serde_json::to_vec(&project)?)?;
    let mut simulation = simulation::Simulation::new(roundtrip, None)?;
    simulation.set_targets(project.targets.clone())?;
    simulation.step(240)?;
    ensure!(
        simulation.readings()?[0].measured > 0.06,
        "native slider did not reach its target"
    );
    ensure!(
        !simulation.render_scene().items.is_empty(),
        "missing robot visuals"
    );
    ensure!(
        simulation.camera_fit().1.is_finite(),
        "invalid camera bounds"
    );
    ensure!(
        simulation.lidar()?.ranges_m.len() == 180,
        "incomplete LiDAR scan"
    );
    let mut arm = Project::new(RobotSource {
        format: RobotFormat::Mjcf,
        xml: include_str!("../../crates/rne_mjcf/tests/fixtures/two_link_arm.xml").into(),
    })?;
    arm.targets.insert(
        "shoulder".into(),
        JointPosition::Revolute { position_rad: 0.3 },
    );
    let mut arm_simulation = simulation::Simulation::new(arm, None)?;
    arm_simulation.step(240)?;
    ensure!(
        arm_simulation
            .readings()?
            .iter()
            .any(|joint| joint.info.name == "shoulder" && joint.measured > 0.1),
        "MJCF arm did not move"
    );
    ensure!(
        arm_simulation.readings()?.len() == 2,
        "MJCF joint import failed"
    );
    println!("workbench smoke passed: URDF servo, MJCF articulation, pose/project roundtrip, visuals, LiDAR; state={:016x}", simulation.state_hash());
    Ok(())
}

fn main() -> Result<()> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    match args.as_slice() {
        [command, path] if command == "--check-project" => {
            let bytes = read_bounded(path).context("read workbench project")?;
            let project = Project::from_json(&bytes)?;
            let joints = project.joint_catalog()?;
            let _simulation = simulation::Simulation::new(project.clone(), None)?;
            println!(
                "project valid: {} joints, {} scene objects, {} saved poses",
                joints.len(),
                project.objects.len(),
                project.poses.len()
            );
            Ok(())
        }
        [command] if command == "--smoke" => smoke(),
        [command] if command == "--help" => {
            println!("rne-workbench [--port PORT] [--robot MODEL.urdf|MODEL.xml] [--asset-root DIRECTORY]\nrne-workbench --check-project PROJECT.json\nrne-workbench --smoke");
            Ok(())
        }
        _ => serve(&args),
    }
}

#[cfg(feature = "gpu")]
fn serve(args: &[String]) -> Result<()> {
    let mut port = 8765;
    let mut robot = None;
    let mut asset_root =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/robots/so101");
    let mut args = args.iter();
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--port" => port = args.next().context("--port requires a number")?.parse()?,
            "--robot" => robot = Some(args.next().context("--robot requires a file")?.clone()),
            "--asset-root" => {
                asset_root =
                    PathBuf::from(args.next().context("--asset-root requires a directory")?)
            }
            _ => bail!("unknown argument {argument}; see --help"),
        }
    }
    let project = if let Some(path) = robot {
        let xml = String::from_utf8(read_bounded(&path)?)?;
        let format = if xml.contains("<mujoco") {
            RobotFormat::Mjcf
        } else {
            RobotFormat::Urdf
        };
        Project::new(RobotSource { format, xml })?
    } else {
        host::preset("so101")?
    };
    server::serve(host::Host::new(project, asset_root.canonicalize()?)?, port)
}

#[cfg(not(feature = "gpu"))]
fn serve(_args: &[String]) -> Result<()> {
    bail!("interactive workbench requires the gpu feature; headless --smoke and --check-project remain available")
}
