//! Process-isolated deterministic fixture for the simulator wire protocol.

use rne_hardware_gateway::simulator::mock::{MockSimulatorAdapter, MockSimulatorBinding};
use rne_hardware_gateway::simulator::wire::{SimulatorAdapterPayload, SimulatorWireCodec};
use std::io::{self, BufReader, BufWriter, Write};

fn main() {
    if let Err(error) = run() {
        eprintln!("simulator mock adapter failed: {error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let binding = parse_args(std::env::args().skip(1))?;
    let mut adapter = MockSimulatorAdapter::new(binding)?;
    let codec = SimulatorWireCodec::default();
    let stdin = io::stdin();
    let stdout = io::stdout();
    let mut reader = BufReader::new(stdin.lock());
    let mut writer = BufWriter::new(stdout.lock());
    while let Some(line) = codec.read_line(&mut reader)? {
        let request = codec.decode_host(&line)?;
        let response = adapter.handle(request)?;
        writer.write_all(&codec.encode_adapter(&response)?)?;
        writer.flush()?;
        if matches!(response.payload, SimulatorAdapterPayload::Closed) {
            break;
        }
    }
    Ok(())
}

fn parse_args(
    mut args: impl Iterator<Item = String>,
) -> Result<MockSimulatorBinding, Box<dyn std::error::Error>> {
    let mut simulator_id = "gazebo_sim_fixture".to_string();
    let mut simulator_version = "8.9.0".to_string();
    let mut adapter_id = "rne_gazebo_fixture".to_string();
    let mut task_id = None;
    let mut task_sha256 = None;
    let mut observation_width = None;
    let mut action_width = None;
    let mut fixed_delta_ticks = None;
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--simulator-id" => simulator_id = required(&mut args, "--simulator-id")?,
            "--simulator-version" => {
                simulator_version = required(&mut args, "--simulator-version")?
            }
            "--adapter-id" => adapter_id = required(&mut args, "--adapter-id")?,
            "--task-id" => task_id = Some(required(&mut args, "--task-id")?),
            "--task-sha256" => task_sha256 = Some(required(&mut args, "--task-sha256")?),
            "--observation-width" => {
                observation_width = Some(required(&mut args, "--observation-width")?.parse()?)
            }
            "--action-width" => {
                action_width = Some(required(&mut args, "--action-width")?.parse()?)
            }
            "--fixed-delta-ticks" => {
                fixed_delta_ticks = Some(required(&mut args, "--fixed-delta-ticks")?.parse()?)
            }
            "--help" | "-h" => {
                eprintln!(
                    "rne-simulator-mock-adapter --task-id ID --task-sha256 HEX \
                     --observation-width N --action-width N --fixed-delta-ticks N"
                );
                std::process::exit(0);
            }
            other => return Err(format!("unknown option {other:?}").into()),
        }
    }
    Ok(MockSimulatorBinding {
        simulator_id,
        simulator_version,
        adapter_id,
        task_id: task_id.ok_or("--task-id is required")?,
        task_sha256: task_sha256.ok_or("--task-sha256 is required")?,
        observation_width: observation_width.ok_or("--observation-width is required")?,
        action_width: action_width.ok_or("--action-width is required")?,
        fixed_delta_ticks: fixed_delta_ticks.ok_or("--fixed-delta-ticks is required")?,
    })
}

fn required(
    args: &mut impl Iterator<Item = String>,
    option: &str,
) -> Result<String, Box<dyn std::error::Error>> {
    args.next()
        .filter(|value| !value.is_empty())
        .ok_or_else(|| format!("{option} requires a value").into())
}
