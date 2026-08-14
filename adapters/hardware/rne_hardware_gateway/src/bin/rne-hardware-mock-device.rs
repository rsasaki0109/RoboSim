//! Process-isolated deterministic implementation of the hardware wire protocol.

use rne_hardware_gateway::mock::{MockDeviceConfig, MockDeviceFault, MockHardwareDevice};
use rne_hardware_gateway::wire::{DeviceWirePayload, HardwareWireCodec};
use std::env;
use std::io::{self, BufReader, BufWriter, Write};

fn main() {
    if let Err(error) = run() {
        eprintln!("rne hardware mock device failed: {error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let config = parse_args(env::args().skip(1))?;
    let mut device = MockHardwareDevice::new(config)?;
    let codec = HardwareWireCodec::default();
    let stdin = io::stdin();
    let stdout = io::stdout();
    let mut reader = BufReader::new(stdin.lock());
    let mut writer = BufWriter::new(stdout.lock());

    while let Some(line) = codec.read_line(&mut reader)? {
        let request = codec.decode_host(&line)?;
        let response = device.handle(request)?;
        writer.write_all(&codec.encode_device(&response)?)?;
        writer.flush()?;
        if matches!(
            response.payload,
            DeviceWirePayload::Closed
                | DeviceWirePayload::Disconnected { .. }
                | DeviceWirePayload::SafetySignal { .. }
        ) {
            break;
        }
    }
    Ok(())
}

fn parse_args(
    mut args: impl Iterator<Item = String>,
) -> Result<MockDeviceConfig, Box<dyn std::error::Error>> {
    let mut config = MockDeviceConfig::default();
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--device-id" => {
                config.device_id = required_value(&mut args, "--device-id")?;
            }
            "--disconnect-after-actuations" => {
                set_fault(
                    &mut config,
                    MockDeviceFault::DisconnectAfterActuations {
                        count: parse_count(&required_value(
                            &mut args,
                            "--disconnect-after-actuations",
                        )?)?,
                    },
                )?;
            }
            "--emergency-stop-after-observations" => {
                set_fault(
                    &mut config,
                    MockDeviceFault::EmergencyStopAfterObservations {
                        count: parse_count(&required_value(
                            &mut args,
                            "--emergency-stop-after-observations",
                        )?)?,
                    },
                )?;
            }
            "--help" | "-h" => {
                eprintln!(
                    "rne-hardware-mock-device [--device-id ID] \
                     [--disconnect-after-actuations COUNT | \
                     --emergency-stop-after-observations COUNT]"
                );
                std::process::exit(0);
            }
            other => return Err(format!("unknown option {other:?}").into()),
        }
    }
    config.validate()?;
    Ok(config)
}

fn required_value(
    args: &mut impl Iterator<Item = String>,
    option: &str,
) -> Result<String, Box<dyn std::error::Error>> {
    args.next()
        .filter(|value| !value.is_empty())
        .ok_or_else(|| format!("{option} requires a value").into())
}

fn parse_count(value: &str) -> Result<u64, Box<dyn std::error::Error>> {
    value
        .parse::<u64>()
        .map_err(|error| format!("invalid fault count {value:?}: {error}").into())
}

fn set_fault(
    config: &mut MockDeviceConfig,
    fault: MockDeviceFault,
) -> Result<(), Box<dyn std::error::Error>> {
    if config.fault.replace(fault).is_some() {
        return Err("only one mock fault may be configured".into());
    }
    Ok(())
}
