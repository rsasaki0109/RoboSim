use anyhow::{ensure, Context};
use rne_data::transport::{
    negotiate_transport, ClientHello, NegotiationPolicy, SensorFrameMetadata,
    TransportCapabilities, TransportFrame, TransportMessageKind, TRANSPORT_PROTOCOL_MAJOR,
};
use rne_data::{
    encode_dataset_action, encode_dataset_annotation, encode_dataset_imu,
    encode_dataset_task_outcome, encode_dataset_transform, DatasetActionSample,
    DatasetGroundTruthAnnotation, DatasetTaskOutcomeSample, ImuSample, PoseSample,
    DATASET_PAYLOAD_SCHEMA_VERSION,
};
use rne_math::Vec3;
use serde_json::{json, Value};
use std::env;
use std::fs;
use std::path::Path;

fn main() -> anyhow::Result<()> {
    let output = env::args_os()
        .nth(1)
        .context("usage: generate_binary_fixtures <output-directory>")?;
    let output = Path::new(&output);
    fs::create_dir_all(output)
        .with_context(|| format!("create fixture output {}", output.display()))?;
    write_json(
        &output.join("frontend-transport-v1.json"),
        &frontend_fixture()?,
    )?;
    write_json(&output.join("dataset-payload-v1.json"), &dataset_fixture()?)
}

fn frontend_fixture() -> anyhow::Result<Value> {
    let hello = ClientHello {
        min_protocol_major: TRANSPORT_PROTOCOL_MAJOR,
        max_protocol_major: TRANSPORT_PROTOCOL_MAJOR,
        capabilities: TransportCapabilities::ALL_V1,
        required_capabilities: TransportCapabilities::CONTROL.union(TransportCapabilities::STATUS),
        max_payload_bytes: 1024 * 1024,
        queue_frame_limit: 16,
        queue_byte_limit: 2 * 1024 * 1024,
        resume_after_sequence: Some(41),
    };
    let frame = TransportFrame::new(
        TransportMessageKind::ClientHello,
        42,
        0x1122_3344_5566_7788,
        hello.encode_payload(),
    );
    let negotiated = negotiate_transport(hello, NegotiationPolicy::default())
        .map_err(|reject| anyhow::anyhow!("reference hello rejected: {:?}", reject.code))?;
    Ok(json!({
        "schema_version": 1,
        "message_kind": "client_hello",
        "protocol_major": frame.protocol_major,
        "protocol_minor": frame.protocol_minor,
        "flags": frame.flags,
        "sequence": frame.sequence,
        "session_id": frame.session_id,
        "frame_hex": lower_hex(&frame.encode()?),
        "hello": {
            "min_protocol_major": hello.min_protocol_major,
            "max_protocol_major": hello.max_protocol_major,
            "capabilities_bits": hello.capabilities.bits(),
            "required_capabilities_bits": hello.required_capabilities.bits(),
            "max_payload_bytes": hello.max_payload_bytes,
            "queue_frame_limit": hello.queue_frame_limit,
            "queue_byte_limit": hello.queue_byte_limit,
            "resume_after_sequence": hello.resume_after_sequence,
        },
        "negotiated": {
            "protocol_major": negotiated.protocol_major,
            "protocol_minor": negotiated.protocol_minor,
            "capabilities_bits": negotiated.capabilities.bits(),
            "max_payload_bytes": negotiated.max_payload_bytes,
            "queue_frame_limit": negotiated.queue_frame_limit,
            "queue_byte_limit": negotiated.queue_byte_limit,
            "resume_after_sequence": negotiated.resume_after_sequence,
        }
    }))
}

fn dataset_fixture() -> anyhow::Result<Value> {
    let metadata = SensorFrameMetadata {
        stream_id: 17,
        sensor_sequence: 42,
        capture_ticks: 1_000_000,
        available_ticks: 1_250_000,
    };
    ensure!(
        metadata.available_ticks >= metadata.capture_ticks,
        "reference dataset timestamp order is invalid"
    );
    let imu = ImuSample {
        angular_velocity_rad_s: Vec3::new(0.125, -0.25, 0.5),
        linear_acceleration_m_s2: Vec3::new(0.0, 9.806_65, -1.5),
    };
    let transform = PoseSample {
        position_m: Vec3::new(1.25, -2.5, 0.75),
        yaw_rad: -0.625,
    };
    let action = DatasetActionSample {
        values: vec![-1.0, 0.25, 2.5],
    };
    let outcome = DatasetTaskOutcomeSample {
        episode_index: 7,
        step_in_episode: 19,
        reward: 0.75,
        cumulative_reward: 12.5,
        terminated: true,
        truncated: false,
        success: Some(true),
    };
    let annotation = DatasetGroundTruthAnnotation {
        class_id: 3,
        instance_id: 99,
        values: vec![1.5, -2.0, 0.125, 4.75],
    };
    Ok(json!({
        "schema_version": DATASET_PAYLOAD_SCHEMA_VERSION,
        "metadata": {
            "stream_id": metadata.stream_id,
            "sensor_sequence": metadata.sensor_sequence,
            "capture_ticks": metadata.capture_ticks,
            "available_ticks": metadata.available_ticks,
        },
        "imu": {
            "angular_velocity_rad_s": [imu.angular_velocity_rad_s.x, imu.angular_velocity_rad_s.y, imu.angular_velocity_rad_s.z],
            "linear_acceleration_m_s2": [imu.linear_acceleration_m_s2.x, imu.linear_acceleration_m_s2.y, imu.linear_acceleration_m_s2.z],
            "payload_hex": lower_hex(&encode_dataset_imu(metadata, &imu)?),
        },
        "transform": {
            "position_m": [transform.position_m.x, transform.position_m.y, transform.position_m.z],
            "yaw_rad": transform.yaw_rad,
            "payload_hex": lower_hex(&encode_dataset_transform(metadata, &transform)?),
        },
        "action": {
            "values": action.values,
            "payload_hex": lower_hex(&encode_dataset_action(metadata, &action)?),
        },
        "outcome": {
            "episode_index": outcome.episode_index,
            "step_in_episode": outcome.step_in_episode,
            "reward": outcome.reward,
            "cumulative_reward": outcome.cumulative_reward,
            "terminated": outcome.terminated,
            "truncated": outcome.truncated,
            "success": outcome.success,
            "payload_hex": lower_hex(&encode_dataset_task_outcome(metadata, &outcome)?),
        },
        "annotation": {
            "class_id": annotation.class_id,
            "instance_id": annotation.instance_id,
            "values": annotation.values,
            "payload_hex": lower_hex(&encode_dataset_annotation(metadata, &annotation)?),
        },
    }))
}

fn lower_hex(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn write_json(path: &Path, value: &Value) -> anyhow::Result<()> {
    let mut bytes = serde_json::to_vec_pretty(value)?;
    bytes.push(b'\n');
    fs::write(path, bytes).with_context(|| format!("write fixture {}", path.display()))
}
