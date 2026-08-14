//! Deterministic stable-toolchain fuzz-smoke campaign for release input boundaries.

use rne_data::transport::{
    decode_control_command, decode_image_depth, decode_image_rgb8, decode_lidar_point_cloud,
    negotiate_transport, ClientHello, ControlAck, GapNotice, NegotiationPolicy, NegotiationReject,
    ServerHello, StatusMessage, TransportFrame, TransportMessageKind,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::panic::{catch_unwind, AssertUnwindSafe};

/// Machine-readable fuzz-smoke report schema version.
pub const FUZZ_SMOKE_REPORT_SCHEMA_VERSION: u32 = 1;
/// Release version covered by the campaign.
pub const FUZZ_SMOKE_RELEASE_VERSION: &str = "0.1.0";
/// Maximum bytes passed to any parser during the bounded CI campaign.
pub const FUZZ_SMOKE_MAX_INPUT_BYTES: usize = 64 * 1024;
/// Fixed seed used by every deterministic mutation stream.
pub const FUZZ_SMOKE_CAMPAIGN_SEED: u64 = 0x524e_452d_4d36_4301;

const MUTATIONS_PER_BOUNDARY: usize = 32;

/// One untrusted input boundary covered by stable and sanitizer-backed fuzzing.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum FuzzBoundary {
    /// OpenSCENARIO XML import.
    OpenScenarioXml,
    /// Scenario replay JSON import.
    ScenarioReplayJson,
    /// SDF XML conversion.
    SdfXml,
    /// MJCF XML conversion.
    MjcfXml,
    /// SUMO network XML import.
    SumoNetXml,
    /// Native RNE traffic JSON import.
    NativeTrafficJson,
    /// URDF XML import.
    UrdfXml,
    /// Runner/frontend frame and typed payload decoding.
    TransportFrame,
    /// Runner/frontend hello and negotiation decoding.
    TransportNegotiation,
}

impl FuzzBoundary {
    /// All release fuzz boundaries in stable report order.
    pub const ALL: [Self; 9] = [
        Self::OpenScenarioXml,
        Self::ScenarioReplayJson,
        Self::SdfXml,
        Self::MjcfXml,
        Self::SumoNetXml,
        Self::NativeTrafficJson,
        Self::UrdfXml,
        Self::TransportFrame,
        Self::TransportNegotiation,
    ];

    /// Import boundaries selected by the cargo-fuzz importer target.
    pub const IMPORTERS: [Self; 7] = [
        Self::OpenScenarioXml,
        Self::ScenarioReplayJson,
        Self::SdfXml,
        Self::MjcfXml,
        Self::SumoNetXml,
        Self::NativeTrafficJson,
        Self::UrdfXml,
    ];

    /// Transport boundaries selected by the cargo-fuzz transport target.
    pub const TRANSPORT: [Self; 2] = [Self::TransportFrame, Self::TransportNegotiation];

    /// Stable report and corpus identifier.
    pub const fn id(self) -> &'static str {
        match self {
            Self::OpenScenarioXml => "openscenario_xml",
            Self::ScenarioReplayJson => "scenario_replay_json",
            Self::SdfXml => "sdf_xml",
            Self::MjcfXml => "mjcf_xml",
            Self::SumoNetXml => "sumo_net_xml",
            Self::NativeTrafficJson => "native_traffic_json",
            Self::UrdfXml => "urdf_xml",
            Self::TransportFrame => "transport_frame",
            Self::TransportNegotiation => "transport_negotiation",
        }
    }

    fn from_id(id: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|boundary| boundary.id() == id)
    }
}

/// Per-boundary deterministic campaign evidence.
#[derive(Clone, Debug, Deserialize, PartialEq, Eq, Serialize)]
pub struct FuzzBoundaryReport {
    /// Stable boundary identifier.
    pub boundary: String,
    /// Number of committed valid and regression seeds.
    pub corpus_cases: usize,
    /// Number of deterministic derived cases.
    pub generated_cases: usize,
    /// Inputs accepted as valid by the boundary parser.
    pub accepted_cases: usize,
    /// Inputs rejected with an ordinary error or campaign limit.
    pub rejected_cases: usize,
    /// Panics caught by the stable campaign; must be zero.
    pub panic_count: usize,
    /// Largest case considered by the campaign, including the limit probe.
    pub largest_case_bytes: usize,
    /// Stable digest of names, bytes, and outcomes for this boundary.
    pub case_digest_sha256: String,
}

/// Stable-toolchain fuzz-smoke release evidence.
#[derive(Clone, Debug, Deserialize, PartialEq, Eq, Serialize)]
pub struct FuzzSmokeReport {
    /// Report schema version.
    pub schema_version: u32,
    /// Release version under test.
    pub release_version: String,
    /// Fixed deterministic campaign seed.
    pub campaign_seed: u64,
    /// Maximum bytes passed into a parser.
    pub max_input_bytes: usize,
    /// Number of cases across all boundaries.
    pub total_cases: usize,
    /// Digest of committed valid and regression corpus bytes.
    pub corpus_digest_sha256: String,
    /// Digest of all boundary case digests and campaign constants.
    pub campaign_digest_sha256: String,
    /// Boundary evidence in stable identifier order.
    pub boundaries: Vec<FuzzBoundaryReport>,
}

impl FuzzSmokeReport {
    /// Validates schema, coverage, limits, and panic freedom.
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != FUZZ_SMOKE_REPORT_SCHEMA_VERSION {
            return Err("unexpected fuzz-smoke report schema".to_string());
        }
        if self.release_version != FUZZ_SMOKE_RELEASE_VERSION {
            return Err("unexpected fuzz-smoke release version".to_string());
        }
        if self.campaign_seed != FUZZ_SMOKE_CAMPAIGN_SEED
            || self.max_input_bytes != FUZZ_SMOKE_MAX_INPUT_BYTES
        {
            return Err("unexpected fuzz-smoke campaign constants".to_string());
        }
        if self.boundaries.len() != FuzzBoundary::ALL.len() {
            return Err("fuzz-smoke boundary coverage is incomplete".to_string());
        }
        let expected = FuzzBoundary::ALL.map(FuzzBoundary::id);
        for (report, expected_id) in self.boundaries.iter().zip(expected) {
            if report.boundary != expected_id {
                return Err("fuzz-smoke boundaries are not in stable order".to_string());
            }
            if report.corpus_cases == 0 || report.accepted_cases == 0 {
                return Err(format!("{} has no accepted corpus", report.boundary));
            }
            if report.panic_count != 0 {
                return Err(format!("{} panicked", report.boundary));
            }
            if report.accepted_cases + report.rejected_cases + report.panic_count
                != report.corpus_cases + report.generated_cases
            {
                return Err(format!(
                    "{} case accounting is inconsistent",
                    report.boundary
                ));
            }
        }
        let counted = self
            .boundaries
            .iter()
            .map(|report| report.corpus_cases + report.generated_cases)
            .sum::<usize>();
        if counted != self.total_cases {
            return Err("fuzz-smoke total case count is inconsistent".to_string());
        }
        Ok(())
    }
}

/// Exercises one parser boundary without swallowing a panic.
///
/// Cargo-fuzz calls this function directly so sanitizer campaigns preserve
/// crash semantics. The stable campaign wraps it with [`catch_unwind`].
pub fn exercise_boundary(boundary: FuzzBoundary, data: &[u8]) -> bool {
    match boundary {
        FuzzBoundary::OpenScenarioXml => {
            utf8(data).is_some_and(|text| rne_openscenario::parse_openscenario_xml(text).is_ok())
        }
        FuzzBoundary::ScenarioReplayJson => utf8(data)
            .is_some_and(|text| rne_openscenario::ScenarioReplayArtifact::from_json(text).is_ok()),
        FuzzBoundary::SdfXml => utf8(data).is_some_and(|text| rne_sdf::sdf_to_urdf(text).is_ok()),
        FuzzBoundary::MjcfXml => {
            utf8(data).is_some_and(|text| rne_mjcf::mjcf_to_urdf(text).is_ok())
        }
        FuzzBoundary::SumoNetXml => rne_sumo::parse_sumo_net_xml(data).is_ok(),
        FuzzBoundary::NativeTrafficJson => rne_traffic::parse_traffic_asset(data).is_ok(),
        FuzzBoundary::UrdfXml => {
            utf8(data).is_some_and(|text| rne_urdf_import::parse_urdf(text).is_ok())
        }
        FuzzBoundary::TransportFrame => exercise_transport_frame(data),
        FuzzBoundary::TransportNegotiation => exercise_transport_negotiation(data),
    }
}

/// Runs every deterministic bounded campaign and returns validated evidence.
pub fn run_fuzz_smoke_campaign() -> FuzzSmokeReport {
    let regressions = regression_corpus();
    let mut corpus_hasher = Sha256::new();
    let mut reports = Vec::new();

    for (boundary_index, boundary) in FuzzBoundary::ALL.into_iter().enumerate() {
        let valid_seed = valid_seed(boundary);
        let mut corpus = vec![("valid".to_string(), valid_seed.clone())];
        corpus.extend(
            regressions
                .iter()
                .filter(|(candidate, _, _)| *candidate == boundary)
                .map(|(_, name, bytes)| (name.clone(), bytes.clone())),
        );
        for (name, bytes) in &corpus {
            hash_field(&mut corpus_hasher, boundary.id().as_bytes());
            hash_field(&mut corpus_hasher, name.as_bytes());
            hash_field(&mut corpus_hasher, bytes);
        }

        let generated = generated_cases(boundary, boundary_index as u64, &valid_seed);
        let corpus_cases = corpus.len();
        let generated_cases = generated.len();
        let mut cases = corpus;
        cases.extend(generated);

        let mut accepted_cases = 0;
        let mut rejected_cases = 0;
        let mut panic_count = 0;
        let mut largest_case_bytes = 0;
        let mut case_hasher = Sha256::new();
        for (name, bytes) in cases {
            largest_case_bytes = largest_case_bytes.max(bytes.len());
            let outcome = if bytes.len() > FUZZ_SMOKE_MAX_INPUT_BYTES {
                1_u8
            } else {
                match catch_unwind(AssertUnwindSafe(|| exercise_boundary(boundary, &bytes))) {
                    Ok(true) => 0,
                    Ok(false) => 1,
                    Err(_) => 2,
                }
            };
            match outcome {
                0 => accepted_cases += 1,
                1 => rejected_cases += 1,
                _ => panic_count += 1,
            }
            hash_field(&mut case_hasher, name.as_bytes());
            hash_field(&mut case_hasher, &bytes);
            case_hasher.update([outcome]);
        }

        reports.push(FuzzBoundaryReport {
            boundary: boundary.id().to_string(),
            corpus_cases,
            generated_cases,
            accepted_cases,
            rejected_cases,
            panic_count,
            largest_case_bytes,
            case_digest_sha256: format!("sha256:{:x}", case_hasher.finalize()),
        });
    }

    let corpus_digest_sha256 = format!("sha256:{:x}", corpus_hasher.finalize());
    let mut campaign_hasher = Sha256::new();
    campaign_hasher.update(FUZZ_SMOKE_REPORT_SCHEMA_VERSION.to_le_bytes());
    campaign_hasher.update(FUZZ_SMOKE_CAMPAIGN_SEED.to_le_bytes());
    campaign_hasher.update((FUZZ_SMOKE_MAX_INPUT_BYTES as u64).to_le_bytes());
    hash_field(&mut campaign_hasher, corpus_digest_sha256.as_bytes());
    for report in &reports {
        hash_field(&mut campaign_hasher, report.boundary.as_bytes());
        hash_field(&mut campaign_hasher, report.case_digest_sha256.as_bytes());
    }
    let total_cases = reports
        .iter()
        .map(|report| report.corpus_cases + report.generated_cases)
        .sum();
    FuzzSmokeReport {
        schema_version: FUZZ_SMOKE_REPORT_SCHEMA_VERSION,
        release_version: FUZZ_SMOKE_RELEASE_VERSION.to_string(),
        campaign_seed: FUZZ_SMOKE_CAMPAIGN_SEED,
        max_input_bytes: FUZZ_SMOKE_MAX_INPUT_BYTES,
        total_cases,
        corpus_digest_sha256,
        campaign_digest_sha256: format!("sha256:{:x}", campaign_hasher.finalize()),
        boundaries: reports,
    }
}

fn utf8(data: &[u8]) -> Option<&str> {
    std::str::from_utf8(data).ok()
}

fn exercise_transport_frame(data: &[u8]) -> bool {
    let Ok(frame) = TransportFrame::decode(data, FUZZ_SMOKE_MAX_INPUT_BYTES) else {
        return false;
    };
    match frame.kind {
        TransportMessageKind::ClientHello => ClientHello::decode_payload(&frame.payload).is_ok(),
        TransportMessageKind::ServerHello => ServerHello::decode_payload(&frame.payload).is_ok(),
        TransportMessageKind::Reject => NegotiationReject::decode_payload(&frame.payload).is_ok(),
        TransportMessageKind::ControlCommand => decode_control_command(&frame.payload).is_ok(),
        TransportMessageKind::ControlAck => ControlAck::decode_payload(&frame.payload).is_ok(),
        TransportMessageKind::Status => StatusMessage::decode_payload(&frame.payload).is_ok(),
        TransportMessageKind::ImageRgb8 => decode_image_rgb8(&frame.payload).is_ok(),
        TransportMessageKind::ImageDepthF32 => decode_image_depth(&frame.payload).is_ok(),
        TransportMessageKind::LidarPointCloud => decode_lidar_point_cloud(&frame.payload).is_ok(),
        TransportMessageKind::Gap => GapNotice::decode_payload(&frame.payload).is_ok(),
    }
}

fn exercise_transport_negotiation(data: &[u8]) -> bool {
    let client = ClientHello::decode_payload(data);
    let server = ServerHello::decode_payload(data);
    let reject = NegotiationReject::decode_payload(data);
    let negotiated = client
        .as_ref()
        .ok()
        .is_some_and(|hello| negotiate_transport(*hello, NegotiationPolicy::default()).is_ok());
    client.is_ok() || server.is_ok() || reject.is_ok() || negotiated
}

fn valid_seed(boundary: FuzzBoundary) -> Vec<u8> {
    match boundary {
        FuzzBoundary::OpenScenarioXml => {
            include_bytes!("../../../crates/rne_openscenario/tests/fixtures/minimal_speed.xosc")
                .to_vec()
        }
        FuzzBoundary::ScenarioReplayJson => valid_scenario_replay(),
        FuzzBoundary::SdfXml => {
            include_bytes!("../../../crates/rne_sdf/tests/fixtures/two_link_arm.sdf").to_vec()
        }
        FuzzBoundary::MjcfXml => {
            include_bytes!("../../../crates/rne_mjcf/tests/fixtures/two_link_arm.xml").to_vec()
        }
        FuzzBoundary::SumoNetXml => {
            include_bytes!("../../../assets/networks/minimal_cross.net.xml").to_vec()
        }
        FuzzBoundary::NativeTrafficJson => {
            include_bytes!("../../../assets/traffic/corridor.rne.traffic.json").to_vec()
        }
        FuzzBoundary::UrdfXml => {
            include_bytes!("../../../crates/rne_urdf_import/tests/fixtures/minimal_diff_drive.urdf")
                .to_vec()
        }
        FuzzBoundary::TransportFrame => {
            let payload = valid_client_hello().encode_payload();
            TransportFrame::new(TransportMessageKind::ClientHello, 1, 1, payload)
                .encode()
                .expect("valid transport seed")
        }
        FuzzBoundary::TransportNegotiation => valid_client_hello().encode_payload(),
    }
}

fn valid_client_hello() -> ClientHello {
    ClientHello {
        min_protocol_major: 1,
        max_protocol_major: 1,
        capabilities: rne_data::transport::TransportCapabilities::ALL_V1,
        required_capabilities: rne_data::transport::TransportCapabilities::CONTROL,
        max_payload_bytes: FUZZ_SMOKE_MAX_INPUT_BYTES as u32,
        queue_frame_limit: 4,
        queue_byte_limit: (FUZZ_SMOKE_MAX_INPUT_BYTES * 2) as u32,
        resume_after_sequence: None,
    }
}

fn valid_scenario_replay() -> Vec<u8> {
    let mut digest_input = b"rne-scenario-result-v1".to_vec();
    digest_input.extend_from_slice(&0_u64.to_le_bytes());
    digest_input.extend_from_slice(&0_u64.to_le_bytes());
    let result = rne_openscenario::ScenarioRunResult {
        stable_hash: 0,
        result_digest: rne_openscenario::stable_replay_input_digest(&digest_input),
        signal_violations: 0,
        collisions: 0,
        final_positions_m: Vec::new(),
        final_actors: Vec::new(),
        action_evidence: Vec::new(),
        unapplied_action_count: 0,
        minimum_observed_gap_m: None,
        ownership: rne_traffic::TrafficOwnershipMetrics {
            total_actor_count: 0,
            runtime_owned_actor_count: 0,
            external_owned_actor_count: 0,
            runtime_advanced_actor_count: 0,
            external_observed_actor_count: 0,
            invalid_actor_count: 0,
        },
        route_length_m: 0.0,
        average_speed_m_s: 0.0,
        steps: 0,
    };
    let artifact = rne_openscenario::ScenarioReplayArtifact::new(
        rne_openscenario::ScenarioReplayInputs::new(
            "fuzz.xosc",
            rne_openscenario::stable_replay_input_digest(b"scenario"),
            "fuzz.rne.traffic.json",
            rne_openscenario::stable_replay_input_digest(b"network"),
        ),
        rne_openscenario::ScenarioRunOptions { steps: 1, hz: 60.0 },
        0,
        Vec::new(),
        result,
    );
    artifact
        .to_json()
        .expect("valid scenario replay seed")
        .into_bytes()
}

fn generated_cases(
    boundary: FuzzBoundary,
    boundary_index: u64,
    seed: &[u8],
) -> Vec<(String, Vec<u8>)> {
    let mut cases = vec![
        ("empty".to_string(), Vec::new()),
        (
            "truncated_one".to_string(),
            seed[..seed.len().min(1)].to_vec(),
        ),
        (
            "truncated_half".to_string(),
            seed[..seed.len() / 2].to_vec(),
        ),
        ("invalid_utf8".to_string(), vec![0xff, 0xfe, 0xf8, 0x00]),
        (
            "delimiter_heavy".to_string(),
            delimiter_heavy(boundary, 4 * 1024),
        ),
        (
            "campaign_limit_plus_one".to_string(),
            vec![b'x'; FUZZ_SMOKE_MAX_INPUT_BYTES + 1],
        ),
    ];
    if boundary == FuzzBoundary::MjcfXml {
        cases.push(("deep_body_nesting".to_string(), deep_mjcf_input()));
    }

    let mut random = XorShift64::new(
        FUZZ_SMOKE_CAMPAIGN_SEED ^ boundary_index.rotate_left(17) ^ stable_seed(seed),
    );
    for index in 0..MUTATIONS_PER_BOUNDARY {
        let mut bytes = seed[..seed.len().min(FUZZ_SMOKE_MAX_INPUT_BYTES)].to_vec();
        mutate(&mut bytes, &mut random);
        cases.push((format!("mutation_{index:02}"), bytes));
    }
    cases
}

fn mutate(bytes: &mut Vec<u8>, random: &mut XorShift64) {
    match random.next() % 5 {
        0 if !bytes.is_empty() => {
            let index = random.index(bytes.len());
            bytes[index] ^= (random.next() as u8) | 1;
        }
        1 if bytes.len() < FUZZ_SMOKE_MAX_INPUT_BYTES => {
            let index = random.index(bytes.len() + 1);
            bytes.insert(index, random.next() as u8);
        }
        2 if !bytes.is_empty() => {
            let start = random.index(bytes.len());
            let end = (start + 1 + random.index(bytes.len() - start)).min(bytes.len());
            bytes.drain(start..end);
        }
        3 if !bytes.is_empty() => {
            let start = random.index(bytes.len());
            let count = (1 + random.index(16)).min(bytes.len() - start);
            for byte in &mut bytes[start..start + count] {
                *byte = [b'<', b'>', b'{', b'}', b'"', 0, 0xff][random.index(7)];
            }
        }
        _ => {
            let new_len = random.index(bytes.len().saturating_add(1));
            bytes.truncate(new_len);
        }
    }
}

fn delimiter_heavy(boundary: FuzzBoundary, len: usize) -> Vec<u8> {
    let delimiters: &[u8] = match boundary {
        FuzzBoundary::TransportFrame | FuzzBoundary::TransportNegotiation => {
            &[0, 0xff, b'R', b'N', b'E', b'F']
        }
        FuzzBoundary::ScenarioReplayJson | FuzzBoundary::NativeTrafficJson => br#"{}[],:\"\\"#,
        _ => br#"<>&/='\""#,
    };
    (0..len)
        .map(|index| delimiters[index % delimiters.len()])
        .collect()
}

fn deep_mjcf_input() -> Vec<u8> {
    let depth = 132;
    let mut xml = String::from("<mujoco><worldbody>");
    for index in 0..depth {
        xml.push_str(&format!("<body name=\"b{index}\">"));
    }
    for _ in 0..depth {
        xml.push_str("</body>");
    }
    xml.push_str("</worldbody></mujoco>");
    xml.into_bytes()
}

fn regression_corpus() -> Vec<(FuzzBoundary, String, Vec<u8>)> {
    include_str!("../corpus/regressions.txt")
        .lines()
        .filter(|line| !line.trim().is_empty() && !line.starts_with('#'))
        .map(|line| {
            let mut fields = line.split('|');
            let boundary = fields
                .next()
                .and_then(FuzzBoundary::from_id)
                .expect("known regression boundary");
            let name = fields.next().expect("regression name").to_string();
            let bytes = decode_hex(fields.next().expect("regression hex"));
            assert!(fields.next().is_none(), "three regression fields");
            (boundary, name, bytes)
        })
        .collect()
}

fn decode_hex(hex: &str) -> Vec<u8> {
    assert!(hex.len().is_multiple_of(2), "even regression hex length");
    hex.as_bytes()
        .chunks_exact(2)
        .map(|pair| {
            let text = std::str::from_utf8(pair).expect("ASCII hex");
            u8::from_str_radix(text, 16).expect("valid regression hex")
        })
        .collect()
}

fn stable_seed(bytes: &[u8]) -> u64 {
    let digest = Sha256::digest(bytes);
    u64::from_le_bytes(digest[..8].try_into().expect("eight digest bytes"))
}

fn hash_field(hasher: &mut Sha256, bytes: &[u8]) {
    hasher.update((bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Debug)]
struct XorShift64(u64);

impl XorShift64 {
    fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }

    fn next(&mut self) -> u64 {
        let mut value = self.0;
        value ^= value << 13;
        value ^= value >> 7;
        value ^= value << 17;
        self.0 = value;
        value
    }

    fn index(&mut self, upper: usize) -> usize {
        if upper == 0 {
            0
        } else {
            (self.next() as usize) % upper
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn campaign_is_deterministic_complete_and_panic_free() {
        let first = run_fuzz_smoke_campaign();
        let second = run_fuzz_smoke_campaign();
        assert_eq!(first, second);
        first.validate().expect("valid fuzz-smoke report");
        assert!(first.total_cases >= 350);
    }

    #[test]
    fn every_valid_seed_is_accepted() {
        for boundary in FuzzBoundary::ALL {
            assert!(
                exercise_boundary(boundary, &valid_seed(boundary)),
                "{}",
                boundary.id()
            );
        }
    }

    #[test]
    fn report_round_trips_through_json() {
        let report = run_fuzz_smoke_campaign();
        let json = serde_json::to_vec_pretty(&report).expect("serialize report");
        let decoded: FuzzSmokeReport = serde_json::from_slice(&json).expect("decode report");
        assert_eq!(decoded, report);
    }
}
