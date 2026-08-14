//! Versioned, streaming dataset bundles for sensor and policy evidence.
//!
//! The format is deliberately renderer-neutral. Producers append one bounded
//! record at a time, while readers validate ordering, timing, and hashes without
//! retaining the complete run in memory.

use crate::transport::{
    decode_image_depth, decode_image_rgb8, decode_lidar_point_cloud, encode_image_depth,
    encode_image_rgb8, encode_lidar_point_cloud, SensorFrameMetadata, TransportError,
    TRANSPORT_MAX_PAYLOAD_BYTES,
};
use crate::{Frame, ImageDepth, ImageRgb8, PointCloud, StreamId};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::fs::{self, File};
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};
use thiserror::Error;

/// Current dataset bundle manifest and record schema.
pub const DATASET_BUNDLE_SCHEMA_VERSION: u32 = 1;
/// Maximum accepted manifest size.
pub const DATASET_MAX_MANIFEST_BYTES: u64 = 4 * 1024 * 1024;
/// Maximum number of declared streams in one bundle.
pub const DATASET_MAX_STREAMS: usize = 4096;

const MANIFEST_NAME: &str = "manifest.json";
const SHARD_NAME: &str = "records.rnedata";
const PARTIAL_SHARD_NAME: &str = "records.rnedata.partial";
const PARTIAL_MANIFEST_NAME: &str = "manifest.json.partial";
const FILE_MAGIC: [u8; 8] = *b"RNEDATA1";
const FILE_HEADER_BYTES: usize = 16;
const RECORD_HEADER_BYTES: usize = 80;

/// Failure while writing, reading, or validating a dataset bundle.
#[derive(Debug, Error)]
pub enum DatasetError {
    /// Filesystem input or output failed.
    #[error("dataset I/O failed: {0}")]
    Io(#[from] io::Error),
    /// JSON serialization or parsing failed.
    #[error("dataset JSON failed: {0}")]
    Json(#[from] serde_json::Error),
    /// A typed sensor payload failed its existing transport codec contract.
    #[error("dataset payload codec failed: {0}")]
    Transport(#[from] TransportError),
    /// A named manifest field violated the dataset contract.
    #[error("invalid dataset field `{field}`: {reason}")]
    InvalidField {
        /// Stable field name.
        field: &'static str,
        /// Human-readable invariant failure.
        reason: String,
    },
    /// A record referenced a stream absent from the manifest.
    #[error("dataset record references undeclared stream {0}")]
    UnknownStream(u64),
    /// A record kind did not match its declared stream kind.
    #[error("dataset record kind {actual:?} does not match stream {stream_id} kind {declared:?}")]
    StreamKindMismatch {
        /// Stream identifier.
        stream_id: u64,
        /// Kind declared in the manifest.
        declared: DatasetStreamKind,
        /// Kind encoded in the record.
        actual: DatasetRecordKind,
    },
    /// A stream sequence was missing, repeated, or out of order.
    #[error("dataset stream {stream_id} expected sequence {expected}, got {actual}")]
    SequenceMismatch {
        /// Stream identifier.
        stream_id: u64,
        /// Next required sequence.
        expected: u64,
        /// Sequence found in the record.
        actual: u64,
    },
    /// A stream capture timestamp moved backwards.
    #[error(
        "dataset stream {stream_id} capture timestamp moved backwards from {previous} to {actual}"
    )]
    CaptureTimeRegression {
        /// Stream identifier.
        stream_id: u64,
        /// Previous capture tick.
        previous: u64,
        /// Current capture tick.
        actual: u64,
    },
    /// A declared or embedded SHA-256 digest did not match the bytes.
    #[error("dataset digest mismatch for {0}")]
    DigestMismatch(String),
    /// A record or file ended before all declared bytes were available.
    #[error("truncated dataset record stream")]
    Truncated,
    /// The record file used an unsupported header or schema.
    #[error("invalid dataset record file header")]
    InvalidFileHeader,
    /// Bytes remained after the declared shard boundary.
    #[error("dataset shard contains trailing or undeclared bytes")]
    TrailingBytes,
}

/// Semantic role and codec family for a declared stream.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DatasetStreamKind {
    /// RGBA8 camera frames.
    Rgb8,
    /// Linear-depth f32 camera frames in metres.
    DepthF32,
    /// LiDAR point clouds.
    LidarPointCloud,
    /// Inertial measurements.
    Imu,
    /// Coordinate transforms or poses.
    Transform,
    /// Policy or controller actions.
    Action,
    /// Episode termination and reward outcomes.
    TaskOutcome,
    /// Ground-truth labels or annotations.
    GroundTruthAnnotation,
}

impl DatasetStreamKind {
    fn is_sensor(self) -> bool {
        matches!(
            self,
            Self::Rgb8 | Self::DepthF32 | Self::LidarPointCloud | Self::Imu
        )
    }

    fn accepts(self, record: DatasetRecordKind) -> bool {
        matches!(
            (self, record),
            (Self::Rgb8, DatasetRecordKind::Rgb8)
                | (Self::DepthF32, DatasetRecordKind::DepthF32)
                | (Self::LidarPointCloud, DatasetRecordKind::LidarPointCloud)
                | (Self::Imu, DatasetRecordKind::Imu)
                | (Self::Transform, DatasetRecordKind::Transform)
                | (Self::Action, DatasetRecordKind::Action)
                | (Self::TaskOutcome, DatasetRecordKind::TaskOutcome)
                | (
                    Self::GroundTruthAnnotation,
                    DatasetRecordKind::GroundTruthAnnotation
                )
        )
    }
}

/// Binary record discriminator stored in `records.rnedata`.
#[repr(u16)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DatasetRecordKind {
    /// Explicit declaration of one or more dropped sequences.
    Gap = 0,
    /// RGBA8 camera frame encoded by the RNE transport codec.
    Rgb8 = 1,
    /// Linear-depth f32 image encoded by the RNE transport codec.
    DepthF32 = 2,
    /// LiDAR point cloud encoded by the RNE transport codec.
    LidarPointCloud = 3,
    /// Versioned inertial payload bytes.
    Imu = 4,
    /// Versioned transform payload bytes.
    Transform = 5,
    /// Versioned action payload bytes.
    Action = 6,
    /// Versioned task outcome payload bytes.
    TaskOutcome = 7,
    /// Versioned ground-truth annotation bytes.
    GroundTruthAnnotation = 8,
}

impl TryFrom<u16> for DatasetRecordKind {
    type Error = DatasetError;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Gap),
            1 => Ok(Self::Rgb8),
            2 => Ok(Self::DepthF32),
            3 => Ok(Self::LidarPointCloud),
            4 => Ok(Self::Imu),
            5 => Ok(Self::Transform),
            6 => Ok(Self::Action),
            7 => Ok(Self::TaskOutcome),
            8 => Ok(Self::GroundTruthAnnotation),
            other => Err(invalid("record_kind", format!("unknown value {other}"))),
        }
    }
}

/// Machine-readable field name, scalar encoding, and unit.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatasetFieldSpec {
    /// Stable field name.
    pub name: String,
    /// Scalar or array encoding such as `f32[]`.
    pub dtype: String,
    /// Explicit unit such as `m`, `rad_s`, or `unitless`.
    pub unit: String,
}

/// Calibration model and finite numeric parameters for one stream.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatasetCalibration {
    /// Versioned calibration model name.
    pub model: String,
    /// Coordinate frame in which the calibration is defined.
    pub reference_frame: String,
    /// Deterministically ordered finite parameters with explicit units in keys.
    pub parameters: BTreeMap<String, f64>,
}

/// Deterministic noise model and seed for one sensor stream.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatasetNoiseSpec {
    /// Versioned noise model name, including `none.v1` for noiseless data.
    pub model: String,
    /// Explicit stream-local seed.
    pub seed: u64,
    /// Deterministically ordered finite model parameters.
    pub parameters: BTreeMap<String, f64>,
}

/// Whether latency is constant or declared independently on each record.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DatasetLatencyModel {
    /// Every frame has exactly `fixed_ticks` latency.
    Fixed,
    /// Each frame declares latency, bounded by `max_ticks`.
    PerFrame,
}

/// Timestamp-to-availability contract for one stream.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatasetLatencySpec {
    /// Latency model.
    pub model: DatasetLatencyModel,
    /// Exact latency for [`DatasetLatencyModel::Fixed`].
    pub fixed_ticks: Option<u64>,
    /// Inclusive maximum latency accepted by the bundle validator.
    pub max_ticks: u64,
}

/// Policy for representing dropped frames.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DatasetGapPolicy {
    /// Every missing sequence must be represented by a `Gap` record.
    ExplicitRecords,
}

/// Simulation-time behavior frozen for a stream.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatasetTimingSpec {
    /// Nominal capture period in simulation nanosecond ticks.
    pub nominal_period_ticks: u64,
    /// Capture-to-availability latency contract.
    pub latency: DatasetLatencySpec,
    /// Dropped-frame representation.
    pub gap_policy: DatasetGapPolicy,
}

/// One stream declared by a dataset manifest.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatasetStreamSpec {
    /// Stable DataBus stream id.
    pub stream_id: StreamId,
    /// Stable human-readable stream name.
    pub name: String,
    /// Semantic stream kind.
    pub kind: DatasetStreamKind,
    /// Versioned payload encoding name.
    pub payload_encoding: String,
    /// Stable source entity name.
    pub source_entity: String,
    /// Coordinate frame of the payload.
    pub frame_id: String,
    /// Ordered public field/unit contract.
    pub fields: Vec<DatasetFieldSpec>,
    /// Required calibration for sensor streams.
    pub calibration: Option<DatasetCalibration>,
    /// Simulation-time and latency behavior.
    pub timing: DatasetTimingSpec,
    /// Required deterministic noise contract for sensor streams.
    pub noise: Option<DatasetNoiseSpec>,
}

/// Content-addressed source asset used to produce the run.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatasetAsset {
    /// Stable asset role such as `robot_model` or `scene`.
    pub role: String,
    /// Portable workspace-relative source path.
    pub path: String,
    /// Lowercase SHA-256 digest with `sha256:` prefix.
    pub sha256: String,
}

/// Typed value for a seeded domain-randomization decision.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum DatasetRandomizationValue {
    /// Finite scalar with an explicit unit.
    Scalar {
        /// Selected value.
        value: f64,
        /// Physical unit.
        unit: String,
    },
    /// Signed integer choice.
    Integer {
        /// Selected value.
        value: i64,
    },
    /// Boolean choice.
    Boolean {
        /// Selected value.
        value: bool,
    },
    /// Bounded textual category.
    Text {
        /// Selected value.
        value: String,
    },
}

/// One reproducible domain-randomization decision.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatasetRandomizationDecision {
    /// Deterministic decision key.
    pub key: String,
    /// Random stream seed used for this decision.
    pub seed: u64,
    /// Selected typed value.
    pub value: DatasetRandomizationValue,
}

/// Per-stream counts and sequence boundary for one shard.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatasetStreamSummary {
    /// Stream identifier.
    pub stream_id: StreamId,
    /// Physical records, including `Gap` records.
    pub record_count: u64,
    /// Materialized payload samples.
    pub sample_count: u64,
    /// Logical dropped sequences declared by gaps.
    pub dropped_count: u64,
    /// First sequence, or `None` for an empty stream.
    pub first_sequence: Option<u64>,
    /// Sequence immediately after the covered interval.
    pub next_sequence: u64,
}

/// One content-addressed binary record shard.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatasetShard {
    /// Portable bundle-relative path.
    pub path: String,
    /// Exact shard length in bytes.
    pub byte_len: u64,
    /// Physical record count.
    pub record_count: u64,
    /// Materialized payload count.
    pub sample_count: u64,
    /// Logical dropped sequence count.
    pub dropped_count: u64,
    /// Lowercase SHA-256 digest with `sha256:` prefix.
    pub sha256: String,
    /// Sorted per-stream summaries.
    pub streams: Vec<DatasetStreamSummary>,
}

/// Versioned manifest for a portable dataset bundle.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatasetManifest {
    /// Must equal [`DATASET_BUNDLE_SCHEMA_VERSION`].
    pub schema_version: u32,
    /// Stable dataset identifier.
    pub dataset_id: String,
    /// TaskSpec content digest used to produce the run.
    pub task_spec_sha256: String,
    /// Simulation fixed step in nanosecond ticks.
    pub fixed_step_ticks: u64,
    /// Root deterministic world seed.
    pub world_seed: u64,
    /// Streams sorted by numeric stream id.
    pub streams: Vec<DatasetStreamSpec>,
    /// Source assets sorted by role then path.
    pub assets: Vec<DatasetAsset>,
    /// Domain-randomization decisions sorted by key.
    pub randomization: Vec<DatasetRandomizationDecision>,
    /// Binary shards; schema v1 contains exactly one.
    pub shards: Vec<DatasetShard>,
    /// Digest of compact canonical JSON with this field empty.
    pub content_sha256: String,
}

impl DatasetManifest {
    /// Creates an unfinished manifest suitable for [`DatasetBundleWriter::create`].
    pub fn new(
        dataset_id: impl Into<String>,
        task_spec_sha256: impl Into<String>,
        fixed_step_ticks: u64,
        world_seed: u64,
        streams: Vec<DatasetStreamSpec>,
    ) -> Self {
        Self {
            schema_version: DATASET_BUNDLE_SCHEMA_VERSION,
            dataset_id: dataset_id.into(),
            task_spec_sha256: task_spec_sha256.into(),
            fixed_step_ticks,
            world_seed,
            streams,
            assets: Vec::new(),
            randomization: Vec::new(),
            shards: Vec::new(),
            content_sha256: String::new(),
        }
    }

    /// Validates metadata, ordering, units, calibration, timing, and digests.
    pub fn validate(&self) -> Result<(), DatasetError> {
        validate_manifest(self, !self.shards.is_empty())
    }

    /// Recomputes the self-excluding manifest content digest.
    pub fn computed_content_sha256(&self) -> Result<String, DatasetError> {
        manifest_digest(self)
    }
}

/// One decoded binary dataset record.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DatasetRecord {
    /// Record kind.
    pub kind: DatasetRecordKind,
    /// Declared stream id.
    pub stream_id: StreamId,
    /// Logical stream sequence.
    pub sequence: u64,
    /// Capture timestamp in simulation nanosecond ticks.
    pub capture_ticks: u64,
    /// Availability timestamp in simulation nanosecond ticks.
    pub available_ticks: u64,
    /// Exact versioned payload bytes.
    pub payload: Vec<u8>,
}

impl DatasetRecord {
    /// Returns the dropped sequence count for a gap record.
    pub fn dropped_count(&self) -> Option<u64> {
        if self.kind != DatasetRecordKind::Gap || self.payload.len() != 8 {
            return None;
        }
        Some(u64::from_le_bytes(self.payload[..8].try_into().ok()?))
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct StreamState {
    record_count: u64,
    sample_count: u64,
    dropped_count: u64,
    first_sequence: Option<u64>,
    next_sequence: u64,
    last_capture_ticks: Option<u64>,
}

impl StreamState {
    fn summary(&self, stream_id: u64) -> DatasetStreamSummary {
        DatasetStreamSummary {
            stream_id: StreamId::new(stream_id),
            record_count: self.record_count,
            sample_count: self.sample_count,
            dropped_count: self.dropped_count,
            first_sequence: self.first_sequence,
            next_sequence: self.next_sequence,
        }
    }
}

/// Streaming writer for one dataset bundle.
pub struct DatasetBundleWriter {
    root: PathBuf,
    manifest: DatasetManifest,
    records: BufWriter<File>,
    shard_hasher: Sha256,
    shard_bytes: u64,
    record_count: u64,
    states: BTreeMap<u64, StreamState>,
}

impl std::fmt::Debug for DatasetBundleWriter {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("DatasetBundleWriter")
            .field("root", &self.root)
            .field("record_count", &self.record_count)
            .finish_non_exhaustive()
    }
}

impl DatasetBundleWriter {
    /// Creates a new bundle directory and writes the versioned shard header.
    ///
    /// The target directory must not already exist, preventing accidental
    /// overwrite of evidence.
    pub fn create(
        root: impl AsRef<Path>,
        mut manifest: DatasetManifest,
    ) -> Result<Self, DatasetError> {
        if !manifest.shards.is_empty() || !manifest.content_sha256.is_empty() {
            return Err(invalid(
                "manifest",
                "writer requires an unfinished manifest",
            ));
        }
        manifest.streams.sort_by_key(|stream| stream.stream_id.0);
        manifest
            .assets
            .sort_by(|left, right| (&left.role, &left.path).cmp(&(&right.role, &right.path)));
        manifest
            .randomization
            .sort_by(|left, right| left.key.cmp(&right.key));
        validate_manifest(&manifest, false)?;

        let root = root.as_ref().to_path_buf();
        fs::create_dir(&root)?;
        let file = File::options()
            .create_new(true)
            .write(true)
            .open(root.join(PARTIAL_SHARD_NAME))?;
        let mut records = BufWriter::new(file);
        let header = file_header();
        records.write_all(&header)?;
        let mut shard_hasher = Sha256::new();
        shard_hasher.update(header);
        let states = manifest
            .streams
            .iter()
            .map(|stream| (stream.stream_id.0, StreamState::default()))
            .collect();
        Ok(Self {
            root,
            manifest,
            records,
            shard_hasher,
            shard_bytes: FILE_HEADER_BYTES as u64,
            record_count: 0,
            states,
        })
    }

    /// Returns the number of physical records written without retaining payloads.
    pub const fn record_count(&self) -> u64 {
        self.record_count
    }

    /// Appends one typed or opaque payload after validating ordering and timing.
    pub fn write_record(&mut self, record: &DatasetRecord) -> Result<(), DatasetError> {
        self.validate_and_advance(record)?;
        self.write_encoded_record(record)
    }

    /// Appends one explicit dropped-sequence interval.
    pub fn write_gap(
        &mut self,
        stream_id: StreamId,
        first_sequence: u64,
        dropped_count: u64,
        capture_ticks: u64,
        available_ticks: u64,
    ) -> Result<(), DatasetError> {
        if dropped_count == 0 {
            return Err(invalid("dropped_count", "must be greater than zero"));
        }
        self.write_record(&DatasetRecord {
            kind: DatasetRecordKind::Gap,
            stream_id,
            sequence: first_sequence,
            capture_ticks,
            available_ticks,
            payload: dropped_count.to_le_bytes().to_vec(),
        })
    }

    /// Encodes and appends one lossless RGBA8 DataBus frame.
    pub fn write_image_rgb8(&mut self, frame: &Frame<ImageRgb8>) -> Result<(), DatasetError> {
        let metadata = sensor_metadata(frame);
        let payload = encode_image_rgb8(metadata, &frame.payload)?;
        self.write_record(&record_from_frame(DatasetRecordKind::Rgb8, frame, payload))
    }

    /// Encodes and appends one linear-depth DataBus frame.
    pub fn write_image_depth(&mut self, frame: &Frame<ImageDepth>) -> Result<(), DatasetError> {
        let metadata = sensor_metadata(frame);
        let payload = encode_image_depth(metadata, &frame.payload)?;
        self.write_record(&record_from_frame(
            DatasetRecordKind::DepthF32,
            frame,
            payload,
        ))
    }

    /// Encodes and appends one LiDAR DataBus frame.
    pub fn write_lidar_point_cloud(
        &mut self,
        frame: &Frame<PointCloud>,
    ) -> Result<(), DatasetError> {
        let metadata = sensor_metadata(frame);
        let payload = encode_lidar_point_cloud(metadata, &frame.payload)?;
        self.write_record(&record_from_frame(
            DatasetRecordKind::LidarPointCloud,
            frame,
            payload,
        ))
    }

    /// Flushes the shard, computes all summaries and hashes, and atomically
    /// publishes the final shard and manifest names.
    pub fn finish(mut self) -> Result<DatasetManifest, DatasetError> {
        self.records.flush()?;
        self.records.get_ref().sync_all()?;
        drop(self.records);

        let shard_digest = format!("sha256:{:x}", self.shard_hasher.finalize());
        let streams = self
            .states
            .iter()
            .map(|(stream_id, state)| state.summary(*stream_id))
            .collect::<Vec<_>>();
        let sample_count = streams.iter().map(|stream| stream.sample_count).sum();
        let dropped_count = streams.iter().map(|stream| stream.dropped_count).sum();
        self.manifest.shards.push(DatasetShard {
            path: SHARD_NAME.to_string(),
            byte_len: self.shard_bytes,
            record_count: self.record_count,
            sample_count,
            dropped_count,
            sha256: shard_digest,
            streams,
        });
        self.manifest.content_sha256 = manifest_digest(&self.manifest)?;
        validate_manifest(&self.manifest, true)?;

        fs::rename(
            self.root.join(PARTIAL_SHARD_NAME),
            self.root.join(SHARD_NAME),
        )?;
        let manifest_path = self.root.join(PARTIAL_MANIFEST_NAME);
        let mut bytes = serde_json::to_vec_pretty(&self.manifest)?;
        bytes.push(b'\n');
        {
            let mut file = File::options()
                .create_new(true)
                .write(true)
                .open(&manifest_path)?;
            file.write_all(&bytes)?;
            file.sync_all()?;
        }
        fs::rename(manifest_path, self.root.join(MANIFEST_NAME))?;
        Ok(self.manifest)
    }

    fn validate_and_advance(&mut self, record: &DatasetRecord) -> Result<(), DatasetError> {
        let stream = self
            .manifest
            .streams
            .iter()
            .find(|stream| stream.stream_id == record.stream_id)
            .ok_or(DatasetError::UnknownStream(record.stream_id.0))?;
        if record.kind != DatasetRecordKind::Gap && !stream.kind.accepts(record.kind) {
            return Err(DatasetError::StreamKindMismatch {
                stream_id: record.stream_id.0,
                declared: stream.kind,
                actual: record.kind,
            });
        }
        validate_record_timing(stream, record)?;
        validate_payload(record)?;
        let state = self
            .states
            .get_mut(&record.stream_id.0)
            .expect("declared streams initialize state");
        advance_state(state, record)?;
        Ok(())
    }

    fn write_encoded_record(&mut self, record: &DatasetRecord) -> Result<(), DatasetError> {
        let payload_len = u64::try_from(record.payload.len())
            .map_err(|_| invalid("payload_len", "does not fit u64"))?;
        let payload_digest: [u8; 32] = Sha256::digest(&record.payload).into();
        let mut header = [0_u8; RECORD_HEADER_BYTES];
        header[0..2].copy_from_slice(&(record.kind as u16).to_le_bytes());
        header[8..16].copy_from_slice(&record.stream_id.0.to_le_bytes());
        header[16..24].copy_from_slice(&record.sequence.to_le_bytes());
        header[24..32].copy_from_slice(&record.capture_ticks.to_le_bytes());
        header[32..40].copy_from_slice(&record.available_ticks.to_le_bytes());
        header[40..48].copy_from_slice(&payload_len.to_le_bytes());
        header[48..80].copy_from_slice(&payload_digest);
        self.records.write_all(&header)?;
        self.records.write_all(&record.payload)?;
        self.shard_hasher.update(header);
        self.shard_hasher.update(&record.payload);
        self.shard_bytes = self
            .shard_bytes
            .checked_add(RECORD_HEADER_BYTES as u64)
            .and_then(|value| value.checked_add(payload_len))
            .ok_or_else(|| invalid("shard.byte_len", "overflow"))?;
        self.record_count = self
            .record_count
            .checked_add(1)
            .ok_or_else(|| invalid("shard.record_count", "overflow"))?;
        Ok(())
    }
}

/// Open, immutable dataset bundle with validated manifest metadata.
#[derive(Clone, Debug)]
pub struct DatasetBundle {
    root: PathBuf,
    manifest: DatasetManifest,
}

impl DatasetBundle {
    /// Opens and validates a bundle manifest without loading the shard payloads.
    pub fn open(root: impl AsRef<Path>) -> Result<Self, DatasetError> {
        let root = root.as_ref().to_path_buf();
        let path = root.join(MANIFEST_NAME);
        let metadata = fs::metadata(&path)?;
        if metadata.len() > DATASET_MAX_MANIFEST_BYTES {
            return Err(invalid(
                "manifest",
                format!("exceeds {DATASET_MAX_MANIFEST_BYTES} bytes"),
            ));
        }
        let bytes = fs::read(path)?;
        let manifest: DatasetManifest = serde_json::from_slice(&bytes)?;
        validate_manifest(&manifest, true)?;
        if manifest.content_sha256 != manifest_digest(&manifest)? {
            return Err(DatasetError::DigestMismatch(MANIFEST_NAME.to_string()));
        }
        let shard = &manifest.shards[0];
        if fs::metadata(root.join(SHARD_NAME))?.len() != shard.byte_len {
            return Err(DatasetError::DigestMismatch(SHARD_NAME.to_string()));
        }
        Ok(Self { root, manifest })
    }

    /// Returns the validated manifest.
    pub const fn manifest(&self) -> &DatasetManifest {
        &self.manifest
    }

    /// Creates a fresh streaming reader over the shard.
    pub fn records(&self) -> Result<DatasetRecordReader, DatasetError> {
        DatasetRecordReader::open(&self.root, &self.manifest)
    }

    /// Streams through all records and returns stable verified counts.
    pub fn verify(&self) -> Result<DatasetVerificationReport, DatasetError> {
        let mut observed = 0_u64;
        for record in self.records()? {
            record?;
            observed = observed
                .checked_add(1)
                .ok_or_else(|| invalid("verification.record_count", "overflow"))?;
        }
        let shard = &self.manifest.shards[0];
        Ok(DatasetVerificationReport {
            schema_version: DATASET_BUNDLE_SCHEMA_VERSION,
            manifest_sha256: self.manifest.content_sha256.clone(),
            stream_count: self.manifest.streams.len() as u64,
            record_count: observed,
            sample_count: shard.sample_count,
            dropped_count: shard.dropped_count,
            passed: true,
        })
    }
}

/// Stable result returned only after complete bundle verification.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatasetVerificationReport {
    /// Bundle schema verified.
    pub schema_version: u32,
    /// Self-excluding manifest digest.
    pub manifest_sha256: String,
    /// Number of declared streams.
    pub stream_count: u64,
    /// Physical records verified.
    pub record_count: u64,
    /// Materialized payload samples verified.
    pub sample_count: u64,
    /// Explicitly dropped logical samples verified.
    pub dropped_count: u64,
    /// Always true because failures return [`DatasetError`].
    pub passed: bool,
}

/// Streaming, hash-verifying iterator over dataset records.
pub struct DatasetRecordReader {
    reader: BufReader<File>,
    streams: Vec<DatasetStreamSpec>,
    expected_shard: DatasetShard,
    shard_hasher: Sha256,
    shard_bytes: u64,
    record_count: u64,
    states: BTreeMap<u64, StreamState>,
    finished: bool,
}

impl std::fmt::Debug for DatasetRecordReader {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("DatasetRecordReader")
            .field("record_count", &self.record_count)
            .field("finished", &self.finished)
            .finish_non_exhaustive()
    }
}

impl DatasetRecordReader {
    fn open(root: &Path, manifest: &DatasetManifest) -> Result<Self, DatasetError> {
        let mut reader = BufReader::new(File::open(root.join(SHARD_NAME))?);
        let mut header = [0_u8; FILE_HEADER_BYTES];
        reader.read_exact(&mut header).map_err(map_exact_error)?;
        if header != file_header() {
            return Err(DatasetError::InvalidFileHeader);
        }
        let mut shard_hasher = Sha256::new();
        shard_hasher.update(header);
        let states = manifest
            .streams
            .iter()
            .map(|stream| (stream.stream_id.0, StreamState::default()))
            .collect();
        Ok(Self {
            reader,
            streams: manifest.streams.clone(),
            expected_shard: manifest.shards[0].clone(),
            shard_hasher,
            shard_bytes: FILE_HEADER_BYTES as u64,
            record_count: 0,
            states,
            finished: false,
        })
    }

    fn read_next(&mut self) -> Result<Option<DatasetRecord>, DatasetError> {
        let mut header = [0_u8; RECORD_HEADER_BYTES];
        if !read_exact_or_eof(&mut self.reader, &mut header)? {
            self.finish_validation()?;
            return Ok(None);
        }
        if header[2..8].iter().any(|byte| *byte != 0) {
            return Err(invalid("record.reserved", "must be zero"));
        }
        let kind = DatasetRecordKind::try_from(u16::from_le_bytes(
            header[0..2].try_into().expect("fixed slice"),
        ))?;
        let stream_id = StreamId::new(read_u64(&header[8..16]));
        let sequence = read_u64(&header[16..24]);
        let capture_ticks = read_u64(&header[24..32]);
        let available_ticks = read_u64(&header[32..40]);
        let payload_len = read_u64(&header[40..48]);
        let payload_len = usize::try_from(payload_len)
            .map_err(|_| invalid("record.payload_len", "does not fit usize"))?;
        if payload_len > TRANSPORT_MAX_PAYLOAD_BYTES {
            return Err(invalid(
                "record.payload_len",
                format!("exceeds {TRANSPORT_MAX_PAYLOAD_BYTES} bytes"),
            ));
        }
        let mut payload = vec![0_u8; payload_len];
        self.reader
            .read_exact(&mut payload)
            .map_err(map_exact_error)?;
        let actual_payload_digest: [u8; 32] = Sha256::digest(&payload).into();
        if header[48..80] != actual_payload_digest {
            return Err(DatasetError::DigestMismatch(format!(
                "stream {} sequence {}",
                stream_id.0, sequence
            )));
        }
        let record = DatasetRecord {
            kind,
            stream_id,
            sequence,
            capture_ticks,
            available_ticks,
            payload,
        };
        self.validate_and_advance(&record)?;
        self.shard_hasher.update(header);
        self.shard_hasher.update(&record.payload);
        self.shard_bytes = self
            .shard_bytes
            .checked_add(RECORD_HEADER_BYTES as u64)
            .and_then(|value| value.checked_add(record.payload.len() as u64))
            .ok_or_else(|| invalid("shard.byte_len", "overflow"))?;
        self.record_count = self
            .record_count
            .checked_add(1)
            .ok_or_else(|| invalid("shard.record_count", "overflow"))?;
        Ok(Some(record))
    }

    fn validate_and_advance(&mut self, record: &DatasetRecord) -> Result<(), DatasetError> {
        let stream = self
            .streams
            .iter()
            .find(|stream| stream.stream_id == record.stream_id)
            .ok_or(DatasetError::UnknownStream(record.stream_id.0))?;
        if record.kind != DatasetRecordKind::Gap && !stream.kind.accepts(record.kind) {
            return Err(DatasetError::StreamKindMismatch {
                stream_id: record.stream_id.0,
                declared: stream.kind,
                actual: record.kind,
            });
        }
        validate_record_timing(stream, record)?;
        validate_payload(record)?;
        let state = self
            .states
            .get_mut(&record.stream_id.0)
            .expect("declared streams initialize state");
        advance_state(state, record)
    }

    fn finish_validation(&mut self) -> Result<(), DatasetError> {
        if self.finished {
            return Ok(());
        }
        self.finished = true;
        if self.shard_bytes != self.expected_shard.byte_len
            || self.record_count != self.expected_shard.record_count
        {
            return Err(DatasetError::TrailingBytes);
        }
        let digest = format!("sha256:{:x}", self.shard_hasher.clone().finalize());
        if digest != self.expected_shard.sha256 {
            return Err(DatasetError::DigestMismatch(SHARD_NAME.to_string()));
        }
        let summaries = self
            .states
            .iter()
            .map(|(stream_id, state)| state.summary(*stream_id))
            .collect::<Vec<_>>();
        if summaries != self.expected_shard.streams {
            return Err(invalid(
                "shard.streams",
                "record counts or sequence boundaries do not match manifest",
            ));
        }
        Ok(())
    }
}

impl Iterator for DatasetRecordReader {
    type Item = Result<DatasetRecord, DatasetError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.finished {
            return None;
        }
        match self.read_next() {
            Ok(Some(record)) => Some(Ok(record)),
            Ok(None) => None,
            Err(error) => {
                self.finished = true;
                Some(Err(error))
            }
        }
    }
}

fn validate_manifest(manifest: &DatasetManifest, finalized: bool) -> Result<(), DatasetError> {
    if manifest.schema_version != DATASET_BUNDLE_SCHEMA_VERSION {
        return Err(invalid(
            "schema_version",
            format!("must be {DATASET_BUNDLE_SCHEMA_VERSION}"),
        ));
    }
    validate_identifier("dataset_id", &manifest.dataset_id)?;
    validate_sha256("task_spec_sha256", &manifest.task_spec_sha256)?;
    if manifest.fixed_step_ticks == 0 {
        return Err(invalid("fixed_step_ticks", "must be greater than zero"));
    }
    if manifest.streams.is_empty() || manifest.streams.len() > DATASET_MAX_STREAMS {
        return Err(invalid(
            "streams",
            format!("must contain 1..={DATASET_MAX_STREAMS} entries"),
        ));
    }
    let mut previous_stream = None;
    let mut names = BTreeSet::new();
    for stream in &manifest.streams {
        if previous_stream.is_some_and(|previous| previous >= stream.stream_id.0) {
            return Err(invalid(
                "streams",
                "must be strictly sorted by unique stream_id",
            ));
        }
        previous_stream = Some(stream.stream_id.0);
        validate_stream(stream)?;
        if !names.insert(stream.name.as_str()) {
            return Err(invalid("streams.name", "must be unique"));
        }
    }
    let mut previous_asset: Option<(&str, &str)> = None;
    for asset in &manifest.assets {
        validate_identifier("assets.role", &asset.role)?;
        validate_relative_path("assets.path", &asset.path)?;
        validate_sha256("assets.sha256", &asset.sha256)?;
        let current = (asset.role.as_str(), asset.path.as_str());
        if previous_asset.is_some_and(|previous| previous >= current) {
            return Err(invalid(
                "assets",
                "must be strictly sorted by unique role and path",
            ));
        }
        previous_asset = Some(current);
    }
    let mut previous_randomization = None;
    for decision in &manifest.randomization {
        validate_identifier("randomization.key", &decision.key)?;
        if previous_randomization.is_some_and(|previous: &str| previous >= decision.key.as_str()) {
            return Err(invalid(
                "randomization",
                "must be strictly sorted by unique key",
            ));
        }
        previous_randomization = Some(decision.key.as_str());
        match &decision.value {
            DatasetRandomizationValue::Scalar { value, unit } => {
                if !value.is_finite() {
                    return Err(invalid("randomization.value", "must be finite"));
                }
                validate_text("randomization.unit", unit)?;
            }
            DatasetRandomizationValue::Text { value } => {
                validate_text("randomization.value", value)?;
            }
            DatasetRandomizationValue::Integer { .. }
            | DatasetRandomizationValue::Boolean { .. } => {}
        }
    }
    if finalized {
        if manifest.shards.len() != 1 {
            return Err(invalid("shards", "schema v1 requires exactly one shard"));
        }
        validate_shard(&manifest.shards[0], &manifest.streams)?;
        validate_sha256("content_sha256", &manifest.content_sha256)?;
    } else if !manifest.shards.is_empty() || !manifest.content_sha256.is_empty() {
        return Err(invalid(
            "manifest",
            "unfinished manifest cannot contain shards or content_sha256",
        ));
    }
    Ok(())
}

fn validate_stream(stream: &DatasetStreamSpec) -> Result<(), DatasetError> {
    validate_identifier("streams.name", &stream.name)?;
    validate_text("streams.payload_encoding", &stream.payload_encoding)?;
    let expected_encoding = match stream.kind {
        DatasetStreamKind::Rgb8 => Some("rne.transport.image_rgb8.v1"),
        DatasetStreamKind::DepthF32 => Some("rne.transport.image_depth_f32.v1"),
        DatasetStreamKind::LidarPointCloud => Some("rne.transport.lidar_point_cloud.v1"),
        DatasetStreamKind::Imu
        | DatasetStreamKind::Transform
        | DatasetStreamKind::Action
        | DatasetStreamKind::TaskOutcome
        | DatasetStreamKind::GroundTruthAnnotation => None,
    };
    if let Some(expected) = expected_encoding {
        if stream.payload_encoding != expected {
            return Err(invalid(
                "streams.payload_encoding",
                format!("must be {expected} for typed transport payload"),
            ));
        }
    }
    validate_text("streams.source_entity", &stream.source_entity)?;
    validate_text("streams.frame_id", &stream.frame_id)?;
    if stream.fields.is_empty() {
        return Err(invalid("streams.fields", "must not be empty"));
    }
    let mut field_names = BTreeSet::new();
    for field in &stream.fields {
        validate_identifier("streams.fields.name", &field.name)?;
        validate_text("streams.fields.dtype", &field.dtype)?;
        validate_text("streams.fields.unit", &field.unit)?;
        if !field_names.insert(field.name.as_str()) {
            return Err(invalid("streams.fields.name", "must be unique"));
        }
    }
    if stream.timing.nominal_period_ticks == 0 {
        return Err(invalid(
            "streams.timing.nominal_period_ticks",
            "must be greater than zero",
        ));
    }
    match stream.timing.latency.model {
        DatasetLatencyModel::Fixed => {
            let fixed = stream.timing.latency.fixed_ticks.ok_or_else(|| {
                invalid(
                    "streams.timing.latency.fixed_ticks",
                    "is required for fixed latency",
                )
            })?;
            if fixed != stream.timing.latency.max_ticks {
                return Err(invalid(
                    "streams.timing.latency.max_ticks",
                    "must equal fixed_ticks for fixed latency",
                ));
            }
        }
        DatasetLatencyModel::PerFrame => {
            if stream.timing.latency.fixed_ticks.is_some() {
                return Err(invalid(
                    "streams.timing.latency.fixed_ticks",
                    "must be absent for per-frame latency",
                ));
            }
        }
    }
    if stream.kind.is_sensor() {
        let calibration = stream
            .calibration
            .as_ref()
            .ok_or_else(|| invalid("streams.calibration", "is required for sensor streams"))?;
        validate_text("streams.calibration.model", &calibration.model)?;
        validate_text(
            "streams.calibration.reference_frame",
            &calibration.reference_frame,
        )?;
        validate_parameters("streams.calibration.parameters", &calibration.parameters)?;
        let noise = stream
            .noise
            .as_ref()
            .ok_or_else(|| invalid("streams.noise", "is required for sensor streams"))?;
        validate_text("streams.noise.model", &noise.model)?;
        validate_parameters("streams.noise.parameters", &noise.parameters)?;
    } else if stream.calibration.is_some() || stream.noise.is_some() {
        return Err(invalid(
            "streams",
            "non-sensor streams cannot declare sensor calibration or noise",
        ));
    }
    Ok(())
}

fn validate_shard(shard: &DatasetShard, streams: &[DatasetStreamSpec]) -> Result<(), DatasetError> {
    if shard.path != SHARD_NAME {
        return Err(invalid(
            "shards.path",
            format!("schema v1 requires {SHARD_NAME}"),
        ));
    }
    if shard.byte_len < FILE_HEADER_BYTES as u64 {
        return Err(invalid("shards.byte_len", "is smaller than file header"));
    }
    validate_sha256("shards.sha256", &shard.sha256)?;
    if shard.streams.len() != streams.len() {
        return Err(invalid(
            "shards.streams",
            "must summarize every declared stream",
        ));
    }
    for (summary, stream) in shard.streams.iter().zip(streams) {
        if summary.stream_id != stream.stream_id {
            return Err(invalid(
                "shards.streams.stream_id",
                "must match declared stream order",
            ));
        }
        let gap_records = summary
            .record_count
            .checked_sub(summary.sample_count)
            .ok_or_else(|| {
                invalid(
                    "shards.streams.record_count",
                    "cannot be smaller than sample_count",
                )
            })?;
        if (summary.dropped_count == 0 && gap_records != 0)
            || (summary.dropped_count > 0
                && (gap_records == 0 || gap_records > summary.dropped_count))
        {
            return Err(invalid(
                "shards.streams.record_count",
                "gap record count is inconsistent with dropped_count",
            ));
        }
        if summary.first_sequence.is_none()
            && (summary.record_count != 0
                || summary.sample_count != 0
                || summary.dropped_count != 0
                || summary.next_sequence != 0)
        {
            return Err(invalid(
                "shards.streams.first_sequence",
                "empty stream summary contains counts",
            ));
        }
        if summary.first_sequence.is_some() && summary.next_sequence == 0 {
            return Err(invalid(
                "shards.streams.next_sequence",
                "covered stream must advance sequence",
            ));
        }
    }
    if shard.record_count
        != shard
            .streams
            .iter()
            .map(|item| item.record_count)
            .sum::<u64>()
        || shard.sample_count
            != shard
                .streams
                .iter()
                .map(|item| item.sample_count)
                .sum::<u64>()
        || shard.dropped_count
            != shard
                .streams
                .iter()
                .map(|item| item.dropped_count)
                .sum::<u64>()
    {
        return Err(invalid(
            "shards",
            "aggregate counts do not match stream summaries",
        ));
    }
    Ok(())
}

fn validate_record_timing(
    stream: &DatasetStreamSpec,
    record: &DatasetRecord,
) -> Result<(), DatasetError> {
    let latency = record
        .available_ticks
        .checked_sub(record.capture_ticks)
        .ok_or_else(|| invalid("record.available_ticks", "precedes capture_ticks"))?;
    match stream.timing.latency.model {
        DatasetLatencyModel::Fixed => {
            if Some(latency) != stream.timing.latency.fixed_ticks {
                return Err(invalid(
                    "record.available_ticks",
                    format!("latency {latency} does not match fixed contract"),
                ));
            }
        }
        DatasetLatencyModel::PerFrame => {
            if latency > stream.timing.latency.max_ticks {
                return Err(invalid(
                    "record.available_ticks",
                    format!("latency {latency} exceeds maximum"),
                ));
            }
        }
    }
    Ok(())
}

fn validate_payload(record: &DatasetRecord) -> Result<(), DatasetError> {
    if record.payload.len() > TRANSPORT_MAX_PAYLOAD_BYTES {
        return Err(invalid(
            "record.payload",
            format!("exceeds {TRANSPORT_MAX_PAYLOAD_BYTES} bytes"),
        ));
    }
    if record.kind == DatasetRecordKind::Gap {
        if record.dropped_count().is_none_or(|count| count == 0) {
            return Err(invalid(
                "record.gap",
                "payload must contain one non-zero u64 count",
            ));
        }
        return Ok(());
    }
    let expected = SensorFrameMetadata {
        stream_id: record.stream_id.0,
        sensor_sequence: record.sequence,
        capture_ticks: record.capture_ticks,
        available_ticks: record.available_ticks,
    };
    let embedded = match record.kind {
        DatasetRecordKind::Rgb8 => Some(decode_image_rgb8(&record.payload)?.0),
        DatasetRecordKind::DepthF32 => Some(decode_image_depth(&record.payload)?.0),
        DatasetRecordKind::LidarPointCloud => Some(decode_lidar_point_cloud(&record.payload)?.0),
        DatasetRecordKind::Gap => unreachable!("gap returned above"),
        DatasetRecordKind::Imu
        | DatasetRecordKind::Transform
        | DatasetRecordKind::Action
        | DatasetRecordKind::TaskOutcome
        | DatasetRecordKind::GroundTruthAnnotation => None,
    };
    if embedded.is_some_and(|metadata| metadata != expected) {
        return Err(invalid(
            "record.payload.metadata",
            "does not match record header",
        ));
    }
    Ok(())
}

fn advance_state(state: &mut StreamState, record: &DatasetRecord) -> Result<(), DatasetError> {
    if record.sequence != state.next_sequence {
        return Err(DatasetError::SequenceMismatch {
            stream_id: record.stream_id.0,
            expected: state.next_sequence,
            actual: record.sequence,
        });
    }
    if let Some(previous) = state.last_capture_ticks {
        if record.capture_ticks < previous {
            return Err(DatasetError::CaptureTimeRegression {
                stream_id: record.stream_id.0,
                previous,
                actual: record.capture_ticks,
            });
        }
    }
    let advance = record.dropped_count().unwrap_or(1);
    state.next_sequence = state
        .next_sequence
        .checked_add(advance)
        .ok_or_else(|| invalid("record.sequence", "overflow"))?;
    state.first_sequence.get_or_insert(record.sequence);
    state.record_count = state
        .record_count
        .checked_add(1)
        .ok_or_else(|| invalid("stream.record_count", "overflow"))?;
    if record.kind == DatasetRecordKind::Gap {
        state.dropped_count = state
            .dropped_count
            .checked_add(advance)
            .ok_or_else(|| invalid("stream.dropped_count", "overflow"))?;
    } else {
        state.sample_count = state
            .sample_count
            .checked_add(1)
            .ok_or_else(|| invalid("stream.sample_count", "overflow"))?;
    }
    state.last_capture_ticks = Some(record.capture_ticks);
    Ok(())
}

fn sensor_metadata<T: crate::FramePayload>(frame: &Frame<T>) -> SensorFrameMetadata {
    SensorFrameMetadata {
        stream_id: frame.stream_id.0,
        sensor_sequence: frame.sequence,
        capture_ticks: frame.capture_time.ticks(),
        available_ticks: frame.available_time.ticks(),
    }
}

fn record_from_frame<T: crate::FramePayload>(
    kind: DatasetRecordKind,
    frame: &Frame<T>,
    payload: Vec<u8>,
) -> DatasetRecord {
    DatasetRecord {
        kind,
        stream_id: frame.stream_id,
        sequence: frame.sequence,
        capture_ticks: frame.capture_time.ticks(),
        available_ticks: frame.available_time.ticks(),
        payload,
    }
}

fn validate_identifier(field: &'static str, value: &str) -> Result<(), DatasetError> {
    validate_text(field, value)?;
    if value.len() > 128
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'_' | b'-'))
    {
        return Err(invalid(
            field,
            "must be at most 128 ASCII identifier characters",
        ));
    }
    Ok(())
}

fn validate_text(field: &'static str, value: &str) -> Result<(), DatasetError> {
    if value.is_empty() || value.len() > 1024 || value.chars().any(char::is_control) {
        return Err(invalid(
            field,
            "must contain 1..=1024 non-control UTF-8 bytes",
        ));
    }
    Ok(())
}

fn validate_relative_path(field: &'static str, value: &str) -> Result<(), DatasetError> {
    validate_text(field, value)?;
    if value.contains('\\')
        || value.starts_with('/')
        || value.contains(':')
        || value
            .split('/')
            .any(|component| component.is_empty() || matches!(component, "." | ".."))
    {
        return Err(invalid(field, "must be a canonical relative slash path"));
    }
    Ok(())
}

fn validate_sha256(field: &'static str, value: &str) -> Result<(), DatasetError> {
    let hex = value.strip_prefix("sha256:").unwrap_or_default();
    if hex.len() != 64
        || !hex
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(invalid(
            field,
            "must be lowercase sha256: followed by 64 hexadecimal digits",
        ));
    }
    Ok(())
}

fn validate_parameters(
    field: &'static str,
    parameters: &BTreeMap<String, f64>,
) -> Result<(), DatasetError> {
    if parameters.len() > 1024 {
        return Err(invalid(field, "contains more than 1024 entries"));
    }
    for (key, value) in parameters {
        validate_identifier(field, key)?;
        if !value.is_finite() {
            return Err(invalid(field, format!("parameter {key} is not finite")));
        }
    }
    Ok(())
}

fn manifest_digest(manifest: &DatasetManifest) -> Result<String, DatasetError> {
    let mut canonical = manifest.clone();
    canonical.content_sha256.clear();
    Ok(format!(
        "sha256:{:x}",
        Sha256::digest(serde_json::to_vec(&canonical)?)
    ))
}

fn file_header() -> [u8; FILE_HEADER_BYTES] {
    let mut header = [0_u8; FILE_HEADER_BYTES];
    header[..8].copy_from_slice(&FILE_MAGIC);
    header[8..12].copy_from_slice(&DATASET_BUNDLE_SCHEMA_VERSION.to_le_bytes());
    header[12..16].copy_from_slice(&(FILE_HEADER_BYTES as u32).to_le_bytes());
    header
}

fn read_exact_or_eof(reader: &mut impl Read, bytes: &mut [u8]) -> Result<bool, DatasetError> {
    let mut first = [0_u8; 1];
    loop {
        match reader.read(&mut first) {
            Ok(0) => return Ok(false),
            Ok(1) => break,
            Ok(_) => unreachable!("one-byte read returned more than one byte"),
            Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
            Err(error) => return Err(error.into()),
        }
    }
    bytes[0] = first[0];
    reader
        .read_exact(&mut bytes[1..])
        .map_err(map_exact_error)?;
    Ok(true)
}

fn map_exact_error(error: io::Error) -> DatasetError {
    if error.kind() == io::ErrorKind::UnexpectedEof {
        DatasetError::Truncated
    } else {
        DatasetError::Io(error)
    }
}

fn read_u64(bytes: &[u8]) -> u64 {
    u64::from_le_bytes(bytes.try_into().expect("fixed eight-byte field"))
}

fn invalid(field: &'static str, reason: impl Into<String>) -> DatasetError {
    DatasetError::InvalidField {
        field,
        reason: reason.into(),
    }
}
