//! Renderer-free offline metrics over verified dataset bundle records.

use crate::dataset::{DatasetBundle, DatasetError, DatasetRecordKind, DatasetRecordReader};
use crate::transport::decode_image_depth;
use crate::{ImageDepth, StreamId};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::fs::File;
use std::io::Write;
use std::path::Path;

/// Current schema for committed offline dataset evaluation reports.
pub const DATASET_OFFLINE_EVALUATION_SCHEMA_VERSION: u32 = 1;

/// Configuration for deterministic depth-pair accuracy evaluation.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DepthPairMetricSpec {
    /// Sensor or prediction depth stream.
    pub predicted_stream: StreamId,
    /// Reference depth stream captured at matching sequences and times.
    pub ground_truth_stream: StreamId,
    /// Inclusive maximum-absolute-error acceptance threshold in metres.
    pub tolerance_m: f64,
}

impl DepthPairMetricSpec {
    /// Validates stream identity and finite tolerance.
    pub fn validate(self) -> Result<(), DatasetError> {
        if self.predicted_stream == self.ground_truth_stream {
            return Err(invalid(
                "predicted_stream",
                "must differ from ground_truth_stream",
            ));
        }
        if !self.tolerance_m.is_finite() || self.tolerance_m < 0.0 {
            return Err(invalid("tolerance_m", "must be finite and non-negative"));
        }
        Ok(())
    }
}

/// Versioned, content-addressed result of one depth-pair evaluation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DepthPairEvaluationReport {
    /// Must equal [`DATASET_OFFLINE_EVALUATION_SCHEMA_VERSION`].
    pub schema_version: u32,
    /// Verified dataset manifest digest.
    pub dataset_manifest_sha256: String,
    /// Evaluated prediction stream.
    pub predicted_stream: StreamId,
    /// Evaluated ground-truth stream.
    pub ground_truth_stream: StreamId,
    /// Matched materialized frame pairs.
    pub compared_frames: u64,
    /// Matched depth values.
    pub compared_pixels: u64,
    /// Sequences explicitly dropped in both streams.
    pub dropped_pairs: u64,
    /// Mean absolute depth error in metres.
    pub mean_absolute_error_m: f64,
    /// Root mean square depth error in metres.
    pub root_mean_square_error_m: f64,
    /// Maximum absolute depth error in metres.
    pub max_absolute_error_m: f64,
    /// Inclusive acceptance threshold in metres.
    pub tolerance_m: f64,
    /// Whether maximum absolute error met the threshold.
    pub passed: bool,
    /// SHA-256 of compact JSON with this field empty.
    pub content_sha256: String,
}

impl DepthPairEvaluationReport {
    /// Validates all derived fields and the self-excluding content digest.
    pub fn validate(&self) -> Result<(), DatasetError> {
        if self.schema_version != DATASET_OFFLINE_EVALUATION_SCHEMA_VERSION {
            return Err(invalid(
                "schema_version",
                format!("must be {DATASET_OFFLINE_EVALUATION_SCHEMA_VERSION}"),
            ));
        }
        DepthPairMetricSpec {
            predicted_stream: self.predicted_stream,
            ground_truth_stream: self.ground_truth_stream,
            tolerance_m: self.tolerance_m,
        }
        .validate()?;
        validate_sha256("dataset_manifest_sha256", &self.dataset_manifest_sha256)?;
        if self.compared_frames == 0 || self.compared_pixels == 0 {
            return Err(invalid(
                "compared_frames",
                "evaluation must compare at least one pixel",
            ));
        }
        for (field, value) in [
            ("mean_absolute_error_m", self.mean_absolute_error_m),
            ("root_mean_square_error_m", self.root_mean_square_error_m),
            ("max_absolute_error_m", self.max_absolute_error_m),
        ] {
            if !value.is_finite() || value < 0.0 {
                return Err(invalid(field, "must be finite and non-negative"));
            }
        }
        if self.mean_absolute_error_m > self.max_absolute_error_m
            || self.root_mean_square_error_m > self.max_absolute_error_m
            || self.passed != (self.max_absolute_error_m <= self.tolerance_m)
        {
            return Err(invalid(
                "metrics",
                "aggregates or pass verdict are inconsistent",
            ));
        }
        if self.content_sha256 != report_digest(self)? {
            return Err(DatasetError::DigestMismatch(
                "offline evaluation report".to_string(),
            ));
        }
        Ok(())
    }

    /// Writes validated pretty JSON without overwriting an existing report.
    pub fn write_json(&self, path: impl AsRef<Path>) -> Result<(), DatasetError> {
        self.validate()?;
        let mut bytes = serde_json::to_vec_pretty(self)?;
        bytes.push(b'\n');
        let mut file = File::options().create_new(true).write(true).open(path)?;
        file.write_all(&bytes)?;
        file.sync_all()?;
        Ok(())
    }
}

impl DatasetBundle {
    /// Evaluates matching depth frames with two streaming shard scans.
    ///
    /// No renderer is initialized, and at most one decoded frame from each
    /// selected stream is retained at a time.
    pub fn evaluate_depth_pair(
        &self,
        spec: DepthPairMetricSpec,
    ) -> Result<DepthPairEvaluationReport, DatasetError> {
        spec.validate()?;
        for stream_id in [spec.predicted_stream, spec.ground_truth_stream] {
            let stream = self
                .manifest()
                .streams
                .iter()
                .find(|stream| stream.stream_id == stream_id)
                .ok_or(DatasetError::UnknownStream(stream_id.0))?;
            if stream.kind != crate::DatasetStreamKind::DepthF32 {
                return Err(invalid(
                    "depth_stream",
                    format!("stream {} is not depth_f32", stream_id.0),
                ));
            }
        }

        let mut predicted = DepthEvents::new(self.records()?, spec.predicted_stream);
        let mut truth = DepthEvents::new(self.records()?, spec.ground_truth_stream);
        let mut compared_frames = 0_u64;
        let mut compared_pixels = 0_u64;
        let mut dropped_pairs = 0_u64;
        let mut absolute_sum = 0.0_f64;
        let mut squared_sum = 0.0_f64;
        let mut maximum = 0.0_f64;

        loop {
            match (predicted.next_event()?, truth.next_event()?) {
                (None, None) => break,
                (Some(DepthEvent::Gap(left)), Some(DepthEvent::Gap(right))) => {
                    if left != right {
                        return Err(invalid(
                            "depth_gap",
                            "prediction and ground truth gaps do not match",
                        ));
                    }
                    dropped_pairs = dropped_pairs
                        .checked_add(left.count)
                        .ok_or_else(|| invalid("dropped_pairs", "overflow"))?;
                }
                (Some(DepthEvent::Sample(left)), Some(DepthEvent::Sample(right))) => {
                    if left.sequence != right.sequence
                        || left.capture_ticks != right.capture_ticks
                        || left.image.width != right.image.width
                        || left.image.height != right.image.height
                        || left.image.depth_m.len() != right.image.depth_m.len()
                    {
                        return Err(invalid(
                            "depth_pair",
                            "sequence, capture time, or dimensions do not match",
                        ));
                    }
                    for (predicted_m, truth_m) in
                        left.image.depth_m.iter().zip(&right.image.depth_m)
                    {
                        if *predicted_m < 0.0 || *truth_m < 0.0 {
                            return Err(invalid("depth_m", "must be non-negative"));
                        }
                        let error = f64::from((*predicted_m - *truth_m).abs());
                        absolute_sum += error;
                        squared_sum += error * error;
                        maximum = maximum.max(error);
                    }
                    compared_pixels = compared_pixels
                        .checked_add(left.image.depth_m.len() as u64)
                        .ok_or_else(|| invalid("compared_pixels", "overflow"))?;
                    compared_frames = compared_frames
                        .checked_add(1)
                        .ok_or_else(|| invalid("compared_frames", "overflow"))?;
                }
                (None, Some(_)) | (Some(_), None) => {
                    return Err(invalid(
                        "depth_pair",
                        "prediction and ground truth stream lengths differ",
                    ));
                }
                _ => {
                    return Err(invalid(
                        "depth_pair",
                        "sample and explicit gap do not align",
                    ));
                }
            }
        }
        if compared_pixels == 0 {
            return Err(invalid(
                "compared_pixels",
                "evaluation contains no materialized depth values",
            ));
        }
        let pixel_count = compared_pixels as f64;
        let mut report = DepthPairEvaluationReport {
            schema_version: DATASET_OFFLINE_EVALUATION_SCHEMA_VERSION,
            dataset_manifest_sha256: self.manifest().content_sha256.clone(),
            predicted_stream: spec.predicted_stream,
            ground_truth_stream: spec.ground_truth_stream,
            compared_frames,
            compared_pixels,
            dropped_pairs,
            mean_absolute_error_m: absolute_sum / pixel_count,
            root_mean_square_error_m: (squared_sum / pixel_count).sqrt(),
            max_absolute_error_m: maximum,
            tolerance_m: spec.tolerance_m,
            passed: maximum <= spec.tolerance_m,
            content_sha256: String::new(),
        };
        report.content_sha256 = report_digest(&report)?;
        report.validate()?;
        Ok(report)
    }

    /// Recomputes a committed depth report from the referenced bundle.
    ///
    /// This is stronger than report self-validation: changing metrics and
    /// recomputing the report's content hash still fails this comparison.
    pub fn verify_depth_pair_report(
        &self,
        report: &DepthPairEvaluationReport,
    ) -> Result<(), DatasetError> {
        report.validate()?;
        if report.dataset_manifest_sha256 != self.manifest().content_sha256 {
            return Err(DatasetError::DigestMismatch(
                "offline evaluation dataset manifest".to_string(),
            ));
        }
        let recomputed = self.evaluate_depth_pair(DepthPairMetricSpec {
            predicted_stream: report.predicted_stream,
            ground_truth_stream: report.ground_truth_stream,
            tolerance_m: report.tolerance_m,
        })?;
        if recomputed != *report {
            return Err(invalid(
                "offline_evaluation",
                "reported metrics do not match dataset recomputation",
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct GapEvent {
    sequence: u64,
    count: u64,
}

#[derive(Clone, Debug)]
struct DepthSample {
    sequence: u64,
    capture_ticks: u64,
    image: ImageDepth,
}

#[derive(Clone, Debug)]
enum DepthEvent {
    Gap(GapEvent),
    Sample(DepthSample),
}

struct DepthEvents {
    records: DatasetRecordReader,
    stream_id: StreamId,
}

impl DepthEvents {
    fn new(records: DatasetRecordReader, stream_id: StreamId) -> Self {
        Self { records, stream_id }
    }

    fn next_event(&mut self) -> Result<Option<DepthEvent>, DatasetError> {
        for record in self.records.by_ref() {
            let record = record?;
            if record.stream_id != self.stream_id {
                continue;
            }
            if record.kind == DatasetRecordKind::Gap {
                return Ok(Some(DepthEvent::Gap(GapEvent {
                    sequence: record.sequence,
                    count: record
                        .dropped_count()
                        .expect("reader validates gap payload"),
                })));
            }
            let (_, image) = decode_image_depth(&record.payload)?;
            return Ok(Some(DepthEvent::Sample(DepthSample {
                sequence: record.sequence,
                capture_ticks: record.capture_ticks,
                image,
            })));
        }
        Ok(None)
    }
}

fn report_digest(report: &DepthPairEvaluationReport) -> Result<String, DatasetError> {
    let mut canonical = report.clone();
    canonical.content_sha256.clear();
    Ok(format!(
        "sha256:{:x}",
        Sha256::digest(serde_json::to_vec(&canonical)?)
    ))
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

fn invalid(field: &'static str, reason: impl Into<String>) -> DatasetError {
    DatasetError::InvalidField {
        field,
        reason: reason.into(),
    }
}
