//! Typed DataBus and frame payloads for Robot Native Engine.

#![deny(missing_docs)]

pub mod bus;
pub mod dataset;
pub mod frame;
pub mod offline;
pub mod payloads;
pub mod stream;
pub mod transport;

pub use bus::{DataBus, InMemoryDataBus, SubscriptionCursor};
pub use dataset::{
    DatasetAsset, DatasetBundle, DatasetBundleWriter, DatasetCalibration, DatasetError,
    DatasetFieldSpec, DatasetGapPolicy, DatasetLatencyModel, DatasetLatencySpec, DatasetManifest,
    DatasetNoiseSpec, DatasetRandomizationDecision, DatasetRandomizationValue, DatasetRecord,
    DatasetRecordKind, DatasetRecordReader, DatasetShard, DatasetStreamKind, DatasetStreamSpec,
    DatasetStreamSummary, DatasetTimingSpec, DatasetVerificationReport,
    DATASET_BUNDLE_SCHEMA_VERSION,
};
pub use frame::{Frame, FrameHeader, FramePayload};
pub use offline::{
    DepthPairEvaluationReport, DepthPairMetricSpec, DATASET_OFFLINE_EVALUATION_SCHEMA_VERSION,
};
pub use payloads::{
    ImageDepth, ImageRgb8, ImuSample, JointState, PointCloud, PoseSample, WheelEncoderSample,
};
pub use stream::StreamId;
