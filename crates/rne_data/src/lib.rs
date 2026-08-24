//! Typed DataBus and frame payloads for Robot Native Engine.

#![deny(missing_docs)]

pub mod bus;
pub mod dataset;
pub mod dataset_payload;
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
    RendererDatasetCaptureReport, DATASET_BUNDLE_SCHEMA_VERSION,
    RENDERER_DATASET_CAPTURE_REPORT_KIND, RENDERER_DATASET_CAPTURE_REPORT_SCHEMA_VERSION,
};
pub use dataset_payload::{
    decode_dataset_action, decode_dataset_annotation, decode_dataset_imu,
    decode_dataset_task_outcome, decode_dataset_transform, encode_dataset_action,
    encode_dataset_annotation, encode_dataset_imu, encode_dataset_task_outcome,
    encode_dataset_transform, DatasetActionSample, DatasetGroundTruthAnnotation,
    DatasetTaskOutcomeSample, DATASET_ACTION_ENCODING, DATASET_ANNOTATION_ENCODING,
    DATASET_IMU_ENCODING, DATASET_PAYLOAD_SCHEMA_VERSION, DATASET_TASK_OUTCOME_ENCODING,
    DATASET_TRANSFORM_ENCODING,
};
pub use frame::{Frame, FrameHeader, FramePayload};
pub use offline::{
    DepthPairEvaluationReport, DepthPairMetricSpec, DATASET_OFFLINE_EVALUATION_SCHEMA_VERSION,
};
pub use payloads::{
    ImageDepth, ImageRgb8, ImuFeedback, ImuFeedbackStatus, ImuSample, JointCommandFeedback,
    JointCommandMode, JointCoordinateFeedback, JointEffortFeedback, JointFeedback,
    JointFeedbackChannel, JointFeedbackStatus, JointState, PointCloud, PoseSample,
    WheelEncoderSample,
};
pub use stream::StreamId;
