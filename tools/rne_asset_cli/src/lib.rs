//! Reusable implementation behind the `rne-asset` command-line workflows.

/// Stable artifact kind for the flagship workflow report.
pub const FLAGSHIP_WORKFLOW_REPORT_KIND: &str = "rne_flagship_workflow_report";
/// Current schema version for the flagship workflow report.
pub const FLAGSHIP_WORKFLOW_REPORT_SCHEMA_VERSION: u32 = 1;
/// Stable artifact kind for the Rapier/MuJoCo flagship comparison report.
pub const FLAGSHIP_CROSS_BACKEND_REPORT_KIND: &str = "rne_flagship_cross_backend_report";
/// Current schema version for the flagship cross-backend comparison report.
pub const FLAGSHIP_CROSS_BACKEND_REPORT_SCHEMA_VERSION: u32 = 2;
/// Stable artifact kind for an installed flagship proof report.
pub const INSTALLED_FLAGSHIP_PROOF_REPORT_KIND: &str = "rne_installed_flagship_proof_report";
/// Current schema version for an installed flagship proof report.
pub const INSTALLED_FLAGSHIP_PROOF_REPORT_SCHEMA_VERSION: u32 = 3;
/// Stable artifact kind for a hardware-named time-to-proof measurement.
pub const TIME_TO_PROOF_REPORT_KIND: &str = "rne_time_to_proof_report";
/// Current schema version for a hardware-named time-to-proof measurement.
pub const TIME_TO_PROOF_REPORT_SCHEMA_VERSION: u32 = 1;

pub mod failure_capsule;
