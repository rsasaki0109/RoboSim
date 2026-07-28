//! Traffic asset validation and I/O errors.

use crate::TrafficId;
use thiserror::Error;

/// Invalid stable traffic identifier.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum TrafficIdError {
    /// An identifier was empty.
    #[error("traffic ID must not be empty")]
    Empty,
    /// An identifier contained a character outside the canonical ASCII set.
    #[error("traffic ID contains unsupported character `{character}`")]
    InvalidCharacter {
        /// Unsupported character.
        character: char,
    },
}

/// Failure while reading, writing, or validating a traffic asset.
#[derive(Debug, Error)]
pub enum TrafficAssetError {
    /// JSON parsing or serialization failed.
    #[error("invalid traffic asset JSON: {0}")]
    Json(#[from] serde_json::Error),
    /// File I/O failed.
    #[error("traffic asset I/O failed at {path}: {message}")]
    Io {
        /// File involved in the failed operation.
        path: String,
        /// Operating-system error text.
        message: String,
    },
    /// The schema name or version is unsupported.
    #[error("unsupported traffic schema `{schema}` version {version}")]
    UnsupportedSchema {
        /// Schema identifier found in the file.
        schema: String,
        /// Schema version found in the file.
        version: u32,
    },
    /// Two records share one globally stable ID.
    #[error("duplicate traffic ID `{id}` used by {first_kind} and {second_kind}")]
    DuplicateId {
        /// Duplicated identifier.
        id: TrafficId,
        /// First record kind that registered the ID.
        first_kind: &'static str,
        /// Later record kind that reused the ID.
        second_kind: &'static str,
    },
    /// A stable-ID reference does not resolve to the required record kind.
    #[error("{owner_kind} `{owner_id}` references missing {target_kind} `{target_id}`")]
    MissingReference {
        /// Kind of record containing the reference.
        owner_kind: &'static str,
        /// Stable ID of the record containing the reference.
        owner_id: TrafficId,
        /// Expected target record kind.
        target_kind: &'static str,
        /// Referenced stable ID.
        target_id: TrafficId,
    },
    /// A numeric, textual, or collection invariant is invalid.
    #[error("invalid `{field}` on {owner_kind} `{owner_id}`: {message}")]
    InvalidValue {
        /// Kind of record containing the invalid value.
        owner_kind: &'static str,
        /// Stable ID of the record containing the invalid value.
        owner_id: TrafficId,
        /// Field name.
        field: &'static str,
        /// Validation detail.
        message: String,
    },
}

impl TrafficAssetError {
    pub(crate) fn io(path: &std::path::Path, error: std::io::Error) -> Self {
        Self::Io {
            path: path.display().to_string(),
            message: error.to_string(),
        }
    }
}
