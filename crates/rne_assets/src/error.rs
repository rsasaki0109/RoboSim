//! Asset loading and spawn errors.

use thiserror::Error;

/// Error while loading or spawning an RNE asset file.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum AssetError {
    /// Failed to read a file from disk.
    #[error("failed to read {path}: {message}")]
    Io {
        /// File path.
        path: String,
        /// OS error message.
        message: String,
    },
    /// TOML syntax or schema validation failed.
    #[error("invalid asset {path}: {message}")]
    Invalid {
        /// File path.
        path: String,
        /// Validation detail.
        message: String,
    },
    /// Robot kind requires an adapter outside the core asset loader.
    #[error("robot kind `{kind}` requires an external spawner")]
    UnsupportedRobotKind {
        /// Robot kind tag from the asset file.
        kind: String,
    },
}

impl AssetError {
    pub(crate) fn invalid(path: impl Into<String>, message: impl Into<String>) -> Self {
        Self::Invalid {
            path: path.into(),
            message: message.into(),
        }
    }
}
