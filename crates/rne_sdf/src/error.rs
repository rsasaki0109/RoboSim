//! SDF import failures.

use thiserror::Error;

/// SDF parsing, conversion, or validation failure.
#[derive(Debug, Error)]
pub enum SdfError {
    /// The SDF file could not be read or written.
    #[error(transparent)]
    Io(#[from] std::io::Error),
    /// The SDF document could not be parsed as XML.
    #[error("SDF XML syntax: {0}")]
    Xml(String),
    /// The SDF document uses a schema feature this importer cannot handle.
    #[error("unsupported SDF element `{element}`: {reason}")]
    Unsupported {
        /// XML element name.
        element: String,
        /// Why the element cannot be imported.
        reason: String,
    },
    /// The SDF document is malformed or missing required elements.
    #[error("invalid SDF: {0}")]
    Invalid(String),
}
