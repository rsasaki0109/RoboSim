//! MJCF import failures.

use thiserror::Error;

/// MJCF parsing, conversion, or validation failure.
#[derive(Debug, Error)]
pub enum MjcfError {
    /// The MJCF file could not be read.
    #[error(transparent)]
    Io(#[from] std::io::Error),
    /// The MJCF document could not be parsed as XML.
    #[error("MJCF XML syntax: {0}")]
    Xml(String),
    /// The MJCF document uses a schema feature this importer cannot handle.
    #[error("unsupported MJCF element `{element}`: {reason}")]
    Unsupported {
        /// XML element name.
        element: String,
        /// Why the element cannot be imported.
        reason: String,
    },
    /// The MJCF document is malformed or missing required elements.
    #[error("invalid MJCF: {0}")]
    Invalid(String),
}
