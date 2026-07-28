//! Stable traffic-network identifiers.

use crate::TrafficIdError;
use serde::{Deserialize, Deserializer, Serialize};
use std::fmt;
use std::str::FromStr;

/// Stable, source-independent identifier used by traffic assets.
///
/// IDs are non-empty ASCII strings. Letters, digits, `-`, `_`, `.`, `~`, `:`,
/// `/`, and `#` are accepted so importers can namespace source identifiers
/// without relying on array position.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
#[serde(transparent)]
pub struct TrafficId(String);

impl TrafficId {
    /// Parses and validates a stable traffic identifier.
    pub fn new(value: impl Into<String>) -> Result<Self, TrafficIdError> {
        let value = value.into();
        if value.is_empty() {
            return Err(TrafficIdError::Empty);
        }
        if let Some(character) = value
            .chars()
            .find(|character| !is_allowed_character(*character))
        {
            return Err(TrafficIdError::InvalidCharacter { character });
        }
        Ok(Self(value))
    }

    /// Returns the identifier text.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for TrafficId {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.0)
    }
}

impl FromStr for TrafficId {
    type Err = TrafficIdError;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        Self::new(value)
    }
}

impl<'de> Deserialize<'de> for TrafficId {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let value = String::deserialize(deserializer)?;
        Self::new(value).map_err(serde::de::Error::custom)
    }
}

fn is_allowed_character(character: char) -> bool {
    character.is_ascii_alphanumeric()
        || matches!(character, '-' | '_' | '.' | '~' | ':' | '/' | '#')
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn namespaced_source_id_is_stable() {
        let id = TrafficId::new("plateau:53394525/road_1#lane-0").expect("valid ID");
        assert_eq!(id.as_str(), "plateau:53394525/road_1#lane-0");
    }

    #[test]
    fn whitespace_and_unicode_are_rejected() {
        assert!(matches!(
            TrafficId::new("lane 1"),
            Err(TrafficIdError::InvalidCharacter { character: ' ' })
        ));
        assert!(matches!(
            TrafficId::new("車線"),
            Err(TrafficIdError::InvalidCharacter { .. })
        ));
    }
}
