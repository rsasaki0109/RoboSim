//! Bounded OpenMCT raw-log ingestion, without calibration or resampling.
//!
//! Loop intervals and GUI/DMM timestamps are source declarations, not certified
//! capture clocks. PWM is not terminal voltage and filtered current is not truth.

use anyhow::{ensure, Context, Result};
use sha2::{Digest, Sha256};
use std::io::Read;

pub mod calibration;
pub mod evaluation;

/// Maximum original log size, including metadata and line endings.
pub const MAX_OPENMCT_BYTES: usize = 8 * 1024 * 1024;
const HEADER: &str =
    "REF,MEAS,DT_ms,CURRENT_RAW,CURRENT_AVG,PWM,DMM_CURRENT_A,DMM_TIME_s,DMM_AGE_ms,DMM_SAMPLE_ID";

/// An asynchronous DMM reading, possibly reused by multiple GUI rows.
#[derive(Clone, Debug, PartialEq)]
pub struct OpenMctDmm {
    /// Original nonnegative integer identifier, not a row counter.
    pub sample_id: u64,
    /// Signed recorded current; lead orientation is not corrected here.
    pub current_a: f64,
    /// Recorded DMM timestamp relative to GUI start, in seconds.
    pub time_s: f64,
    /// Recorded age when the GUI row was written, in milliseconds.
    pub age_ms: f64,
}

/// Unmodified numeric source channels, with absent DMM explicitly represented.
#[derive(Clone, Debug, PartialEq)]
pub struct OpenMctSample {
    /// Mode-dependent reference command; no inferred unit conversion.
    pub reference: f64,
    /// Firmware-reported speed in revolutions per minute.
    pub measured_speed_rpm: f64,
    /// Declared positive loop interval in milliseconds.
    pub loop_interval_ms: f64,
    /// Nonnegative raw ADC reading, not amperes.
    pub current_adc_counts: f64,
    /// Firmware-filtered, previously calibrated current; units not inferred.
    pub filtered_current_source: f64,
    /// Applied PWM command, not measured voltage.
    pub pwm_command: f64,
    /// None only for the exact documented missing-DMM tuple.
    pub dmm: Option<OpenMctDmm>,
}

/// Immutable parsed capture bound to original bytes, not authenticated hardware.
#[derive(Clone, Debug, PartialEq)]
pub struct OpenMctSeries {
    source_sha256: String,
    mode: String,
    date: String,
    samples: Vec<OpenMctSample>,
}

impl OpenMctSeries {
    /// Exact-byte SHA-256, including line endings and metadata.
    pub fn source_sha256(&self) -> &str {
        &self.source_sha256
    }
    /// Original mode text; it does not certify an experimental protocol.
    pub fn mode(&self) -> &str {
        &self.mode
    }
    /// Original date declaration, not used as simulation time.
    pub fn date(&self) -> &str {
        &self.date
    }
    /// Validated rows in original order, including reused and missing readings.
    pub fn samples(&self) -> &[OpenMctSample] {
        &self.samples
    }
}

/// Read a strict ten-column OpenMCT capture without dropping invalid records.
///
/// Requires UTF-8, three metadata/header lines, at most 100,000 data rows,
/// and lines no longer than 1,024 bytes. Reused DMM IDs must retain exact current
/// and time bits; new IDs/times cannot regress. A missing reading is not filled.
pub fn read_openmct(reader: impl Read) -> Result<OpenMctSeries> {
    let mut bytes = Vec::new();
    reader
        .take((MAX_OPENMCT_BYTES + 1) as u64)
        .read_to_end(&mut bytes)?;
    ensure!(
        bytes.len() <= MAX_OPENMCT_BYTES,
        "OpenMCT byte limit exceeded"
    );
    let text = std::str::from_utf8(&bytes).context("OpenMCT UTF-8")?;
    let mut lines = text.lines();
    let metadata = |line: Option<&str>, prefix: &str| -> Result<String> {
        let line = line.context("missing OpenMCT metadata")?;
        ensure!(line.len() <= 1024, "metadata line too long");
        let value = line
            .strip_prefix(prefix)
            .context("invalid metadata prefix")?;
        ensure!(
            !value.is_empty() && !value.chars().any(char::is_control),
            "invalid metadata value"
        );
        Ok(value.to_owned())
    };
    let mode = metadata(lines.next(), "Mode: ")?;
    let date = metadata(lines.next(), "Date: ")?;
    ensure!(lines.next() == Some(HEADER), "invalid OpenMCT header");
    let mut samples = Vec::new();
    let mut previous: Option<OpenMctDmm> = None;
    for (row, line) in lines.enumerate() {
        ensure!(
            row < 100_000 && line.len() <= 1024,
            "OpenMCT row/line limit exceeded"
        );
        let fields: Vec<_> = line.split(',').collect();
        ensure!(
            fields.len() == 10,
            "OpenMCT row {row}: expected ten columns"
        );
        let finite = |index: usize| -> Result<f64> {
            ensure!(
                fields[index].trim() == fields[index],
                "unexpected whitespace"
            );
            let value: f64 = fields[index].parse().context("invalid numeric token")?;
            ensure!(value.is_finite(), "nonfinite required channel at row {row}");
            Ok(value)
        };
        let mut values = [0.0; 6];
        for (index, value) in values.iter_mut().enumerate() {
            *value = finite(index)?;
        }
        ensure!(
            values[2] > 0.0 && values[3] >= 0.0,
            "invalid interval or ADC at row {row}"
        );
        let dmm = if fields[9] == "-1" {
            ensure!(
                fields[6..9] == ["nan", "nan", "nan"],
                "partial missing DMM tuple"
            );
            None
        } else {
            ensure!(
                !fields[9].is_empty() && fields[9].bytes().all(|b| b.is_ascii_digit()),
                "invalid DMM identifier"
            );
            let sample = OpenMctDmm {
                sample_id: fields[9].parse()?,
                current_a: finite(6)?,
                time_s: finite(7)?,
                age_ms: finite(8)?,
            };
            ensure!(
                sample.time_s >= 0.0 && sample.age_ms >= 0.0,
                "negative DMM time/age"
            );
            if let Some(old) = &previous {
                ensure!(
                    sample.sample_id >= old.sample_id && sample.time_s >= old.time_s,
                    "DMM ordering regressed"
                );
                if sample.sample_id == old.sample_id {
                    ensure!(
                        sample.current_a.to_bits() == old.current_a.to_bits()
                            && sample.time_s.to_bits() == old.time_s.to_bits(),
                        "reused DMM identity changed"
                    );
                }
            }
            previous = Some(sample.clone());
            Some(sample)
        };
        samples.push(OpenMctSample {
            reference: values[0],
            measured_speed_rpm: values[1],
            loop_interval_ms: values[2],
            current_adc_counts: values[3],
            filtered_current_source: values[4],
            pwm_command: values[5],
            dmm,
        });
    }
    ensure!(!samples.is_empty(), "empty OpenMCT capture");
    Ok(OpenMctSeries {
        source_sha256: format!("{:x}", Sha256::digest(&bytes)),
        mode,
        date,
        samples,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture(rows: &str) -> String {
        format!("Mode: System Identification\nDate: 2026-05-02 20:54:36\n{HEADER}\n{rows}")
    }
    #[test]
    fn preserves_missing_signed_and_reused_measurements() {
        let text = fixture("1,2,20,3,4,5,nan,nan,nan,-1\n1,2,20,3,4,5,-0.1,0.02,1,7\n1,2,20,3,4,5,-0.1,0.02,21,7\n");
        let series = read_openmct(text.as_bytes()).unwrap();
        assert_eq!(series, read_openmct(text.as_bytes()).unwrap());
        assert!(series.samples()[0].dmm.is_none());
        assert_eq!(series.samples()[1].dmm.as_ref().unwrap().current_a, -0.1);
        assert_eq!(series.samples()[2].dmm.as_ref().unwrap().sample_id, 7);
        assert_ne!(
            series.source_sha256(),
            read_openmct(text.replace('\n', "\r\n").as_bytes())
                .unwrap()
                .source_sha256()
        );
    }
    #[test]
    fn rejects_malformed_rows_and_changed_identity() {
        for row in [
            "1,2,20,3,4,5,nan,nan,0,-1",
            "1,2,0,3,4,5,nan,nan,nan,-1",
            "1,2,20,3,4,5,0,0,-1,7",
            "1,2,20,3,4,5,0,0,1,7.0",
            "1,2,20,3,4,5,0,0,1,7,extra",
            "1,inf,20,3,4,5,nan,nan,nan,-1",
            "",
        ] {
            assert!(read_openmct(fixture(row).as_bytes()).is_err(), "{row}");
        }
        for tail in [
            "1,2,20,3,4,5,0.2,0.02,2,7",
            "1,2,20,3,4,5,0.1,0.01,2,8",
            "1,2,20,3,4,5,0.1,0.03,2,6",
        ] {
            assert!(read_openmct(
                fixture(&format!("1,2,20,3,4,5,0.1,0.02,1,7\n{tail}")).as_bytes()
            )
            .is_err());
        }
    }

    #[test]
    fn bounds_input_and_propagates_read_errors() {
        struct Broken;
        impl Read for Broken {
            fn read(&mut self, _: &mut [u8]) -> std::io::Result<usize> {
                Err(std::io::Error::other("source interrupted"))
            }
        }
        assert!(read_openmct(Broken).is_err());
        assert!(read_openmct(std::io::repeat(b'x')).is_err());
        assert!(read_openmct(&[0xff][..]).is_err());
        assert!(read_openmct(fixture(&"1".repeat(1025)).as_bytes()).is_err());
        let row = "1,2,20,3,4,5,nan,nan,nan,-1\n";
        let maximum = fixture(&row.repeat(100_000));
        assert_eq!(
            read_openmct(maximum.as_bytes()).unwrap().samples().len(),
            100_000
        );
        assert!(read_openmct(fixture(&row.repeat(100_001)).as_bytes()).is_err());
        let valid = fixture(row);
        for malformed in [
            valid.replace("Mode: ", "Mode:"),
            valid.replace(HEADER, "wrong"),
            valid.replace("Date: 2026-05-02 20:54:36", "Date: "),
        ] {
            assert!(read_openmct(malformed.as_bytes()).is_err());
        }
    }
}
