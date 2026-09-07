//! Bounded NCLT wheel/IMU ingestion, preserving recorded values and source times.
//!
//! This is offline evidence ingestion, not a calibrated sensor or physics model.
//! Time follows NCLT's Unix-microsecond convention; receipt time and uncertainty
//! are unknown. IMU vectors remain in the source sensor frame, internally filtered.

use anyhow::{ensure, Context, Result};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::io::Read;

pub mod replay;

/// Maximum uncompressed input size accepted by each reader.
pub const MAX_NCLT_CSV_BYTES: usize = 64 * 1024 * 1024;
const MAX_ROWS: usize = 1_000_000;
const MAX_LINE_BYTES: usize = 1024;

/// One recorded left/right wheel-speed observation, not raw encoder counts.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct NcltWheelSample {
    /// Source timestamp using the dataset's Unix-microsecond convention.
    pub timestamp_us: u64,
    /// Recorded left wheel speed in meters per second.
    pub left_speed_m_s: f64,
    /// Recorded right wheel speed in meters per second.
    pub right_speed_m_s: f64,
}

/// One internally filtered Microstrain sample in its original sensor frame.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct NcltImuSample {
    /// Source timestamp; this does not establish capture versus receipt timing.
    pub timestamp_us: u64,
    /// Source magnetic field, x/y/z, in Gauss (not SI tesla).
    pub magnetic_field_gauss: [f64; 3],
    /// Source acceleration, x/y/z, in meters per second squared.
    pub acceleration_m_s2: [f64; 3],
    /// Source angular velocity about x/y/z, in radians per second.
    pub angular_velocity_rad_s: [f64; 3],
}

/// Parsed rows bound to the exact input bytes; not an authenticity attestation.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct NcltSeries<T> {
    /// SHA-256 of unmodified input bytes, including line endings.
    pub source_sha256: String,
    /// Number of uncompressed bytes consumed.
    pub source_bytes: usize,
    /// Samples in strictly increasing source timestamp order, without resampling.
    pub samples: Vec<T>,
}

/// Read a nonempty three-column wheel CSV; reject malformed, unordered or oversized input.
pub fn read_wheel_samples(reader: impl Read) -> Result<NcltSeries<NcltWheelSample>> {
    parse(reader, |timestamp_us, v: [f64; 2]| NcltWheelSample {
        timestamp_us,
        left_speed_m_s: v[0],
        right_speed_m_s: v[1],
    })
}

/// Read the ten-column `ms25.csv`; do not pass Euler-angle or KVH-heading files.
pub fn read_imu_samples(reader: impl Read) -> Result<NcltSeries<NcltImuSample>> {
    parse(reader, |timestamp_us, v: [f64; 9]| NcltImuSample {
        timestamp_us,
        magnetic_field_gauss: [v[0], v[1], v[2]],
        acceleration_m_s2: [v[3], v[4], v[5]],
        angular_velocity_rad_s: [v[6], v[7], v[8]],
    })
}

fn parse<const N: usize, T>(
    reader: impl Read,
    convert: impl Fn(u64, [f64; N]) -> T,
) -> Result<NcltSeries<T>> {
    let mut bytes = Vec::new();
    reader
        .take(MAX_NCLT_CSV_BYTES as u64 + 1)
        .read_to_end(&mut bytes)
        .context("read NCLT CSV")?;
    ensure!(
        bytes.len() <= MAX_NCLT_CSV_BYTES,
        "NCLT byte limit exceeded"
    );
    let text = std::str::from_utf8(&bytes).context("NCLT CSV must be UTF-8")?;
    let mut samples = Vec::new();
    let mut previous = None;
    for (index, line) in text.lines().enumerate() {
        let row = index + 1;
        ensure!(row <= MAX_ROWS, "NCLT row limit exceeded");
        ensure!(line.len() <= MAX_LINE_BYTES, "row {row}: line too long");
        let mut columns = line.split(',');
        let timestamp = columns.next().unwrap_or_default();
        // Keep the integer exact: no float parsing, rounding, signed or exponent forms.
        ensure!(
            timestamp.len() == 16 && timestamp.bytes().all(|b| b.is_ascii_digit()),
            "row {row}: expected 16-digit NCLT timestamp"
        );
        let timestamp_us = timestamp.parse::<u64>()?;
        ensure!(
            previous.is_none_or(|p| timestamp_us > p),
            "row {row}: timestamps must strictly increase"
        );
        let mut values = [0.0_f64; N];
        for value in &mut values {
            let column = columns
                .next()
                .with_context(|| format!("row {row}: missing column"))?;
            *value = column
                .parse()
                .with_context(|| format!("row {row}: invalid number"))?;
            ensure!(value.is_finite(), "row {row}: non-finite number");
        }
        ensure!(columns.next().is_none(), "row {row}: extra column");
        samples.push(convert(timestamp_us, values));
        previous = Some(timestamp_us);
    }
    ensure!(!samples.is_empty(), "NCLT CSV is empty");
    Ok(NcltSeries {
        source_sha256: format!("{:x}", Sha256::digest(&bytes)),
        source_bytes: bytes.len(),
        samples,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    // These are synthetic format fixtures, not excerpts of physical measurements.
    #[test]
    fn exact_time_units_and_source_bytes_are_preserved() {
        let bytes = b"1350000000000000,-1.25,2\r\n1350000000000014,0,3.5\r\n";
        let a = read_wheel_samples(&bytes[..]).unwrap();
        assert_eq!(a, read_wheel_samples(&bytes[..]).unwrap());
        assert_eq!(a.source_bytes, bytes.len());
        assert_eq!(a.samples[1].timestamp_us - a.samples[0].timestamp_us, 14);
        assert_eq!(a.samples[0].left_speed_m_s, -1.25);
        assert_eq!(a.samples[1].right_speed_m_s, 3.5);
        let lf = String::from_utf8(bytes.to_vec())
            .unwrap()
            .replace("\r\n", "\n");
        let b = read_wheel_samples(lf.as_bytes()).unwrap();
        assert_eq!(a.samples, b.samples);
        assert_ne!(a.source_sha256, b.source_sha256);
    }

    #[test]
    fn imu_columns_are_not_rotated_or_gravity_corrected() {
        let data = read_imu_samples(&b"1350000000000000,1,2,3,4,5,6,7,8,9\n"[..]).unwrap();
        assert_eq!(data.samples[0].magnetic_field_gauss, [1.0, 2.0, 3.0]);
        assert_eq!(data.samples[0].acceleration_m_s2, [4.0, 5.0, 6.0]);
        assert_eq!(data.samples[0].angular_velocity_rad_s, [7.0, 8.0, 9.0]);
        assert!(read_imu_samples(&b"1350000000000000,1,2,3\n"[..]).is_err());
    }

    #[test]
    fn malformed_and_unordered_inputs_fail_closed() {
        for text in [
            "",
            "\n",
            "time,left,right\n",
            "1350000000000000,1\n",
            "1350000000000000,1,2,3\n",
            "1350000000000000,NaN,1\n",
            "1350000000000000,1,inf\n",
            "1.350000000000000e15,1,2\n",
            "1350000000000000,1,2\n\n",
            "1350000000000000,1,2\n1350000000000000,3,4\n",
            "1350000000000001,1,2\n1350000000000000,3,4\n",
        ] {
            assert!(read_wheel_samples(text.as_bytes()).is_err(), "{text:?}");
        }
        assert!(read_wheel_samples(&[255u8][..]).is_err());
        assert!(
            read_wheel_samples(format!("1350000000000000,{},0", "1".repeat(1024)).as_bytes())
                .is_err()
        );
    }

    #[test]
    fn byte_limit_and_io_errors_are_rejected() {
        assert!(
            read_wheel_samples(std::io::repeat(b'0').take(MAX_NCLT_CSV_BYTES as u64 + 1)).is_err()
        );
        struct FailingReader;
        impl Read for FailingReader {
            fn read(&mut self, _: &mut [u8]) -> std::io::Result<usize> {
                Err(std::io::Error::other("injected failure"))
            }
        }
        assert!(read_wheel_samples(FailingReader).is_err());
    }
}
