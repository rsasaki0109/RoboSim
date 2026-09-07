//! Bounded streaming audit of headerless IPIN static three-axis CSV exports.
//!
//! Source coordinates are preserved, not certified as physical capture times.
//! No calibration, nominal fault status, resampling or sensor-frame rotation is inferred.

use anyhow::{ensure, Context, Result};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::io::{BufRead, Read};

/// Maximum uncompressed bytes in one accepted source.
pub const MAX_IPIN_BYTES: u64 = 1024 * 1024 * 1024;
/// Maximum rows in one accepted source.
pub const MAX_IPIN_ROWS: u64 = 20_000_000;
const MAX_LINE_BYTES: u64 = 512;

/// Complete-source audit, produced only after successful EOF validation.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct IpinImuAudit {
    /// SHA-256 of exact source bytes, including line endings.
    pub source_sha256: String,
    /// Uncompressed bytes consumed.
    pub source_bytes: u64,
    /// Number of accepted records.
    pub rows: u64,
    /// First source time-of-week coordinate in microseconds, not receipt time.
    pub first_source_time_us: u64,
    /// Last source time-of-week coordinate in microseconds.
    pub last_source_time_us: u64,
    /// Minimum adjacent source-time difference; absent for a single row.
    pub minimum_interval_us: Option<u64>,
    /// Maximum adjacent source-time difference; absent for a single row.
    pub maximum_interval_us: Option<u64>,
    /// Per-axis minimum in source units; caller must qualify sensor identity/units.
    pub minimum_source_values: [f64; 3],
    /// Per-axis maximum in source units.
    pub maximum_source_values: [f64; 3],
}

fn parse_time_us(text: &str) -> Result<u64> {
    let (seconds, fraction) = text
        .split_once('.')
        .context("expected decimal source time")?;
    ensure!(
        !seconds.is_empty()
            && seconds.len() <= 6
            && fraction.len() == 6
            && seconds
                .bytes()
                .chain(fraction.bytes())
                .all(|b| b.is_ascii_digit()),
        "expected unsigned time with exactly six fractional digits"
    );
    let seconds: u64 = seconds.parse()?;
    ensure!(seconds < 604800, "source time outside GPS week");
    Ok(seconds * 1_000_000 + fraction.parse::<u64>()?)
}

/// Audit an entire static acceleration or rotation export with bounded memory.
///
/// Accept exactly four columns, finite values, strictly increasing six-decimal
/// source times within a GPS week, LF/CRLF or an unterminated final line. Reject
/// headers, gaps represented as blank lines, time wrap, or any malformed row.
/// Unequal positive time intervals are retained in the report, not repaired.
/// This format audit cannot distinguish acceleration from rotation or certify units.
/// Memory is bounded by a single line plus the caller's input buffer. An error
/// returns no partial report and makes no external publications.
pub fn audit_ipin_imu(reader: impl BufRead) -> Result<IpinImuAudit> {
    audit_bounded(reader, MAX_IPIN_BYTES, MAX_IPIN_ROWS)
}

fn audit_bounded(mut reader: impl BufRead, max_bytes: u64, max_rows: u64) -> Result<IpinImuAudit> {
    let (_, audit) = consume_ipin(&mut reader, max_bytes, max_rows, |stream| {
        for sample in stream {
            sample?;
        }
        Ok(())
    })?;
    Ok(audit)
}

/// Source-bound single-axis irregular statistic, with no physical calibration claim.
#[derive(Debug)]
pub struct IpinAxisDeviation {
    /// Complete input-byte and source-coordinate audit.
    pub source: IpinImuAudit,
    /// Source axis index, 0=X, 1=Y, 2=Z; no frame rotation.
    pub axis: usize,
    /// Explicit grid in nanosecond ticks derived from source time-of-week seconds.
    pub plan: rne_sensor::allan::timed::TimeWindowPlan,
    /// Sample-count-weighted statistic, not a fitted sensor profile.
    pub statistic: rne_sensor::allan::timed::TimedDeviation,
}

/// Analyze one axis while hashing and validating all three columns to EOF.
/// Source microseconds are exactly multiplied by 1,000 into RNE ticks. This does
/// not certify capture timing or assign a GPS week. No unknown flags are invented.
pub fn analyze_ipin_axis(
    reader: impl BufRead,
    axis: usize,
    plan: rne_sensor::allan::timed::TimeWindowPlan,
) -> Result<IpinAxisDeviation> {
    ensure!(axis < 3, "IPIN axis must be 0, 1 or 2");
    let (statistic, source) = consume_ipin(reader, MAX_IPIN_BYTES, MAX_IPIN_ROWS, |stream| {
        let mut source_error = None;
        let mapped = stream.map(|item| match item {
            Ok((time_us, values)) => Ok(rne_sensor::allan::AllanSample {
                capture_ticks: time_us * 1000,
                value: values[axis],
            }),
            Err(error) => {
                source_error = Some(error);
                Err(rne_sensor::allan::AllanError::Arithmetic)
            }
        });
        let result = rne_sensor::allan::timed::weighted_time_deviation(mapped, plan);
        if let Some(error) = source_error {
            return Err(error);
        }
        Ok(result?)
    })?;
    Ok(IpinAxisDeviation {
        source,
        axis,
        plan,
        statistic,
    })
}

type SourceSample = (u64, [f64; 3]);

fn consume_ipin<T>(
    mut reader: impl BufRead,
    max_bytes: u64,
    max_rows: u64,
    consume: impl FnOnce(&mut dyn Iterator<Item = Result<SourceSample>>) -> Result<T>,
) -> Result<(T, IpinImuAudit)> {
    let mut hash = Sha256::new();
    let mut line = Vec::new();
    let mut report = IpinImuAudit {
        source_sha256: String::new(),
        source_bytes: 0,
        rows: 0,
        first_source_time_us: 0,
        last_source_time_us: 0,
        minimum_interval_us: None,
        maximum_interval_us: None,
        minimum_source_values: [f64::INFINITY; 3],
        maximum_source_values: [f64::NEG_INFINITY; 3],
    };
    let mut finished = false;
    let output = {
        let mut stream = std::iter::from_fn(|| {
            if finished {
                return None;
            }
            let next = (|| -> Result<Option<SourceSample>> {
                line.clear();
                let size = (&mut reader)
                    .take(MAX_LINE_BYTES + 1)
                    .read_until(b'\n', &mut line)?;
                if size == 0 {
                    return Ok(None);
                }
                ensure!(size as u64 <= MAX_LINE_BYTES, "IPIN line limit exceeded");
                report.source_bytes += size as u64;
                ensure!(report.source_bytes <= max_bytes, "IPIN byte limit exceeded");
                ensure!(report.rows < max_rows, "IPIN row limit exceeded");
                hash.update(&line);
                let text = std::str::from_utf8(&line).context("IPIN must be UTF-8")?;
                let text = text.strip_suffix('\n').unwrap_or(text);
                let text = text.strip_suffix('\r').unwrap_or(text);
                let mut fields = text.split(',');
                let time = parse_time_us(fields.next().unwrap_or_default())
                    .with_context(|| format!("IPIN row {} time", report.rows + 1))?;
                if report.rows == 0 {
                    report.first_source_time_us = time;
                } else {
                    ensure!(
                        time > report.last_source_time_us,
                        "IPIN source times must increase"
                    );
                    let dt = time - report.last_source_time_us;
                    report.minimum_interval_us =
                        Some(report.minimum_interval_us.map_or(dt, |v| v.min(dt)));
                    report.maximum_interval_us =
                        Some(report.maximum_interval_us.map_or(dt, |v| v.max(dt)));
                }
                let mut values = [0.0; 3];
                for (axis, stored) in values.iter_mut().enumerate() {
                    let field = fields.next().context("missing IPIN axis")?;
                    let value: f64 = field.parse().context("invalid IPIN axis")?;
                    ensure!(value.is_finite(), "nonfinite IPIN axis");
                    *stored = value;
                    report.minimum_source_values[axis] =
                        report.minimum_source_values[axis].min(value);
                    report.maximum_source_values[axis] =
                        report.maximum_source_values[axis].max(value);
                }
                ensure!(fields.next().is_none(), "extra IPIN column");
                report.last_source_time_us = time;
                report.rows += 1;
                Ok(Some((time, values)))
            })();
            if !matches!(&next, Ok(Some(_))) {
                finished = true;
            }
            next.transpose()
        });
        consume(&mut stream)?
    };
    ensure!(finished, "IPIN consumer did not validate to EOF");
    ensure!(report.rows > 0, "empty IPIN source");
    report.source_sha256 = format!("{:x}", hash.finalize());
    Ok((output, report))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{self, BufReader, Cursor};

    #[test]
    fn analysis_binds_exact_source_and_rejects_unused_axis_corruption() {
        use rne_sensor::allan::timed::TimeWindowPlan;
        let source = [0, 1, 3, 4, 8, 9, 10]
            .map(|t| format!("{t}.000000,{},1,2\n", 2 * t))
            .concat();
        let plan = TimeWindowPlan {
            window_ticks: 2_000_000_000,
            first_endpoint_ticks: 4_000_000_000,
            endpoint_period_ticks: 1_000_000_000,
            endpoints: 7,
        };
        let got = analyze_ipin_axis(source.as_bytes(), 0, plan).unwrap();
        assert_eq!(got.source, audit_ipin_imu(source.as_bytes()).unwrap());
        assert_eq!(got.statistic.variance, 10.4);
        assert_eq!(
            (got.statistic.valid_pairs, got.statistic.empty_pairs),
            (3, 4)
        );
        assert_eq!(
            analyze_ipin_axis(source.as_bytes(), 1, plan)
                .unwrap()
                .statistic
                .variance,
            0.0
        );
        assert!(analyze_ipin_axis(source.as_bytes(), 3, plan).is_err());
        let corrupt = format!("{source}11.000000,0,NaN,0\n");
        assert!(analyze_ipin_axis(corrupt.as_bytes(), 0, plan)
            .unwrap_err()
            .to_string()
            .contains("nonfinite IPIN"));
    }

    #[test]
    fn exact_bytes_and_nonuniform_time_survive_small_buffers() {
        let data = b"259553.004897,-1,2,3\r\n259553.009864,4,5,-6\r\n259553.014830,1,1,1";
        let a = audit_ipin_imu(BufReader::with_capacity(1, Cursor::new(data))).unwrap();
        assert_eq!(a.rows, 3);
        assert_eq!(a.minimum_interval_us, Some(4966));
        assert_eq!(a.maximum_interval_us, Some(4967));
        assert_eq!(a.first_source_time_us, 259553004897);
        assert_eq!(a.minimum_source_values, [-1., 1., -6.]);
        assert_eq!(a.maximum_source_values, [4., 5., 3.]);
        assert_eq!(a.source_sha256, format!("{:x}", Sha256::digest(data)));
        let lf = String::from_utf8(data.to_vec())
            .unwrap()
            .replace("\r\n", "\n");
        let b = audit_ipin_imu(lf.as_bytes()).unwrap();
        assert_ne!(a.source_sha256, b.source_sha256);
        assert_eq!(a.minimum_interval_us, b.minimum_interval_us);
    }

    #[test]
    fn invalid_data_and_limits_fail_without_report() {
        for data in [
            "",
            "\n",
            "time,x,y,z",
            "1.000000,1,2",
            "1.000000,1,2,3,4",
            "1.000000,NaN,2,3",
            "1.000000,1,inf,3",
            "604800.000000,1,2,3",
            "1.00000,1,2,3",
            "+1.000000,1,2,3",
            "1e0.000000,1,2,3",
            "1.000000,1,2,3\n1.000000,1,2,3",
            "2.000000,1,2,3\n1.000000,1,2,3",
        ] {
            assert!(audit_ipin_imu(data.as_bytes()).is_err(), "{data}");
        }
        let data = b"1.000000,1,2,3\n";
        assert!(audit_bounded(&data[..], data.len() as u64 - 1, 1).is_err());
        assert!(audit_bounded(&data[..], 100, 0).is_err());
        assert!(audit_ipin_imu(&vec![b'1'; 513][..]).is_err());
        struct Broken;
        impl Read for Broken {
            fn read(&mut self, _: &mut [u8]) -> io::Result<usize> {
                Err(io::Error::other("fixture"))
            }
        }
        assert!(audit_ipin_imu(BufReader::new(Broken)).is_err());
    }
}
