//! Offline DDMR ingestion and raw-row-disjoint evaluation partitions.
//!
//! Recorded voltage semantics and capture/receipt timing are unqualified. These
//! values must not be presented as calibrated terminal voltage or sensor timing.
//! No resampling, inferred encoder counts, synthetic rows, or physics fit occurs.

use anyhow::{ensure, Context, Result};
use sha2::{Digest, Sha256};
use std::{io::Read, ops::Range};

pub mod response;

/// Maximum source size, including header and line endings.
pub const MAX_DDMR_CSV_BYTES: usize = 32 * 1024 * 1024;
const MAX_ROWS: usize = 500_000;
const MAX_LINE_BYTES: usize = 512;
const HEADER: &str = "Time (s),Left_Volt (V),Right_Volt (V),Left_Speed (rad/s),Right_Speed (rad/s)";

/// Unmodified numeric values from one source record; not a calibrated sensor frame.
#[derive(Clone, Debug, PartialEq)]
pub struct DdmrSample {
    /// Original time token, retaining decimal spelling for source inspection.
    pub source_time_text: String,
    /// Source seconds as f64; not converted to simulation ticks or wall time.
    pub source_time_s: f64,
    /// Left/right recorded voltage fields; command versus measurement is unknown.
    pub recorded_voltage_v: [f64; 2],
    /// Left/right recorded wheel angular speed, with unknown encoder processing.
    pub recorded_speed_rad_s: [f64; 2],
}

/// Validated ordered source, bound to the exact bytes read, not authenticated.
#[derive(Clone, Debug, PartialEq)]
pub struct DdmrSeries {
    source_sha256: String,
    source_bytes: usize,
    samples: Vec<DdmrSample>,
}

impl DdmrSeries {
    /// SHA-256 of original bytes, including any BOM and line endings.
    pub fn source_sha256(&self) -> &str {
        &self.source_sha256
    }

    /// Exact source byte count.
    pub fn source_bytes(&self) -> usize {
        self.source_bytes
    }

    /// Ordered samples; callers cannot mutate the validated series in place.
    pub fn samples(&self) -> &[DdmrSample] {
        &self.samples
    }

    /// Split raw records before constructing any prediction windows.
    ///
    /// Cut indices are zero-based exclusive ends. All three partitions must be
    /// nonempty. This prevents shared raw history, but does not make segments of
    /// one physical capture independent experiments. Cuts must be frozen before
    /// inspecting held-out performance.
    pub fn split(&self, train_end: usize, validation_end: usize) -> Result<DdmrSplit<'_>> {
        ensure!(
            0 < train_end && train_end < validation_end && validation_end < self.samples.len(),
            "DDMR requires three nonempty ordered raw-row partitions"
        );
        let partition = |rows: Range<usize>| DdmrPartition {
            samples: &self.samples[rows.clone()],
            rows,
        };
        Ok(DdmrSplit {
            training: partition(0..train_end),
            validation: partition(train_end..validation_end),
            test: partition(validation_end..self.samples.len()),
        })
    }
}

/// Three chronological partitions with no raw row shared between them.
#[derive(Debug)]
pub struct DdmrSplit<'a> {
    /// Fit parameters here only.
    pub training: DdmrPartition<'a>,
    /// Select model/hyperparameters here without inspecting test performance.
    pub validation: DdmrPartition<'a>,
    /// Frozen held-out evaluation, not an independent recording session.
    pub test: DdmrPartition<'a>,
}

/// Read-only raw records belonging to one split.
#[derive(Debug)]
pub struct DdmrPartition<'a> {
    samples: &'a [DdmrSample],
    rows: Range<usize>,
}

impl<'a> DdmrPartition<'a> {
    /// Zero-based raw data row range, excluding the header.
    pub fn raw_rows(&self) -> Range<usize> {
        self.rows.clone()
    }

    /// Construct prediction windows entirely inside this partition, without copies.
    ///
    /// History is ordered oldest to newest. A horizon of one predicts the next
    /// row after history. The horizon counts rows, not time; use source times for
    /// elapsed seconds. No future row is supplied in history, and no padding or
    /// preceding-partition warm-up is allowed. Reject a partition too short for
    /// even one window, rather than report an empty successful evaluation.
    pub fn windows(
        &self,
        history_rows: usize,
        horizon_rows: usize,
    ) -> Result<impl ExactSizeIterator<Item = DdmrWindow<'a>> + '_> {
        ensure!(
            history_rows > 0 && horizon_rows > 0,
            "DDMR window sizes must be positive"
        );
        let span = history_rows
            .checked_add(horizon_rows)
            .context("DDMR window size overflow")?;
        ensure!(
            span <= self.samples.len(),
            "DDMR partition is too short for requested window"
        );
        Ok(
            (0..self.samples.len() - span + 1).map(move |start| DdmrWindow {
                history: &self.samples[start..start + history_rows],
                target: &self.samples[start + span - 1],
                history_raw_rows: self.rows.start + start..self.rows.start + start + history_rows,
                target_raw_row: self.rows.start + start + span - 1,
            }),
        )
    }
}

/// One past-only prediction input and its held-out future record.
#[derive(Debug)]
pub struct DdmrWindow<'a> {
    /// Past records. A model must declare which recorded fields it consumes.
    pub history: &'a [DdmrSample],
    /// Future label record, never part of history.
    pub target: &'a DdmrSample,
    /// Original data-row coordinates for auditing window membership.
    pub history_raw_rows: Range<usize>,
    /// Original target data-row coordinate.
    pub target_raw_row: usize,
}

/// Read the exact five-column DDMR numeric CSV dialect with bounded memory.
///
/// Accept LF/CRLF, an optional initial UTF-8 BOM and optional final newline.
/// Reject quoted fields, blank records, changed headers, nonfinite values,
/// negative time, non-increasing f64 times and source/row/line limit violations.
/// An I/O error returns no partial series. Uniform sampling is not assumed.
pub fn read_ddmr_samples(reader: impl Read) -> Result<DdmrSeries> {
    let mut bytes = Vec::new();
    reader
        .take(MAX_DDMR_CSV_BYTES as u64 + 1)
        .read_to_end(&mut bytes)
        .context("read DDMR source")?;
    ensure!(
        bytes.len() <= MAX_DDMR_CSV_BYTES,
        "DDMR byte limit exceeded"
    );
    let text = std::str::from_utf8(&bytes).context("DDMR source must be UTF-8")?;
    let mut lines = text.strip_prefix('\u{feff}').unwrap_or(text).lines();
    ensure!(lines.next() == Some(HEADER), "DDMR header mismatch");
    let mut samples = Vec::new();
    let mut previous = None;
    for (index, line) in lines.enumerate() {
        let row = index + 1;
        ensure!(row <= MAX_ROWS, "DDMR row limit exceeded");
        ensure!(
            line.len() <= MAX_LINE_BYTES,
            "DDMR row {row}: line limit exceeded"
        );
        let mut columns = line.split(',');
        let mut values = [0.0_f64; 5];
        let mut time_text = "";
        for (column_index, value) in values.iter_mut().enumerate() {
            let token = columns
                .next()
                .with_context(|| format!("DDMR row {row}: missing column"))?;
            if column_index == 0 {
                time_text = token;
            }
            *value = token
                .parse()
                .with_context(|| format!("DDMR row {row}: invalid number"))?;
            ensure!(value.is_finite(), "DDMR row {row}: nonfinite number");
        }
        ensure!(columns.next().is_none(), "DDMR row {row}: extra column");
        ensure!(values[0] >= 0.0, "DDMR row {row}: negative time");
        ensure!(
            previous.is_none_or(|p| values[0] > p),
            "DDMR row {row}: time must increase"
        );
        previous = Some(values[0]);
        samples.push(DdmrSample {
            source_time_text: time_text.to_owned(),
            source_time_s: values[0],
            recorded_voltage_v: [values[1], values[2]],
            recorded_speed_rad_s: [values[3], values[4]],
        });
    }
    ensure!(!samples.is_empty(), "DDMR source has no samples");
    Ok(DdmrSeries {
        source_sha256: format!("{:x}", Sha256::digest(&bytes)),
        source_bytes: bytes.len(),
        samples,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture(rows: usize) -> String {
        let mut text = format!("{HEADER}\n");
        for i in 0..rows {
            text.push_str(&format!("{i},-1,2,3,-4\n"));
        }
        text
    }

    #[test]
    fn source_values_and_original_spelling_are_preserved() {
        let text = format!("\u{feff}{HEADER}\r\n0.000,-0,2,3,-4\r\n0.017,5,6,7,8\r\n");
        let data = read_ddmr_samples(text.as_bytes()).unwrap();
        assert_eq!(data.source_bytes(), text.len());
        assert_eq!(
            data.source_sha256(),
            format!("{:x}", Sha256::digest(text.as_bytes()))
        );
        assert_eq!(data.samples()[0].source_time_text, "0.000");
        assert_eq!(
            data.samples()[0].recorded_voltage_v[0].to_bits(),
            (-0.0_f64).to_bits()
        );
        assert_eq!(data.samples()[1].source_time_s, 0.017);
        assert_eq!(data.samples()[1].recorded_speed_rad_s, [7.0, 8.0]);
        let lf = read_ddmr_samples(text.replace("\r\n", "\n").as_bytes()).unwrap();
        assert_eq!(data.samples(), lf.samples());
        assert_ne!(data.source_sha256(), lf.source_sha256());
    }

    #[test]
    fn splits_have_disjoint_history_and_targets() {
        let source = read_ddmr_samples(fixture(30).as_bytes()).unwrap();
        let split = source.split(10, 20).unwrap();
        for (offset, part) in [
            (0, &split.training),
            (10, &split.validation),
            (20, &split.test),
        ] {
            assert_eq!(part.raw_rows(), offset..offset + 10);
            let windows: Vec<_> = part.windows(3, 2).unwrap().collect();
            assert_eq!(windows.len(), 6);
            for (i, window) in windows.iter().enumerate() {
                assert_eq!(window.history_raw_rows, offset + i..offset + i + 3);
                assert_eq!(window.target_raw_row, offset + i + 4);
                assert!(part.raw_rows().contains(&window.target_raw_row));
                assert!(window
                    .history_raw_rows
                    .clone()
                    .all(|row| part.raw_rows().contains(&row)));
                assert_eq!(window.target.source_time_s, (offset + i + 4) as f64);
                assert!(window.history.last().unwrap().source_time_s < window.target.source_time_s);
            }
            assert_eq!(part.windows(9, 1).unwrap().len(), 1);
            for (history, horizon) in [(0, 1), (1, 0), (10, 1), (usize::MAX, 1)] {
                assert!(part.windows(history, horizon).is_err());
            }
        }
        for (a, b) in [(0, 20), (10, 10), (20, 10), (10, 30), (10, usize::MAX)] {
            assert!(source.split(a, b).is_err());
        }
    }

    #[test]
    fn invalid_sources_fail_without_partial_output() {
        for body in [
            "",
            "\n",
            "0,1,2,3\n",
            "0,1,2,3,4,5\n",
            "0,NaN,2,3,4\n",
            "0,1,2,inf,4\n",
            "-1,1,2,3,4\n",
            "0,1,2,3,4\n0,1,2,3,4\n",
            "1,1,2,3,4\n0,1,2,3,4\n",
            "0,1,2,3,4\n\n",
            "\"0\",1,2,3,4\n",
        ] {
            assert!(
                read_ddmr_samples(format!("{HEADER}\n{body}").as_bytes()).is_err(),
                "{body:?}"
            );
        }
        assert!(read_ddmr_samples(&b"time,left,right\n0,1,2\n"[..]).is_err());
        assert!(read_ddmr_samples(&[255][..]).is_err());
        assert!(
            read_ddmr_samples(format!("{HEADER}\n{},1,2,3,4", "0".repeat(513)).as_bytes()).is_err()
        );
        assert!(read_ddmr_samples(fixture(MAX_ROWS + 1).as_bytes()).is_err());
        assert!(
            read_ddmr_samples(std::io::repeat(b'0').take(MAX_DDMR_CSV_BYTES as u64 + 1)).is_err()
        );
        struct Broken;
        impl Read for Broken {
            fn read(&mut self, _: &mut [u8]) -> std::io::Result<usize> {
                Err(std::io::Error::other("injected read failure"))
            }
        }
        assert!(read_ddmr_samples(Broken).is_err());
    }
}
