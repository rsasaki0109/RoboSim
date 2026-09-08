//! Explicit offline velocity reconstruction for acquired suspension positions.
//! This is not a causal controller sensor or an automatic acquisition converter.

use anyhow::{ensure, Result};
use serde::{Deserialize, Serialize};

use crate::suspension_acquisition::{
    SuspensionEvidenceFileRef, SuspensionPhysicalAcquisitionManifest, SuspensionSignalOrigin,
};
use crate::suspension_identification::SuspensionIdentificationDataset;

/// Caller-declared executable interpretation of one retained velocity procedure.
/// The file reference binds bytes, not the truth of the caller's interpretation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionDerivativeBinding {
    /// Exact capture identity from the acquisition manifest.
    pub capture_id: String,
    /// Exact derived velocity calibration/procedure reference from the manifest.
    pub procedure: SuspensionEvidenceFileRef,
    /// Explicit numerical procedure; never inferred from a source label.
    pub operator: SuspensionDerivativeOperator,
    /// Caller-declared inclusive maximum absolute reconstruction error (m/s).
    pub absolute_tolerance_m_s: f64,
}

impl SuspensionDerivativeBinding {
    /// Validates declarations and reconstructs every velocity without changing inputs.
    /// Does not read retained files; use `verify_files` for file-bound verification.
    /// A mismatch, including at an endpoint, rejects the entire acquisition.
    pub fn validate(
        &self,
        dataset: &SuspensionIdentificationDataset,
        manifest: &SuspensionPhysicalAcquisitionManifest,
    ) -> Result<Vec<f64>> {
        manifest.validate(dataset)?;
        // Manifest validation fixes the position/velocity/force order and size.
        let velocity = &manifest.signals[1];
        ensure!(
            self.capture_id == manifest.capture_id,
            "derivative capture mismatch"
        );
        ensure!(
            velocity.origin == SuspensionSignalOrigin::Derived
                && self.procedure == velocity.calibration_artifact,
            "derivative procedure binding mismatch"
        );
        ensure!(
            self.absolute_tolerance_m_s.is_finite() && self.absolute_tolerance_m_s >= 0.0,
            "invalid derivative reconstruction tolerance"
        );
        let times: Vec<_> = dataset.samples.iter().map(|s| s.capture_time_s).collect();
        let positions: Vec<_> = dataset.samples.iter().map(|s| s.position_m).collect();
        let velocities = self.operator.reconstruct(&times, &positions)?;
        for (row, (reconstructed, sample)) in velocities.iter().zip(&dataset.samples).enumerate() {
            let error_m_s = (reconstructed - sample.velocity_m_s).abs();
            ensure!(
                error_m_s.is_finite() && error_m_s <= self.absolute_tolerance_m_s,
                "derived velocity mismatch at row {row}"
            );
        }
        Ok(velocities)
    }

    /// Checks all retained raw/procedure/calibration bytes and nominal reconstruction.
    /// This proves neither calibration authenticity nor physical model acceptance.
    pub fn verify_files(
        &self,
        dataset: &SuspensionIdentificationDataset,
        manifest: &SuspensionPhysicalAcquisitionManifest,
        root: &std::path::Path,
    ) -> Result<Vec<f64>> {
        let velocities = self.validate(dataset, manifest)?;
        manifest.verify_files(dataset, root)?;
        Ok(velocities)
    }
}

/// Versioned numerical derivative, with no implicit filtering or resampling.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SuspensionDerivativeOperator {
    /// Three-point quadratic derivative at interior samples using actual times;
    /// the two endpoints use their adjacent two-point secant. Requires >= 3 rows.
    /// Interior values need one future position: offline use only.
    NonuniformThreePointSecantEndsV1,
}

impl SuspensionDerivativeOperator {
    /// Returns one velocity (m/s) per input timestamp (s), in unchanged row order.
    ///
    /// Position is in metres. Clocks must be finite and strictly increasing
    /// within this single acquisition. No sorting, trimming, filtering, nominal
    /// rate substitution, or cross-acquisition differencing is performed.
    /// Invalid inputs or non-finite arithmetic fail the entire realization.
    pub fn reconstruct(self, timestamps_s: &[f64], positions_m: &[f64]) -> Result<Vec<f64>> {
        ensure!(
            timestamps_s.len() == positions_m.len() && (3..=100_000).contains(&timestamps_s.len()),
            "derivative requires 3..=100000 matching time/position rows"
        );
        ensure!(
            timestamps_s
                .iter()
                .chain(positions_m)
                .all(|x| x.is_finite()),
            "non-finite derivative input"
        );
        let mut intervals_s = Vec::with_capacity(timestamps_s.len() - 1);
        let mut slopes_m_s = Vec::with_capacity(timestamps_s.len() - 1);
        for (t, x) in timestamps_s.windows(2).zip(positions_m.windows(2)) {
            let dt_s = t[1] - t[0];
            ensure!(
                dt_s.is_finite() && dt_s > 0.0,
                "invalid derivative clock interval"
            );
            let slope_m_s = (x[1] - x[0]) / dt_s;
            ensure!(slope_m_s.is_finite(), "non-finite derivative secant");
            intervals_s.push(dt_s);
            slopes_m_s.push(slope_m_s);
        }
        let mut velocities_m_s = Vec::with_capacity(timestamps_s.len());
        velocities_m_s.push(slopes_m_s[0]);
        for i in 1..timestamps_s.len() - 1 {
            let left_s = intervals_s[i - 1];
            let right_s = intervals_s[i];
            // Scale before summation to avoid overflow for large valid intervals.
            let scale_s = left_s.max(right_s);
            let left = left_s / scale_s;
            let right = right_s / scale_s;
            // Derivative of the interpolating quadratic, expressed as weighted
            // adjacent secants to cancel a constant position offset naturally.
            let velocity_m_s = (right / (left + right)) * slopes_m_s[i - 1]
                + (left / (left + right)) * slopes_m_s[i];
            ensure!(velocity_m_s.is_finite(), "non-finite derivative result");
            velocities_m_s.push(velocity_m_s);
        }
        velocities_m_s.push(slopes_m_s[slopes_m_s.len() - 1]);
        Ok(velocities_m_s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const OP: SuspensionDerivativeOperator =
        SuspensionDerivativeOperator::NonuniformThreePointSecantEndsV1;

    fn close(actual: &[f64], expected: &[f64]) {
        assert_eq!(actual.len(), expected.len());
        for (a, b) in actual.iter().zip(expected) {
            assert!((a - b).abs() < 1e-12, "{a} != {b}");
        }
    }

    #[test]
    fn nonuniform_quadratic_and_explicit_secant_endpoints() {
        let time = [0.0, 0.25, 1.0, 2.0];
        let position = time.map(|t| t * t);
        let velocity = OP.reconstruct(&time, &position).unwrap();
        close(&velocity, &[0.25, 0.5, 2.0, 3.0]);
        assert_eq!(velocity, OP.reconstruct(&time, &position).unwrap());
        let local_clock = time.map(|t| t + 128.0);
        close(&OP.reconstruct(&local_clock, &position).unwrap(), &velocity);
    }

    #[test]
    fn common_offset_sample_noise_and_clock_scale_are_distinct() {
        let time = [0.0, 1.0, 2.0, 3.0, 4.0];
        let position = [0.0; 5];
        close(
            &OP.reconstruct(&time, &position.map(|x| x + 8.0)).unwrap(),
            &[0.0; 5],
        );
        // One position error affects two neighbouring velocities with opposite
        // signs: independent velocity draws would miss this coupling.
        let noisy = [0.0, 0.0, 2.0, 0.0, 0.0];
        close(
            &OP.reconstruct(&time, &noisy).unwrap(),
            &[0.0, 1.0, 0.0, -1.0, 0.0],
        );
        close(
            &OP.reconstruct(&time.map(|t| t * 2.0), &noisy).unwrap(),
            &[0.0, 0.5, 0.0, -0.5, 0.0],
        );
    }

    #[test]
    fn malformed_or_overflowing_realization_is_not_trimmed() {
        for time in [
            [0.0, 0.0, 1.0],
            [0.0, 2.0, 1.0],
            [0.0, f64::NAN, 2.0],
            [-f64::MAX, f64::MAX, f64::MAX],
        ] {
            assert!(OP.reconstruct(&time, &[0.0; 3]).is_err());
        }
        assert!(OP
            .reconstruct(&[0.0, 1.0, 2.0], &[0.0, f64::INFINITY, 0.0])
            .is_err());
        assert!(OP
            .reconstruct(&[0.0, 1.0, 2.0], &[-f64::MAX, f64::MAX, 0.0])
            .is_err());
        assert!(OP
            .reconstruct(&[0.0, 1e-320, 1.0], &[0.0, 1.0, 2.0])
            .is_err());
        assert!(OP.reconstruct(&[0.0, 1.0], &[0.0, 1.0]).is_err());
        assert!(OP.reconstruct(&[0.0, 1.0, 2.0], &[0.0, 1.0]).is_err());
        assert!(OP
            .reconstruct(&vec![0.0; 100_001], &vec![0.0; 100_001])
            .is_err());
    }

    #[test]
    fn operator_identity_is_explicit_and_versioned() {
        let bytes = serde_json::to_vec(&OP).unwrap();
        assert_eq!(
            serde_json::from_slice::<SuspensionDerivativeOperator>(&bytes).unwrap(),
            OP
        );
        assert!(serde_json::from_str::<SuspensionDerivativeOperator>("\"default\"").is_err());
        assert!(serde_json::from_str::<SuspensionDerivativeOperator>(
            "\"nonuniform_three_point_secant_ends_v2\""
        )
        .is_err());
    }
}
