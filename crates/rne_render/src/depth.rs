//! Depth buffer output from a camera pass.

/// Linear view-space depth in meters produced by a render pass.
#[derive(Clone, Debug, PartialEq)]
pub struct DepthFrame {
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
    /// Row-major linear depth values in meters.
    pub depth_m: Vec<f32>,
}

impl DepthFrame {
    /// Creates a depth frame from raw values.
    pub fn new(width: u32, height: u32, depth_m: Vec<f32>) -> Self {
        Self {
            width,
            height,
            depth_m,
        }
    }

    /// Returns a stable hash of the depth buffer for determinism tests.
    pub fn hash_depth(&self) -> u64 {
        hash_depth_f32(&self.depth_m)
    }
}

/// Computes a stable FNV-1a hash over depth values bit patterns.
pub fn hash_depth_f32(values: &[f32]) -> u64 {
    let mut hash = 0xcbf29ce484222325_u64;
    for value in values {
        for byte in value.to_bits().to_le_bytes() {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(0x100000001b3);
        }
    }
    hash
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn depth_hash_is_stable() {
        let frame = DepthFrame::new(2, 2, vec![1.0, 2.0, 3.0, 4.0]);
        assert_eq!(frame.hash_depth(), frame.hash_depth());
    }
}
