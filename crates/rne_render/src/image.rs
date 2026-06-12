//! Render targets and image frames.

/// Off-screen render target description.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RenderTarget {
    /// Width in pixels.
    pub width: u32,
    /// Height in pixels.
    pub height: u32,
}

impl RenderTarget {
    /// Creates a render target with the given dimensions.
    pub const fn new(width: u32, height: u32) -> Self {
        Self { width, height }
    }

    /// Returns the number of RGBA8 pixels.
    pub const fn pixel_count(&self) -> usize {
        self.width as usize * self.height as usize
    }

    /// Returns the byte length of an RGBA8 buffer for this target.
    pub const fn rgba8_len(&self) -> usize {
        self.pixel_count() * 4
    }
}

/// CPU-side RGBA8 image produced by a render backend.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ImageFrame {
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
    /// RGBA8 pixel data in row-major order.
    pub rgba8: Vec<u8>,
}

impl ImageFrame {
    /// Creates an image frame from RGBA8 bytes.
    pub fn from_rgba8(width: u32, height: u32, rgba8: Vec<u8>) -> Self {
        Self {
            width,
            height,
            rgba8,
        }
    }

    /// Returns a stable hash of the pixel buffer for determinism tests.
    pub fn hash_pixels(&self) -> u64 {
        hash_rgba8(&self.rgba8)
    }
}

/// Computes a stable FNV-1a hash over RGBA8 bytes.
pub fn hash_rgba8(bytes: &[u8]) -> u64 {
    let mut hash = 0xcbf29ce484222325_u64;
    for byte in bytes {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x100000001b3);
    }
    hash
}
