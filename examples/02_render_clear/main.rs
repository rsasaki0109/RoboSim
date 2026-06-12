//! Clears an off-screen wgpu render target.

use rne_render::{hash_rgba8, RenderBackend, RenderTarget};
use rne_render_wgpu::WgpuRenderBackend;

fn main() {
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        println!("RNE_SKIP_GPU set; skipping wgpu example");
        return;
    }

    let mut backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("wgpu unavailable: {error}");
            return;
        }
    };

    let frame = backend
        .render_clear(RenderTarget::new(64, 48), [0.1, 0.3, 0.5, 1.0])
        .expect("clear render");

    println!(
        "rendered {}x{} image, hash={:#018x}",
        frame.width,
        frame.height,
        hash_rgba8(&frame.rgba8)
    );
}
