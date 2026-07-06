//! Browser WASM viewer for Robot Native Engine.

#![deny(missing_docs)]

#[cfg(target_arch = "wasm32")]
mod app;
mod scene;

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

/// Starts the interactive web viewer on the page canvas.
#[cfg(target_arch = "wasm32")]
#[wasm_bindgen(start)]
pub fn start() {
    app::run();
}

pub use scene::WebScene;
