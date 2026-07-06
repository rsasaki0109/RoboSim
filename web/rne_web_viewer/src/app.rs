//! winit application loop for the browser viewer.

use crate::scene::WebScene;
use rne_render_wgpu::{CameraOrbit, InteractiveViewer, ViewerError};
use std::sync::Arc;
use winit::application::ApplicationHandler;
use winit::dpi::PhysicalPosition;
use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, EventLoop, EventLoopProxy};
use winit::window::{Window, WindowId};

const CLEAR_COLOR: [f32; 4] = [0.05, 0.08, 0.12, 1.0];

/// GPU initialization completion event for wasm async startup.
struct ViewerInitEvent(Result<InteractiveViewer, ViewerError>);

enum InitState {
    Pending,
    Ready {
        viewer: InteractiveViewer,
        scene: WebScene,
    },
    Failed(String),
}

impl Default for InitState {
    fn default() -> Self {
        Self::Pending
    }
}

/// Runs the viewer event loop until the page is closed.
pub fn run() {
    let event_loop = EventLoop::<ViewerInitEvent>::with_user_event()
        .build()
        .expect("create event loop");
    let proxy = event_loop.create_proxy();
    let mut app = App {
        proxy: Some(proxy),
        ..App::default()
    };
    event_loop.run_app(&mut app).expect("run web viewer");
}

#[derive(Default)]
struct App {
    proxy: Option<EventLoopProxy<ViewerInitEvent>>,
    window: Option<Arc<Window>>,
    init: InitState,
    pending_scene: Option<WebScene>,
    orbit: CameraOrbit,
    frame_index: u64,
    dragging: bool,
    last_cursor: Option<PhysicalPosition<f64>>,
}

impl ApplicationHandler<ViewerInitEvent> for App {
    fn resumed(&mut self, _event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }

        let scene = match WebScene::load_mm_minimal() {
            Ok(scene) => scene,
            Err(error) => {
                self.init = InitState::Failed(error);
                return;
            }
        };
        self.orbit.focus = scene.focus();
        self.pending_scene = Some(scene);

        #[cfg_attr(
            not(target_arch = "wasm32"),
            expect(unused_mut, reason = "wasm attaches canvas")
        )]
        let mut attributes = Window::default_attributes().with_title("RNE Web Viewer — mm_minimal");

        #[cfg(target_arch = "wasm32")]
        {
            use wasm_bindgen::JsCast;
            use winit::platform::web::WindowAttributesExtWebSys;
            let canvas = web_sys::window()
                .and_then(|window| window.document())
                .and_then(|document| document.get_element_by_id("canvas"))
                .and_then(|element| element.dyn_into::<web_sys::HtmlCanvasElement>().ok())
                .expect("canvas element with id `canvas`");
            attributes = attributes.with_canvas(Some(canvas));
        }

        let window = Arc::new(
            _event_loop
                .create_window(attributes)
                .expect("create window"),
        );
        self.window = Some(window.clone());
        self.init = InitState::Pending;

        let proxy = self
            .proxy
            .clone()
            .expect("event loop proxy should be set before resumed");
        #[cfg(target_arch = "wasm32")]
        wasm_bindgen_futures::spawn_local(async move {
            let result = InteractiveViewer::new_async(window).await;
            let _ = proxy.send_event(ViewerInitEvent(result));
        });
        #[cfg(not(target_arch = "wasm32"))]
        {
            let result = InteractiveViewer::new(window);
            let _ = proxy.send_event(ViewerInitEvent(result));
        }
    }

    fn user_event(&mut self, event_loop: &ActiveEventLoop, event: ViewerInitEvent) {
        let Some(scene) = self.pending_scene.take() else {
            return;
        };

        match event.0 {
            Ok(viewer) => {
                self.init = InitState::Ready { viewer, scene };
                if let Some(window) = &self.window {
                    window.request_redraw();
                }
            }
            Err(error) => {
                self.init = InitState::Failed(error.to_string());
                event_loop.exit();
            }
        }
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: WindowId,
        event: WindowEvent,
    ) {
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => {
                if let InitState::Ready { viewer, .. } = &mut self.init {
                    viewer.resize(size.width, size.height);
                }
            }
            WindowEvent::MouseInput { state, button, .. } => {
                if button == MouseButton::Left {
                    self.dragging = state == ElementState::Pressed;
                    if state == ElementState::Released {
                        self.last_cursor = None;
                    }
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                if !self.dragging {
                    return;
                }
                if let Some(last) = self.last_cursor {
                    let dx = position.x - last.x;
                    let dy = position.y - last.y;
                    self.orbit.yaw_rad += dx * 0.005;
                    self.orbit.pitch_rad = (self.orbit.pitch_rad + dy * 0.004).clamp(0.15, 1.45);
                }
                self.last_cursor = Some(position);
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let scroll = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y as f64,
                    MouseScrollDelta::PixelDelta(delta) => delta.y * 0.002,
                };
                self.orbit.distance_m = (self.orbit.distance_m - scroll * 0.35).clamp(1.5, 12.0);
            }
            WindowEvent::RedrawRequested => {
                if let Err(error) = self.draw_frame() {
                    #[cfg(target_arch = "wasm32")]
                    web_sys::console::error_1(&error.into());
                    #[cfg(not(target_arch = "wasm32"))]
                    eprintln!("render error: {error}");
                    event_loop.exit();
                }
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }
}

impl App {
    fn draw_frame(&mut self) -> Result<(), String> {
        let InitState::Ready { viewer, scene } = &mut self.init else {
            return Ok(());
        };

        let render_scene = scene.frame(self.frame_index);
        self.frame_index = self.frame_index.saturating_add(1);
        self.orbit.focus = scene.focus();

        let view = self.orbit.camera_transform();
        viewer
            .render(&view, &render_scene, CLEAR_COLOR)
            .map_err(|error| error.to_string())
    }
}
