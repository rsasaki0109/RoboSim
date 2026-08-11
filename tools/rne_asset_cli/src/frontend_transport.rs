//! Reconnecting binary runner frontend transport.

use anyhow::{Context as _, Result};
use rne_core::control::{ControlCommand, RunnerControl, RunnerControlState};
use rne_data::transport::{
    decode_control_command, negotiate_transport, BoundedEgressQueue, ClientHello, ControlAck,
    EgressKey, GapNotice, NegotiationPolicy, NegotiationReject, NegotiationRejectCode,
    SensorFrameMetadata, ServerHello, StatusMessage, TransportCapabilities, TransportFrame,
    TransportMessageKind, TRANSPORT_MAX_PAYLOAD_BYTES, TRANSPORT_PROTOCOL_MAJOR,
};
use rne_data::{DataBus, Frame, ImageDepth, ImageRgb8, InMemoryDataBus, PointCloud};
use std::collections::BTreeMap;
use std::io;
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicU8, Ordering};
use std::sync::{mpsc, Arc, Condvar, Mutex, TryLockError};
use std::thread;
use std::time::Duration;

const HANDSHAKE_TIMEOUT: Duration = Duration::from_secs(5);
const WRITE_TIMEOUT: Duration = Duration::from_millis(500);
const ACCEPT_POLL_INTERVAL: Duration = Duration::from_millis(10);
const WRITER_POLL_INTERVAL: Duration = Duration::from_millis(10);
const DEFAULT_QUEUE_FRAMES: usize = 32;
const DEFAULT_QUEUE_BYTES: usize = 64 * 1024 * 1024;

/// Runner-control endpoint consumed by [`rne_core::control::RunControl`].
pub(crate) struct BinaryFrontendControl {
    receiver: mpsc::Receiver<ControlCommand>,
    shared: Arc<Shared>,
    paused: bool,
}

/// Cloneable non-blocking publisher for typed sensor payloads.
#[derive(Clone, Debug)]
pub(crate) struct BinaryFrontendPublisher {
    shared: Arc<Shared>,
}

#[derive(Debug)]
struct Shared {
    session_id: u64,
    queue: Mutex<BoundedEgressQueue>,
    queue_wake: Condvar,
    connected: AtomicBool,
    shutdown: AtomicBool,
    capabilities: AtomicU64,
    max_payload_bytes: AtomicU32,
    next_sequence: AtomicU64,
    reconnect_generation: AtomicU32,
    dropped_messages: AtomicU64,
    intended_state: AtomicU8,
    last_sensor_sequences: Mutex<BTreeMap<(u64, u16), u64>>,
}

impl Shared {
    fn new(session_id: u64) -> Self {
        Self {
            session_id,
            queue: Mutex::new(
                BoundedEgressQueue::new(DEFAULT_QUEUE_FRAMES, DEFAULT_QUEUE_BYTES)
                    .expect("default queue limits are valid"),
            ),
            queue_wake: Condvar::new(),
            connected: AtomicBool::new(false),
            shutdown: AtomicBool::new(false),
            capabilities: AtomicU64::new(0),
            max_payload_bytes: AtomicU32::new(TRANSPORT_MAX_PAYLOAD_BYTES as u32),
            next_sequence: AtomicU64::new(0),
            reconnect_generation: AtomicU32::new(0),
            dropped_messages: AtomicU64::new(0),
            intended_state: AtomicU8::new(1),
            last_sensor_sequences: Mutex::new(BTreeMap::new()),
        }
    }

    fn next_sequence(&self) -> Option<u64> {
        self.next_sequence
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |sequence| {
                sequence.checked_add(1)
            })
            .ok()
            .and_then(|previous| previous.checked_add(1))
    }

    fn capabilities(&self) -> TransportCapabilities {
        TransportCapabilities::from_bits(self.capabilities.load(Ordering::Acquire))
    }

    fn has_capability(&self, capability: TransportCapabilities) -> bool {
        self.connected.load(Ordering::Acquire) && self.capabilities().contains(capability)
    }

    fn configured_payload_limit(&self) -> usize {
        self.max_payload_bytes.load(Ordering::Acquire) as usize
    }

    fn configure_connection(
        &self,
        capabilities: TransportCapabilities,
        max_payload_bytes: u32,
        queue_frame_limit: u32,
        queue_byte_limit: u32,
    ) -> Result<()> {
        let queue = BoundedEgressQueue::new(queue_frame_limit as usize, queue_byte_limit as usize)
            .map_err(|error| anyhow::anyhow!(error))?;
        *self
            .queue
            .lock()
            .map_err(|_| anyhow::anyhow!("frontend queue lock poisoned"))? = queue;
        self.last_sensor_sequences
            .lock()
            .map_err(|_| anyhow::anyhow!("sensor sequence lock poisoned"))?
            .clear();
        self.capabilities
            .store(capabilities.bits(), Ordering::Release);
        self.max_payload_bytes
            .store(max_payload_bytes, Ordering::Release);
        self.connected.store(true, Ordering::Release);
        Ok(())
    }

    fn disconnect(&self) {
        self.connected.store(false, Ordering::Release);
        self.capabilities.store(0, Ordering::Release);
        if let Ok(mut queue) = self.queue.lock() {
            *queue = BoundedEgressQueue::new(DEFAULT_QUEUE_FRAMES, DEFAULT_QUEUE_BYTES)
                .expect("default queue limits are valid");
        }
        self.queue_wake.notify_all();
    }

    fn enqueue_latest(&self, kind: TransportMessageKind, key: EgressKey, payload: Vec<u8>) {
        if !self.connected.load(Ordering::Acquire)
            || payload.len() > self.configured_payload_limit()
        {
            return;
        }
        let Some(sequence) = self.next_sequence() else {
            self.disconnect();
            return;
        };
        let frame = TransportFrame::new(kind, sequence, self.session_id, payload);
        let mut queue = match self.queue.try_lock() {
            Ok(queue) => queue,
            Err(TryLockError::WouldBlock) => {
                self.dropped_messages.fetch_add(1, Ordering::AcqRel);
                return;
            }
            Err(TryLockError::Poisoned(_)) => {
                self.disconnect();
                return;
            }
        };
        let _ = queue.push_latest(key, frame);
        if let Some(range) = queue.take_last_dropped_range() {
            let total = self
                .dropped_messages
                .fetch_add(range.count, Ordering::AcqRel)
                .saturating_add(range.count);
            if queue.max_frames() > 1 {
                if let Some(gap_sequence) = self.next_sequence() {
                    let notice = GapNotice {
                        first_missing_sequence: range.first_sequence,
                        last_missing_sequence: range.last_sequence,
                        total_dropped_messages: total,
                    };
                    let gap = TransportFrame::new(
                        TransportMessageKind::Gap,
                        gap_sequence,
                        self.session_id,
                        notice.encode_payload(),
                    );
                    let _ = queue.push_latest(EgressKey::Gap, gap);
                    if let Some(gap_range) = queue.take_last_dropped_range() {
                        self.dropped_messages
                            .fetch_add(gap_range.count, Ordering::AcqRel);
                    }
                }
            }
        }
        drop(queue);
        self.queue_wake.notify_one();
    }

    fn enqueue_reliable(&self, kind: TransportMessageKind, payload: Vec<u8>) -> Result<()> {
        if payload.len() > self.configured_payload_limit() {
            anyhow::bail!("reliable frontend payload exceeds negotiated limit");
        }
        let sequence = self
            .next_sequence()
            .ok_or_else(|| anyhow::anyhow!("transport sequence exhausted"))?;
        let frame = TransportFrame::new(kind, sequence, self.session_id, payload);
        self.queue
            .lock()
            .map_err(|_| anyhow::anyhow!("frontend queue lock poisoned"))?
            .push_reliable(frame)
            .map_err(|error| anyhow::anyhow!(error))?;
        self.queue_wake.notify_one();
        Ok(())
    }

    fn pop_waiting(&self, alive: &AtomicBool) -> Option<TransportFrame> {
        let mut queue = self.queue.lock().ok()?;
        loop {
            if let Some(frame) = queue.pop_front() {
                return Some(frame);
            }
            if !alive.load(Ordering::Acquire) || self.shutdown.load(Ordering::Acquire) {
                return None;
            }
            let waited = self
                .queue_wake
                .wait_timeout(queue, WRITER_POLL_INTERVAL)
                .ok()?;
            queue = waited.0;
        }
    }
}

impl BinaryFrontendControl {
    /// Binds the production frontend listener and starts its reconnect loop.
    pub(crate) fn start(port: u16) -> Result<(Self, BinaryFrontendPublisher, u16)> {
        let listener = TcpListener::bind(("127.0.0.1", port))
            .with_context(|| format!("bind binary frontend listener on port {port}"))?;
        listener
            .set_nonblocking(true)
            .context("set binary frontend listener non-blocking")?;
        let bound_port = listener
            .local_addr()
            .context("query binary frontend listener address")?
            .port();
        let session_id = (u64::from(std::process::id()) << 32) | u64::from(bound_port);
        let shared = Arc::new(Shared::new(session_id));
        let thread_shared = shared.clone();
        let (sender, receiver) = mpsc::channel();
        thread::Builder::new()
            .name("rne-frontend-binary".into())
            .spawn(move || accept_connections(listener, sender, thread_shared))
            .context("spawn binary frontend listener")?;
        Ok((
            Self {
                receiver,
                shared: shared.clone(),
                paused: true,
            },
            BinaryFrontendPublisher { shared },
            bound_port,
        ))
    }

    fn update_state(&mut self, command: ControlCommand) {
        self.paused = match command {
            ControlCommand::Pause | ControlCommand::Step { .. } | ControlCommand::Reset => true,
            ControlCommand::Resume => false,
            ControlCommand::Quit => self.paused,
        };
    }
}

impl Drop for BinaryFrontendControl {
    fn drop(&mut self) {
        self.shared.shutdown.store(true, Ordering::Release);
        self.shared.queue_wake.notify_all();
    }
}

impl RunnerControl for BinaryFrontendControl {
    fn try_poll(&mut self) -> Option<ControlCommand> {
        let command = self.receiver.try_recv().ok();
        if let Some(command) = command {
            self.update_state(command);
        }
        command
    }

    fn wait_command(&mut self) -> ControlCommand {
        let command = self.receiver.recv().unwrap_or(ControlCommand::Quit);
        self.update_state(command);
        command
    }

    fn report_status(&mut self, step: u64, sim_time_s: f64, snapshot: &[u8]) {
        if !self.shared.has_capability(TransportCapabilities::STATUS) {
            return;
        }
        let sim_time_ticks = (sim_time_s * 1_000_000_000.0).round().max(0.0) as u64;
        let status = StatusMessage {
            step,
            sim_time_ticks,
            state: if self.paused {
                RunnerControlState::Paused
            } else {
                RunnerControlState::Running
            },
            snapshot_json: snapshot.to_vec(),
        };
        if let Ok(payload) = status.encode_payload() {
            self.shared
                .enqueue_latest(TransportMessageKind::Status, EgressKey::Status, payload);
        }
    }
}

impl BinaryFrontendPublisher {
    /// Returns true when a negotiated client currently wants this capability.
    pub(crate) fn wants(&self, capability: TransportCapabilities) -> bool {
        self.shared.has_capability(capability)
    }

    /// Publishes one already validated typed sensor payload at most once per
    /// sensor sequence for the current connection.
    pub(crate) fn publish_sensor(
        &self,
        kind: TransportMessageKind,
        stream_id: u64,
        sensor_sequence: u64,
        payload: Vec<u8>,
    ) {
        let capability = match kind {
            TransportMessageKind::ImageRgb8 => TransportCapabilities::IMAGE_RGB8,
            TransportMessageKind::ImageDepthF32 => TransportCapabilities::IMAGE_DEPTH_F32,
            TransportMessageKind::LidarPointCloud => TransportCapabilities::LIDAR_POINT_CLOUD,
            _ => return,
        };
        if !self.shared.has_capability(capability) {
            return;
        }
        let key = (stream_id, kind as u16);
        let mut sequences = match self.shared.last_sensor_sequences.try_lock() {
            Ok(sequences) => sequences,
            Err(TryLockError::WouldBlock) => {
                self.shared.dropped_messages.fetch_add(1, Ordering::AcqRel);
                return;
            }
            Err(TryLockError::Poisoned(_)) => {
                self.shared.disconnect();
                return;
            }
        };
        if sequences
            .get(&key)
            .is_some_and(|last| *last >= sensor_sequence)
        {
            return;
        }
        sequences.insert(key, sensor_sequence);
        drop(sequences);
        self.shared
            .enqueue_latest(kind, EgressKey::Sensor { stream_id, kind }, payload);
    }

    #[cfg(test)]
    fn connected(&self) -> bool {
        self.shared.connected.load(Ordering::Acquire)
    }
}

fn accept_connections(
    listener: TcpListener,
    sender: mpsc::Sender<ControlCommand>,
    shared: Arc<Shared>,
) {
    while !shared.shutdown.load(Ordering::Acquire) {
        match listener.accept() {
            Ok((stream, _peer)) => {
                if let Err(error) = serve_connection(stream, &sender, &shared) {
                    eprintln!("frontend: connection closed: {error:#}");
                }
                shared.disconnect();
            }
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                thread::sleep(ACCEPT_POLL_INTERVAL);
            }
            Err(_) => break,
        }
    }
}

fn serve_connection(
    mut stream: TcpStream,
    sender: &mpsc::Sender<ControlCommand>,
    shared: &Arc<Shared>,
) -> Result<()> {
    // Windows may inherit the listener's non-blocking mode on accepted
    // sockets. Connection I/O uses dedicated threads and finite deadlines, so
    // restore blocking semantics explicitly on every platform.
    stream
        .set_nonblocking(false)
        .context("set accepted frontend socket blocking")?;
    stream
        .set_read_timeout(Some(HANDSHAKE_TIMEOUT))
        .context("set frontend handshake read timeout")?;
    stream
        .set_write_timeout(Some(WRITE_TIMEOUT))
        .context("set frontend write timeout")?;
    let Some(frame) = TransportFrame::read_from(&mut stream, TRANSPORT_MAX_PAYLOAD_BYTES)
        .context("read frontend ClientHello")?
    else {
        return Ok(());
    };
    if frame.kind != TransportMessageKind::ClientHello {
        write_rejection(
            &mut stream,
            shared,
            NegotiationReject {
                code: NegotiationRejectCode::InvalidVersionRange,
                message: "first frame must be ClientHello".to_string(),
            },
        )?;
        return Ok(());
    }
    if frame.session_id != 0 && frame.session_id != shared.session_id {
        write_rejection(
            &mut stream,
            shared,
            NegotiationReject {
                code: NegotiationRejectCode::InvalidVersionRange,
                message: "resume session id does not match this runner".to_string(),
            },
        )?;
        return Ok(());
    }
    let hello = ClientHello::decode_payload(&frame.payload).context("decode ClientHello")?;
    let negotiated = match negotiate_transport(hello, NegotiationPolicy::default()) {
        Ok(negotiated) => negotiated,
        Err(rejection) => {
            write_rejection(&mut stream, shared, rejection)?;
            return Ok(());
        }
    };
    if let Some(resume_after) = negotiated.resume_after_sequence {
        if resume_after > shared.next_sequence.load(Ordering::Acquire) {
            write_rejection(
                &mut stream,
                shared,
                NegotiationReject {
                    code: NegotiationRejectCode::InvalidVersionRange,
                    message: "resume cursor is ahead of the server".to_string(),
                },
            )?;
            return Ok(());
        }
    }

    shared.configure_connection(
        negotiated.capabilities,
        negotiated.max_payload_bytes,
        negotiated.queue_frame_limit,
        negotiated.queue_byte_limit,
    )?;
    let reconnect_generation = shared
        .reconnect_generation
        .fetch_add(1, Ordering::AcqRel)
        .saturating_add(1);
    let current_before_hello = shared.next_sequence.load(Ordering::Acquire);
    let hello_sequence = shared
        .next_sequence()
        .ok_or_else(|| anyhow::anyhow!("transport sequence exhausted"))?;
    let server_hello = ServerHello {
        negotiated,
        reconnect_generation,
        current_sequence: current_before_hello,
        dropped_messages: shared.dropped_messages.load(Ordering::Acquire),
    };
    TransportFrame::new(
        TransportMessageKind::ServerHello,
        hello_sequence,
        shared.session_id,
        server_hello.encode_payload(),
    )
    .write_to(&mut stream)
    .context("write ServerHello")?;

    if let Some(resume_after) = negotiated.resume_after_sequence {
        if resume_after < current_before_hello
            && negotiated
                .capabilities
                .contains(TransportCapabilities::RESUME_LATEST)
        {
            let sequence = shared
                .next_sequence()
                .ok_or_else(|| anyhow::anyhow!("transport sequence exhausted"))?;
            let gap = GapNotice {
                first_missing_sequence: resume_after.saturating_add(1),
                last_missing_sequence: current_before_hello,
                total_dropped_messages: shared.dropped_messages.load(Ordering::Acquire),
            };
            TransportFrame::new(
                TransportMessageKind::Gap,
                sequence,
                shared.session_id,
                gap.encode_payload(),
            )
            .write_to(&mut stream)
            .context("write reconnect gap")?;
        }
    }

    stream
        .set_read_timeout(None)
        .context("clear frontend read timeout")?;
    let read_stream = stream.try_clone().context("clone frontend stream")?;
    let alive = Arc::new(AtomicBool::new(true));
    let reader_alive = alive.clone();
    let reader_shared = shared.clone();
    let reader_sender = sender.clone();
    let max_payload_bytes = negotiated.max_payload_bytes as usize;
    let reader = thread::Builder::new()
        .name("rne-frontend-reader".into())
        .spawn(move || {
            read_commands(
                read_stream,
                &reader_sender,
                &reader_shared,
                &reader_alive,
                max_payload_bytes,
            )
        })
        .context("spawn frontend command reader")?;

    while alive.load(Ordering::Acquire) && !shared.shutdown.load(Ordering::Acquire) {
        let Some(frame) = shared.pop_waiting(&alive) else {
            break;
        };
        if frame.write_to(&mut stream).is_err() {
            alive.store(false, Ordering::Release);
            break;
        }
    }
    alive.store(false, Ordering::Release);
    let _ = stream.shutdown(std::net::Shutdown::Both);
    let _ = reader.join();
    Ok(())
}

fn read_commands(
    mut stream: TcpStream,
    sender: &mpsc::Sender<ControlCommand>,
    shared: &Shared,
    alive: &AtomicBool,
    max_payload_bytes: usize,
) {
    while alive.load(Ordering::Acquire) && !shared.shutdown.load(Ordering::Acquire) {
        let frame = match TransportFrame::read_from(&mut stream, max_payload_bytes) {
            Ok(Some(frame)) => frame,
            Ok(None) => break,
            Err(error) => {
                eprintln!("frontend: command stream failed: {error}");
                break;
            }
        };
        if frame.protocol_major != TRANSPORT_PROTOCOL_MAJOR
            || frame.session_id != shared.session_id
            || frame.kind != TransportMessageKind::ControlCommand
        {
            eprintln!(
                "frontend: invalid command frame major={} session={} kind={:?}",
                frame.protocol_major, frame.session_id, frame.kind
            );
            break;
        }
        let command = match decode_control_command(&frame.payload) {
            Ok(command) => command,
            Err(error) => {
                eprintln!("frontend: invalid command payload: {error}");
                break;
            }
        };
        let intended_state = match command {
            ControlCommand::Pause | ControlCommand::Step { .. } | ControlCommand::Reset => 1,
            ControlCommand::Resume => 0,
            ControlCommand::Quit => shared.intended_state.load(Ordering::Acquire),
        };
        shared
            .intended_state
            .store(intended_state, Ordering::Release);
        let ack = ControlAck {
            command_sequence: frame.sequence,
            state: if intended_state == 0 {
                RunnerControlState::Running
            } else {
                RunnerControlState::Paused
            },
        };
        if shared
            .enqueue_reliable(TransportMessageKind::ControlAck, ack.encode_payload())
            .is_err()
        {
            eprintln!("frontend: reliable acknowledgement queue is full");
            break;
        }
        if sender.send(command).is_err() {
            eprintln!("frontend: runner command receiver closed");
            break;
        }
    }
    alive.store(false, Ordering::Release);
    shared.queue_wake.notify_all();
}

fn write_rejection(
    stream: &mut TcpStream,
    shared: &Shared,
    rejection: NegotiationReject,
) -> Result<()> {
    let sequence = shared
        .next_sequence()
        .ok_or_else(|| anyhow::anyhow!("transport sequence exhausted"))?;
    TransportFrame::new(
        TransportMessageKind::Reject,
        sequence,
        shared.session_id,
        rejection.encode_payload(),
    )
    .write_to(stream)
    .context("write frontend rejection")
}

/// Converts a typed DataBus frame into protocol metadata.
pub(crate) fn sensor_metadata<T: rne_data::FramePayload>(frame: &Frame<T>) -> SensorFrameMetadata {
    SensorFrameMetadata {
        stream_id: frame.stream_id.0,
        sensor_sequence: frame.sequence,
        capture_ticks: frame.capture_time.ticks(),
        available_ticks: frame.available_time.ticks(),
    }
}

/// Publishes the latest camera pair and LiDAR cloud for selected stream ids.
pub(crate) fn publish_bulk_sensors<'a>(
    publisher: &BinaryFrontendPublisher,
    bus: &InMemoryDataBus,
    streams: impl IntoIterator<Item = (u64, &'a str)>,
    camera_depth_stream_offset: u64,
) {
    for (stream_id, kind) in streams {
        let stream = rne_data::StreamId::new(stream_id);
        match kind {
            "camera" => {
                if publisher.wants(TransportCapabilities::IMAGE_RGB8) {
                    if let Some(frame) = bus.latest::<ImageRgb8>(stream) {
                        if let Ok(payload) = rne_data::transport::encode_image_rgb8(
                            sensor_metadata(&frame),
                            &frame.payload,
                        ) {
                            publisher.publish_sensor(
                                TransportMessageKind::ImageRgb8,
                                stream_id,
                                frame.sequence,
                                payload,
                            );
                        }
                    }
                }
                if publisher.wants(TransportCapabilities::IMAGE_DEPTH_F32) {
                    let depth_stream_id = stream_id.saturating_add(camera_depth_stream_offset);
                    let depth_stream = rne_data::StreamId::new(depth_stream_id);
                    if let Some(frame) = bus.latest::<ImageDepth>(depth_stream) {
                        if let Ok(payload) = rne_data::transport::encode_image_depth(
                            sensor_metadata(&frame),
                            &frame.payload,
                        ) {
                            publisher.publish_sensor(
                                TransportMessageKind::ImageDepthF32,
                                depth_stream_id,
                                frame.sequence,
                                payload,
                            );
                        }
                    }
                }
            }
            "lidar" if publisher.wants(TransportCapabilities::LIDAR_POINT_CLOUD) => {
                if let Some(frame) = bus.latest::<PointCloud>(stream) {
                    if let Ok(payload) = rne_data::transport::encode_lidar_point_cloud(
                        sensor_metadata(&frame),
                        &frame.payload,
                    ) {
                        publisher.publish_sensor(
                            TransportMessageKind::LidarPointCloud,
                            stream_id,
                            frame.sequence,
                            payload,
                        );
                    }
                }
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_data::transport::{
        encode_control_command, ControlAck, GapNotice, ServerHello, TransportMessageKind,
    };
    use std::thread;

    fn connect(
        port: u16,
        session_id: u64,
        resume_after_sequence: Option<u64>,
    ) -> (TcpStream, TransportFrame, ServerHello) {
        let mut stream = TcpStream::connect(("127.0.0.1", port)).expect("connect frontend");
        stream
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        let hello = ClientHello {
            min_protocol_major: 1,
            max_protocol_major: 1,
            capabilities: TransportCapabilities::ALL_V1,
            required_capabilities: TransportCapabilities::CONTROL,
            max_payload_bytes: 1024 * 1024,
            queue_frame_limit: 8,
            queue_byte_limit: 2 * 1024 * 1024,
            resume_after_sequence,
        };
        TransportFrame::new(
            TransportMessageKind::ClientHello,
            1,
            session_id,
            hello.encode_payload(),
        )
        .write_to(&mut stream)
        .unwrap();
        let frame = TransportFrame::read_from(&mut stream, TRANSPORT_MAX_PAYLOAD_BYTES)
            .unwrap()
            .expect("ServerHello");
        assert_eq!(frame.kind, TransportMessageKind::ServerHello);
        let server = ServerHello::decode_payload(&frame.payload).unwrap();
        (stream, frame, server)
    }

    fn wait_for(predicate: impl Fn() -> bool) {
        for _ in 0..100 {
            if predicate() {
                return;
            }
            thread::sleep(Duration::from_millis(10));
        }
        panic!("condition did not become true");
    }

    #[test]
    fn disconnect_does_not_quit_and_same_session_reconnects() {
        let (mut control, publisher, port) = BinaryFrontendControl::start(0).unwrap();
        let (mut first, hello_frame, _) = connect(port, 0, None);
        wait_for(|| publisher.connected());
        let session_id = hello_frame.session_id;

        let command = TransportFrame::new(
            TransportMessageKind::ControlCommand,
            10,
            session_id,
            encode_control_command(ControlCommand::Step { frames: 1 }),
        );
        command.write_to(&mut first).unwrap();
        assert_eq!(control.wait_command(), ControlCommand::Step { frames: 1 });
        let ack = TransportFrame::read_from(&mut first, TRANSPORT_MAX_PAYLOAD_BYTES)
            .unwrap()
            .unwrap();
        assert_eq!(ack.kind, TransportMessageKind::ControlAck);
        assert_eq!(
            ControlAck::decode_payload(&ack.payload)
                .unwrap()
                .command_sequence,
            10
        );

        drop(first);
        wait_for(|| !publisher.connected());
        assert!(
            control.try_poll().is_none(),
            "disconnect must not synthesize quit"
        );

        let (mut second, _, server) = connect(port, session_id, Some(hello_frame.sequence));
        assert_eq!(server.reconnect_generation, 2);
        let gap = TransportFrame::read_from(&mut second, TRANSPORT_MAX_PAYLOAD_BYTES)
            .unwrap()
            .unwrap();
        assert_eq!(gap.kind, TransportMessageKind::Gap);
        assert!(GapNotice::decode_payload(&gap.payload).is_ok());

        TransportFrame::new(
            TransportMessageKind::ControlCommand,
            11,
            session_id,
            encode_control_command(ControlCommand::Quit),
        )
        .write_to(&mut second)
        .unwrap();
        assert_eq!(control.wait_command(), ControlCommand::Quit);
    }

    #[test]
    fn unsupported_protocol_is_rejected_explicitly() {
        let (control, _publisher, port) = BinaryFrontendControl::start(0).unwrap();
        let mut stream = TcpStream::connect(("127.0.0.1", port)).unwrap();
        let hello = ClientHello {
            min_protocol_major: 9,
            max_protocol_major: 10,
            capabilities: TransportCapabilities::ALL_V1,
            required_capabilities: TransportCapabilities::CONTROL,
            max_payload_bytes: 1024,
            queue_frame_limit: 2,
            queue_byte_limit: 2048,
            resume_after_sequence: None,
        };
        TransportFrame::new(
            TransportMessageKind::ClientHello,
            1,
            0,
            hello.encode_payload(),
        )
        .write_to(&mut stream)
        .unwrap();
        let rejection = TransportFrame::read_from(&mut stream, 4096)
            .unwrap()
            .unwrap();
        assert_eq!(rejection.kind, TransportMessageKind::Reject);
        assert_eq!(
            NegotiationReject::decode_payload(&rejection.payload)
                .unwrap()
                .code,
            NegotiationRejectCode::UnsupportedVersion
        );
        drop(control);
    }
}
