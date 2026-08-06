(() => {
  "use strict";

  const SUPPORTED_VERSION = 1;
  const fileInput = document.getElementById("replay-file");
  const playButton = document.getElementById("replay-play");
  const rangeInput = document.getElementById("replay-range");
  const statusOutput = document.getElementById("replay-status");
  const sceneOutput = document.getElementById("replay-scene");
  const clockOutput = document.getElementById("replay-clock");
  const frameOutput = document.getElementById("replay-frame");
  const actionOutput = document.getElementById("replay-action");
  const observationOutput = document.getElementById("replay-observation");
  const hashOutput = document.getElementById("replay-hash");
  const reportOutput = document.getElementById("replay-report");
  const progress = document.getElementById("replay-progress");

  const state = {
    artifact: null,
    frameIndex: 0,
    playing: false,
    lastTimestampMs: null,
    accumulatedMs: 0,
    animationId: null,
  };

  function setStatus(message, isError = false) {
    statusOutput.textContent = message;
    statusOutput.dataset.error = isError ? "true" : "false";
  }

  function validateArtifact(artifact) {
    if (!artifact || artifact.version !== SUPPORTED_VERSION) {
      throw new Error(
        `unsupported replay version (expected ${SUPPORTED_VERSION})`,
      );
    }
    if (typeof artifact.scene !== "string" || artifact.scene.trim() === "") {
      throw new Error("scene path is empty");
    }
    if (
      !artifact.clock ||
      !Number.isInteger(artifact.clock.steps) ||
      artifact.clock.steps < 0 ||
      !Number.isFinite(artifact.clock.hz) ||
      artifact.clock.hz <= 0
    ) {
      throw new Error("clock is invalid");
    }
    if (
      !Array.isArray(artifact.frames) ||
      artifact.frames.length !== artifact.clock.steps
    ) {
      throw new Error("frame count does not match clock.steps");
    }

    let previousTicks = null;
    artifact.frames.forEach((frame, index) => {
      if (!frame || frame.step !== index) {
        throw new Error(`frame ${index} is not sequential`);
      }
      if (
        previousTicks !== null &&
        (!Number.isInteger(frame.sim_ticks) || frame.sim_ticks <= previousTicks)
      ) {
        throw new Error(`frame ${index} has non-increasing sim_ticks`);
      }
      if (!Number.isInteger(frame.sim_ticks) || frame.sim_ticks < 0) {
        throw new Error(`frame ${index} has invalid sim_ticks`);
      }
      validateAction(frame.action, index);
      validateObservation(frame.observation, index);
      if (
        typeof frame.physics_hash !== "string" &&
        !Number.isInteger(frame.physics_hash)
      ) {
        throw new Error(`frame ${index} has an invalid physics hash`);
      }
      previousTicks = frame.sim_ticks;
    });
    validateFinalReport(artifact.final_report);
    return artifact;
  }

  function validateFinalReport(report) {
    if (!report || typeof report !== "object") {
      throw new Error("final report is missing");
    }
    if (
      report.contact_pairs_max !== undefined &&
      !Number.isInteger(report.contact_pairs_max)
    ) {
      throw new Error("final report has invalid contact_pairs_max");
    }
    if (
      report.contact_impulse_max_ns !== undefined &&
      !Number.isFinite(report.contact_impulse_max_ns)
    ) {
      throw new Error("final report has invalid contact_impulse_max_ns");
    }
    if (
      report.min_base_height_m !== undefined &&
      report.min_base_height_m !== null &&
      !Number.isFinite(report.min_base_height_m)
    ) {
      throw new Error("final report has invalid min_base_height_m");
    }
    if (
      report.failure !== undefined &&
      report.failure !== null &&
      typeof report.failure !== "string"
    ) {
      throw new Error("final report has invalid failure kind");
    }
  }

  function validateAction(action, index) {
    if (!action || typeof action !== "object") {
      throw new Error(`frame ${index} has an invalid action`);
    }
    const kind = action.kind || "differential_drive";
    if (kind === "differential_drive") {
      if (!Number.isFinite(action.wheel_velocity_rad_s)) {
        throw new Error(`frame ${index} has an invalid wheel action`);
      }
      return;
    }
    if (kind === "joint_velocity") {
      if (
        typeof action.joint !== "string" ||
        action.joint.trim() === "" ||
        !Number.isFinite(action.velocity_rad_s)
      ) {
        throw new Error(`frame ${index} has an invalid joint velocity action`);
      }
      return;
    }
    if (kind === "joint_effort") {
      if (
        typeof action.joint !== "string" ||
        action.joint.trim() === "" ||
        !Number.isFinite(action.effort_nm)
      ) {
        throw new Error(`frame ${index} has an invalid joint effort action`);
      }
      return;
    }
    throw new Error(`frame ${index} has an unknown action kind`);
  }

  function validateObservation(observation, index) {
    if (!observation || typeof observation !== "object") {
      throw new Error(`frame ${index} has an invalid observation`);
    }
    if (observation.joint_state) {
      const state = observation.joint_state;
      if (
        !Array.isArray(state.names) ||
        !Array.isArray(state.positions_rad) ||
        !Array.isArray(state.velocities_rad_s) ||
        state.names.length !== state.positions_rad.length ||
        state.names.length !== state.velocities_rad_s.length
      ) {
        throw new Error(`frame ${index} has an invalid joint state`);
      }
    }
    if (
      observation.sensor_streams !== undefined &&
      !Array.isArray(observation.sensor_streams)
    ) {
      throw new Error(`frame ${index} has invalid sensor streams`);
    }
    if (
      observation.sensor_payloads !== undefined &&
      !Array.isArray(observation.sensor_payloads)
    ) {
      throw new Error(`frame ${index} has invalid sensor payloads`);
    }
    if (Array.isArray(observation.sensor_payloads)) {
      observation.sensor_payloads.forEach((payload, payloadIndex) => {
        if (
          !payload ||
          !Number.isInteger(payload.stream_id) ||
          typeof payload.kind !== "string" ||
          payload.kind.trim() === "" ||
          !Number.isInteger(payload.sequence)
        ) {
          throw new Error(
            `frame ${index} has invalid sensor payload ${payloadIndex}`,
          );
        }
      });
    }
    if (observation.contact !== undefined && observation.contact !== null) {
      const contact = observation.contact;
      if (
        !Number.isInteger(contact.pair_count) ||
        !Number.isFinite(contact.total_impulse_ns) ||
        !Number.isFinite(contact.max_impulse_ns)
      ) {
        throw new Error(`frame ${index} has invalid contact annotations`);
      }
    }
  }

  function parseArtifactText(text) {
    // JSON.parse rounds u64 physics hashes when they are left as numbers.
    // Quote those fields before parsing so the inspector can display the
    // exact hash emitted by the Rust artifact writer.
    const losslessText = text.replace(
      /("(?:physics_hash|payload_hash)"\s*:\s*)(\d+)/g,
      '$1"$2"',
    );
    return validateArtifact(JSON.parse(losslessText));
  }

  function formatHash(value) {
    try {
      const decimal = BigInt(String(value));
      return `0x${decimal.toString(16).padStart(16, "0")}`;
    } catch (_error) {
      return String(value);
    }
  }

  function formatBaseTranslation(observation) {
    const translation = observation && observation.base_translation_m;
    if (!Array.isArray(translation) || translation.length !== 3) {
      return "none";
    }
    return `[${translation.map((value) => Number(value).toFixed(4)).join(", ")}] m`;
  }

  function formatAction(action) {
    const kind = action.kind || "differential_drive";
    if (kind === "differential_drive") {
      return `${Number(action.wheel_velocity_rad_s).toFixed(4)} rad/s`;
    }
    if (kind === "joint_velocity") {
      return `${action.joint}: ${Number(action.velocity_rad_s).toFixed(4)} rad/s`;
    }
    if (kind === "joint_effort") {
      return `${action.joint}: ${Number(action.effort_nm).toFixed(4)} N·m`;
    }
    return kind;
  }

  function formatObservation(observation) {
    const parts = [formatBaseTranslation(observation)];
    const jointState = observation && observation.joint_state;
    if (jointState) {
      parts.push(`joints=${jointState.names.length}`);
    }
    const sensors = observation && observation.sensor_streams;
    if (Array.isArray(sensors)) {
      const sensorSummary = sensors
        .map(
          (sensor) =>
            `${sensor.kind}#${sensor.stream_id}=${formatHash(sensor.payload_hash)}`,
        )
        .join(", ");
      parts.push(`sensors=${sensors.length}${sensorSummary ? ` (${sensorSummary})` : ""}`);
    }
    const payloads = observation && observation.sensor_payloads;
    if (Array.isArray(payloads) && payloads.length > 0) {
      const payloadSummary = payloads
        .map((payload) => formatPayload(payload))
        .join(", ");
      parts.push(`payloads=${payloads.length} (${payloadSummary})`);
    }
    const contact = observation && observation.contact;
    if (contact) {
      parts.push(`contact: ${formatContact(contact)}`);
    }
    return parts.join("; ");
  }

  function formatPayload(payload) {
    const kind = payload.kind;
    const sequence = `seq=${payload.sequence}`;
    const data = payload.data || {};
    if (kind === "lidar") {
      const points = data.points_m ? data.points_m.length : "?";
      return `${kind} ${sequence} pts=${points}`;
    }
    if (kind === "camera") {
      const rgb = data.rgb || {};
      const depth = data.depth || {};
      const rgbDims =
        rgb.width && rgb.height ? `${rgb.width}x${rgb.height}` : "?";
      const depthDims =
        depth.width && depth.height ? `${depth.width}x${depth.height}` : "?";
      return `${kind} ${sequence} rgb=${rgbDims} depth=${depthDims}`;
    }
    if (kind === "imu" || kind === "wheel_encoder") {
      return `${kind} ${sequence}`;
    }
    return `${kind}#${payload.stream_id} ${sequence}`;
  }

  function formatContact(contact) {
    if (!contact) {
      return "none";
    }
    return `${contact.pair_count} pairs, total ${Number(
      contact.total_impulse_ns,
    ).toFixed(4)} N·s, max ${Number(contact.max_impulse_ns).toFixed(4)} N·s`;
  }

  function formatReport(report) {
    const parts = [];
    if (report.contact_pairs_max !== undefined) {
      parts.push(`contacts_pairs_max=${report.contact_pairs_max}`);
    }
    if (
      report.contact_impulse_max_ns !== undefined &&
      report.contact_impulse_max_ns !== null
    ) {
      parts.push(
        `contact_impulse_max=${Number(report.contact_impulse_max_ns).toFixed(4)} N·s`,
      );
    }
    if (report.min_base_height_m !== undefined && report.min_base_height_m !== null) {
      parts.push(`min_height=${Number(report.min_base_height_m).toFixed(3)} m`);
    }
    if (report.failure) {
      parts.push(`FAILED: ${report.failure}`);
    } else {
      parts.push("failure=ok");
    }
    return parts.join(", ");
  }

  function renderFrame() {
    const artifact = state.artifact;
    if (!artifact || artifact.frames.length === 0) {
      frameOutput.textContent = "no frame";
      actionOutput.textContent = "—";
      observationOutput.textContent = "—";
      hashOutput.textContent = "—";
      reportOutput.textContent = "—";
      progress.style.width = "0%";
      return;
    }

    const frame = artifact.frames[state.frameIndex];
    rangeInput.value = String(state.frameIndex);
    frameOutput.textContent = `${frame.step} / ${artifact.clock.steps - 1} (${(
      frame.sim_ticks / 1_000_000_000
    ).toFixed(6)} s)`;
    actionOutput.textContent = formatAction(frame.action);
    observationOutput.textContent = formatObservation(frame.observation);
    hashOutput.textContent = formatHash(frame.physics_hash);
    progress.style.width = `${
      artifact.frames.length <= 1
        ? 100
        : (state.frameIndex / (artifact.frames.length - 1)) * 100
    }%`;
  }

  function updateControls() {
    const hasFrames = Boolean(state.artifact && state.artifact.frames.length);
    rangeInput.disabled = !hasFrames;
    playButton.disabled = !hasFrames;
    playButton.textContent = state.playing ? "Pause" : "Play";
  }

  function stopPlayback() {
    state.playing = false;
    state.lastTimestampMs = null;
    state.accumulatedMs = 0;
    if (state.animationId !== null) {
      cancelAnimationFrame(state.animationId);
      state.animationId = null;
    }
    updateControls();
  }

  function playbackFrame(timestampMs) {
    if (!state.playing || !state.artifact) {
      return;
    }
    if (state.lastTimestampMs === null) {
      state.lastTimestampMs = timestampMs;
    }
    state.accumulatedMs += timestampMs - state.lastTimestampMs;
    state.lastTimestampMs = timestampMs;
    const periodMs = 1_000 / state.artifact.clock.hz;
    while (state.accumulatedMs >= periodMs) {
      state.accumulatedMs -= periodMs;
      state.frameIndex += 1;
      if (state.frameIndex >= state.artifact.frames.length) {
        state.frameIndex = state.artifact.frames.length - 1;
        stopPlayback();
        renderFrame();
        return;
      }
    }
    renderFrame();
    state.animationId = requestAnimationFrame(playbackFrame);
  }

  function loadArtifact(artifact, fileName) {
    stopPlayback();
    state.artifact = artifact;
    state.frameIndex = 0;
    rangeInput.max = String(Math.max(0, artifact.frames.length - 1));
    sceneOutput.textContent = artifact.scene;
    clockOutput.textContent = `${artifact.clock.steps} steps @ ${artifact.clock.hz} Hz`;
    reportOutput.textContent = formatReport(artifact.final_report);
    setStatus(`${fileName}: loaded ${artifact.frames.length} frames`);
    updateControls();
    renderFrame();
  }

  fileInput.addEventListener("change", async () => {
    const file = fileInput.files && fileInput.files[0];
    if (!file) {
      return;
    }
    try {
      loadArtifact(parseArtifactText(await file.text()), file.name);
    } catch (error) {
      stopPlayback();
      state.artifact = null;
      updateControls();
      renderFrame();
      setStatus(`load failed: ${error.message}`, true);
    }
  });

  rangeInput.addEventListener("input", () => {
    if (!state.artifact) {
      return;
    }
    stopPlayback();
    state.frameIndex = Number(rangeInput.value);
    renderFrame();
  });

  playButton.addEventListener("click", () => {
    if (!state.artifact || state.artifact.frames.length === 0) {
      return;
    }
    if (state.playing) {
      stopPlayback();
      return;
    }
    if (state.frameIndex >= state.artifact.frames.length - 1) {
      state.frameIndex = 0;
    }
    state.playing = true;
    state.lastTimestampMs = null;
    state.accumulatedMs = 0;
    updateControls();
    state.animationId = requestAnimationFrame(playbackFrame);
  });

  updateControls();
  renderFrame();
})();
