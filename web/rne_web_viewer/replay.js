(() => {
  "use strict";

  const SUPPORTED_VERSION = 1;
  const BEHAVIOR_REPLAY_KIND = "rne_behavior_replay";
  const SUPPORTED_BEHAVIOR_VERSION = 1;
  const SUPPORTED_BEHAVIOR_CONTRACT_VERSION = 2;
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

  function normalizeArtifact(artifact) {
    if (!artifact || artifact.kind !== BEHAVIOR_REPLAY_KIND) {
      return artifact;
    }
    validateBehaviorArtifact(artifact);
    const failure = artifact.failure;
    return {
      version: SUPPORTED_VERSION,
      scene: artifact.scenario,
      clock: {
        steps: artifact.frames.length,
        hz: 1_000_000_000 / artifact.fixed_delta_ticks,
      },
      frames: artifact.frames.map((frame) => ({
        step: frame.step,
        sim_ticks: frame.sim_time_ticks,
        action: { kind: "behavior_step", behavior_action: frame.action },
        observation: frame.observation,
        physics_hash: frame.state_digest,
      })),
      final_report: {
        failure: `${failure.contract.name}: ${failure.violation.message}`,
        behavior_failure: {
          seed: artifact.seed,
          contract: failure.contract.name,
          step: failure.violation.step,
          state_digest: failure.violation.state_digest,
          entities: failure.violation.entities,
          dimensions: artifact.dimensions,
          minimized: Boolean(artifact.minimization),
        },
      },
    };
  }

  function validateBehaviorArtifact(artifact) {
    if (artifact.schema_version !== SUPPORTED_BEHAVIOR_VERSION) {
      throw new Error(
        `unsupported behavior replay version (expected ${SUPPORTED_BEHAVIOR_VERSION})`,
      );
    }
    if (
      artifact.contract_schema_version !== SUPPORTED_BEHAVIOR_CONTRACT_VERSION
    ) {
      throw new Error(
        `unsupported behavior contract version (expected ${SUPPORTED_BEHAVIOR_CONTRACT_VERSION})`,
      );
    }
    if (
      typeof artifact.engine_version !== "string" ||
      artifact.engine_version.trim() === "" ||
      !isDigest(artifact.contract_digest) ||
      !isDigest(artifact.scenario_digest)
    ) {
      throw new Error("behavior compatibility metadata is invalid");
    }
    if (typeof artifact.scenario !== "string" || artifact.scenario.trim() === "") {
      throw new Error("behavior scenario is empty");
    }
    if (
      !Number.isInteger(artifact.fixed_delta_ticks) ||
      artifact.fixed_delta_ticks <= 0
    ) {
      throw new Error("behavior fixed_delta_ticks is invalid");
    }
    if (
      !Number.isFinite(artifact.observation_numeric_tolerance) ||
      artifact.observation_numeric_tolerance < 0
    ) {
      throw new Error("behavior observation tolerance is invalid");
    }
    if (!Array.isArray(artifact.contracts) || artifact.contracts.length === 0) {
      throw new Error("behavior contract manifest is empty");
    }
    if (!Array.isArray(artifact.dimensions)) {
      throw new Error("behavior dimensions are invalid");
    }
    if (!Array.isArray(artifact.frames) || artifact.frames.length === 0) {
      throw new Error("behavior replay has no frames");
    }
    artifact.frames.forEach((frame, index) => {
      if (!frame || frame.step !== index) {
        throw new Error(`behavior frame ${index} is not sequential`);
      }
      if (
        frame.sim_time_ticks !== artifact.fixed_delta_ticks * index ||
        !frame.observation ||
        typeof frame.observation !== "object" ||
        !isDigest(frame.state_digest)
      ) {
        throw new Error(`behavior frame ${index} is invalid`);
      }
      const expectedAction = index === 0 ? "initial_observation" : "advance";
      if (frame.action !== expectedAction) {
        throw new Error(`behavior frame ${index} has an invalid action`);
      }
    });
    const failure = artifact.failure;
    const finalFrame = artifact.frames[artifact.frames.length - 1];
    if (
      !failure ||
      !failure.contract ||
      typeof failure.contract.name !== "string" ||
      !failure.violation ||
      failure.violation.step !== finalFrame.step ||
      failure.violation.sim_time_ticks !== finalFrame.sim_time_ticks ||
      String(failure.violation.state_digest) !== String(finalFrame.state_digest)
    ) {
      throw new Error("behavior failure does not match the final frame");
    }
  }

  function isDigest(value) {
    return (
      (Number.isInteger(value) && value >= 0) ||
      (typeof value === "string" && /^\d+$/.test(value))
    );
  }

  function validateArtifact(artifact) {
    artifact = normalizeArtifact(artifact);
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
    if (kind === "joint_positions" || kind === "joint_velocities") {
      if (!Array.isArray(action.samples) || action.samples.length === 0) {
        throw new Error(`frame ${index} has invalid ${kind} samples`);
      }
      const valueKey =
        kind === "joint_positions" ? "position_rad" : "velocity_rad_s";
      let previousJoint = null;
      action.samples.forEach((sample) => {
        if (
          !sample ||
          typeof sample.joint !== "string" ||
          sample.joint.trim() === "" ||
          !Number.isFinite(sample[valueKey]) ||
          (previousJoint !== null && previousJoint >= sample.joint)
        ) {
          throw new Error(`frame ${index} has invalid ${kind} samples`);
        }
        previousJoint = sample.joint;
      });
      return;
    }
    if (kind === "robot_joint_velocities") {
      if (!Array.isArray(action.samples) || action.samples.length === 0) {
        throw new Error(`frame ${index} has invalid robot joint velocity samples`);
      }
      let previousKey = null;
      action.samples.forEach((sample) => {
        const key = sample && `${sample.robot_id}\0${sample.joint}`;
        if (
          !sample ||
          typeof sample.robot_id !== "string" ||
          sample.robot_id.trim() === "" ||
          typeof sample.joint !== "string" ||
          sample.joint.trim() === "" ||
          !Number.isFinite(sample.velocity_rad_s) ||
          (previousKey !== null && previousKey >= key)
        ) {
          throw new Error(`frame ${index} has invalid robot joint velocity samples`);
        }
        previousKey = key;
      });
      return;
    }
    if (kind === "behavior_step") {
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
      /("(?:physics_hash|payload_hash|state_digest|contract_digest|scenario_digest|source_state_digest|seed)"\s*:\s*)(\d+)/g,
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
    if (kind === "joint_positions") {
      return action.samples
        .map((sample) => `${sample.joint}=${Number(sample.position_rad).toFixed(4)} rad`)
        .join(", ");
    }
    if (kind === "joint_velocities") {
      return action.samples
        .map((sample) => `${sample.joint}=${Number(sample.velocity_rad_s).toFixed(4)} rad/s`)
        .join(", ");
    }
    if (kind === "robot_joint_velocities") {
      return action.samples
        .map(
          (sample) =>
            `${sample.robot_id}/${sample.joint}=${Number(sample.velocity_rad_s).toFixed(4)} rad/s`,
        )
        .join(", ");
    }
    if (kind === "behavior_step") {
      return `${action.behavior_action}: evaluate behavior contracts`;
    }
    return kind;
  }

  function formatObservation(observation) {
    const parts = [formatBaseTranslation(observation)];
    if (typeof observation.phase === "string") {
      parts[0] = `phase=${observation.phase}`;
    }
    if (
      Array.isArray(observation.part_position_m) &&
      observation.part_position_m.length === 3
    ) {
      parts.push(
        `part=[${observation.part_position_m
          .map((value) => Number(value).toFixed(4))
          .join(", ")}] m`,
      );
    }
    if (typeof observation.dual_contact === "boolean") {
      parts.push(
        `dual_contact=${observation.dual_contact}, grasped=${Boolean(
          observation.grasped,
        )}, inactive_contact=${Boolean(
          observation.inactive_hand_workcell_contact,
        )}`,
      );
    }
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
    if (report.behavior_failure) {
      const behavior = report.behavior_failure;
      parts.push(
        `seed=${behavior.seed}, contract=${behavior.contract}, step=${behavior.step}, state=${formatHash(
          behavior.state_digest,
        )}`,
      );
      parts.push(`dimensions=${behavior.dimensions.length}`);
      if (behavior.minimized) {
        parts.push("minimized=yes");
      }
    }
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
