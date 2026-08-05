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
      if (
        !frame.action ||
        !Number.isFinite(frame.action.wheel_velocity_rad_s)
      ) {
        throw new Error(`frame ${index} has an invalid wheel action`);
      }
      if (
        typeof frame.physics_hash !== "string" &&
        !Number.isInteger(frame.physics_hash)
      ) {
        throw new Error(`frame ${index} has an invalid physics hash`);
      }
      previousTicks = frame.sim_ticks;
    });
    return artifact;
  }

  function parseArtifactText(text) {
    // JSON.parse rounds u64 physics hashes when they are left as numbers.
    // Quote those fields before parsing so the inspector can display the
    // exact hash emitted by the Rust artifact writer.
    const losslessText = text.replace(
      /("physics_hash"\s*:\s*)(\d+)/g,
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

  function renderFrame() {
    const artifact = state.artifact;
    if (!artifact || artifact.frames.length === 0) {
      frameOutput.textContent = "no frame";
      actionOutput.textContent = "—";
      observationOutput.textContent = "—";
      hashOutput.textContent = "—";
      progress.style.width = "0%";
      return;
    }

    const frame = artifact.frames[state.frameIndex];
    rangeInput.value = String(state.frameIndex);
    frameOutput.textContent = `${frame.step} / ${artifact.clock.steps - 1} (${(
      frame.sim_ticks / 1_000_000_000
    ).toFixed(6)} s)`;
    actionOutput.textContent = `${Number(
      frame.action.wheel_velocity_rad_s,
    ).toFixed(4)} rad/s`;
    observationOutput.textContent = formatBaseTranslation(frame.observation);
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
