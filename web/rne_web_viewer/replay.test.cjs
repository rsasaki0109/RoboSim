"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");

class StubElement {
  constructor() {
    this.dataset = {};
    this.disabled = false;
    this.files = [];
    this.listeners = {};
    this.max = "0";
    this.style = {};
    this.textContent = "";
    this.value = "0";
  }

  addEventListener(type, callback) {
    this.listeners[type] = callback;
  }
}

const elementIds = [
  "replay-file",
  "replay-play",
  "replay-range",
  "replay-status",
  "replay-scene",
  "replay-clock",
  "replay-frame",
  "replay-action",
  "replay-observation",
  "replay-hash",
  "replay-report",
  "replay-progress",
];
const elements = Object.fromEntries(
  elementIds.map((id) => [id, new StubElement()]),
);

global.document = {
  getElementById(id) {
    return elements[id];
  },
};
global.requestAnimationFrame = () => 1;
global.cancelAnimationFrame = () => {};

const artifact = {
  schema_version: 1,
  kind: "rne_behavior_replay",
  engine_version: "0.14.0-rc.1",
  contract_schema_version: 2,
  contract_digest: 1,
  scenario: "browser_behavior_fixture",
  scenario_digest: 2,
  seed: 7,
  fixed_delta_ticks: 10,
  observation_numeric_tolerance: 1e-12,
  dimensions: [
    { name: "tray_on_inactive_palm", value: true, baseline: false },
  ],
  contracts: [
    {
      name: "no_inactive_hand_contact",
      kind: { type: "always" },
      entities: ["left_hand_palm_link", "tray"],
    },
  ],
  frames: [
    {
      step: 0,
      sim_time_ticks: 0,
      action: "initial_observation",
      observation: {
        phase: "approach",
        part_position_m: [0.1, 0.2, 0.3],
        dual_contact: false,
        grasped: false,
        inactive_hand_workcell_contact: false,
      },
      state_digest: 11,
    },
    {
      step: 1,
      sim_time_ticks: 10,
      action: "advance",
      observation: {
        phase: "approach",
        part_position_m: [0.1, 0.2, 0.3],
        dual_contact: false,
        grasped: false,
        inactive_hand_workcell_contact: true,
      },
      state_digest: 12,
    },
  ],
  failure: {
    contract: {
      name: "no_inactive_hand_contact",
      kind: { type: "always" },
      entities: ["left_hand_palm_link", "tray"],
    },
    violation: {
      step: 1,
      sim_time_ticks: 10,
      state_digest: 12,
      entities: ["left_hand_palm_link", "tray"],
      message: "predicate was false",
    },
  },
};

vm.runInThisContext(fs.readFileSync(`${__dirname}/replay.js`, "utf8"), {
  filename: "replay.js",
});

async function main() {
  const behaviorText = JSON.stringify(artifact)
    .replace('"contract_digest":1', '"contract_digest":18446744073709551613')
    .replace('"scenario_digest":2', '"scenario_digest":18446744073709551614')
    .replace('"seed":7', '"seed":18446744073709551615')
    .replace('"state_digest":11', '"state_digest":18446744073709551614')
    .replaceAll('"state_digest":12', '"state_digest":18446744073709551615');
  elements["replay-file"].files = [
    { text: async () => behaviorText },
  ];
  await elements["replay-file"].listeners.change();

  assert.match(elements["replay-status"].textContent, /loaded 2 frames/);
  assert.equal(
    elements["replay-scene"].textContent,
    "browser_behavior_fixture",
  );
  assert.match(
    elements["replay-report"].textContent,
    /contract=no_inactive_hand_contact/,
  );
  assert.match(
    elements["replay-report"].textContent,
    /seed=18446744073709551615/,
  );

  elements["replay-range"].value = "1";
  elements["replay-range"].listeners.input();
  assert.match(elements["replay-action"].textContent, /^advance:/);
  assert.match(
    elements["replay-observation"].textContent,
    /inactive_contact=true/,
  );
  assert.equal(elements["replay-hash"].textContent, "0xffffffffffffffff");

  const standardArtifact = {
    version: 1,
    scene: "standard_scene",
    seed: 9,
    clock: { steps: 2, hz: 60 },
    frames: [
      {
        step: 0,
        sim_ticks: 1,
        action: { kind: "differential_drive", wheel_velocity_rad_s: 2 },
        observation: {},
        physics_hash: 17,
      },
      {
        step: 1,
        sim_ticks: 2,
        action: {
          kind: "robot_joint_velocities",
          samples: [
            {
              robot_id: "robot_a",
              joint: "shoulder_joint",
              velocity_rad_s: 1.5,
            },
            {
              robot_id: "robot_b",
              joint: "shoulder_joint",
              velocity_rad_s: -0.25,
            },
          ],
        },
        observation: {},
        physics_hash: 18,
      },
    ],
    final_report: {},
  };
  const standardText = JSON.stringify(standardArtifact).replace(
    '"physics_hash":17',
    '"physics_hash":18446744073709551615',
  );
  elements["replay-file"].files = [
    { text: async () => standardText },
  ];
  await elements["replay-file"].listeners.change();
  assert.equal(elements["replay-scene"].textContent, "standard_scene");
  assert.equal(elements["replay-action"].textContent, "2.0000 rad/s");
  assert.equal(elements["replay-hash"].textContent, "0xffffffffffffffff");
  elements["replay-range"].value = "1";
  elements["replay-range"].listeners.input();
  assert.equal(
    elements["replay-action"].textContent,
    "robot_a/shoulder_joint=1.5000 rad/s, robot_b/shoulder_joint=-0.2500 rad/s",
  );
}

main()
  .then(() => process.stdout.write("behavior replay inspector: ok\n"))
  .catch((error) => {
    console.error(error);
    process.exitCode = 1;
  });
