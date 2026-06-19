"""Versioned policy artifact metadata for the mobile-manipulator RL example."""

import copy
import hashlib
import json


POLICY_ARTIFACT_VERSION = 2
POLICY_ARTIFACT_ALGORITHM = "rne_mobile_manipulator_linear_reach_policy_v1"
POLICY_PARAM_DIM = 4
POLICY_ACTION_LIMIT_RAD_S = 6.0
POLICY_FEATURES = ("target_dx", "target_dz")
POLICY_OUTPUTS = ("shoulder_velocity_rad_s", "elbow_velocity_rad_s")
POLICY_OBSERVATION_SCHEMA = {
    "id": "rne_mobile_manipulator_observation_v1",
    "dtype": "float64",
    "shape": [15],
    "fields": [
        {"name": "base_x", "unit": "m"},
        {"name": "base_y", "unit": "m"},
        {"name": "base_z", "unit": "m"},
        {"name": "base_yaw", "unit": "rad"},
        {"name": "ee_x", "unit": "m"},
        {"name": "ee_y", "unit": "m"},
        {"name": "ee_z", "unit": "m"},
        {"name": "shoulder_position", "unit": "rad"},
        {"name": "elbow_position", "unit": "rad"},
        {"name": "gripper_position", "unit": "rad"},
        {"name": "wrist_camera_pixels", "unit": "count"},
        {"name": "joint_state_count", "unit": "count"},
        {"name": "target_dx", "unit": "m"},
        {"name": "target_dy", "unit": "m"},
        {"name": "target_dz", "unit": "m"},
    ],
}
POLICY_ACTION_SCHEMA = {
    "id": "rne_mobile_manipulator_action_v1",
    "dtype": "float64",
    "shape": [5],
    "fields": [
        {"name": "left_wheel_velocity_rad_s", "unit": "rad/s"},
        {"name": "right_wheel_velocity_rad_s", "unit": "rad/s"},
        {"name": "shoulder_velocity_rad_s", "unit": "rad/s"},
        {"name": "elbow_velocity_rad_s", "unit": "rad/s"},
        {"name": "gripper_velocity_rad_s", "unit": "rad/s"},
    ],
}
POLICY_NORMALIZATION = {"type": "identity"}
POLICY_ACTION_SCALING = {
    "type": "clip",
    "limit_rad_s": POLICY_ACTION_LIMIT_RAD_S,
    "outputs": list(POLICY_OUTPUTS),
}
POLICY_TASK_COMPATIBILITY = {
    "environment": "VectorizedMobileManipulatorEnv",
    "tasks": ["reach_random"],
    "episode_family": "mobile_manipulator_reach",
}
POLICY_ENGINE_COMPATIBILITY = {
    "binding": "rne_py",
    "api": "VectorizedMobileManipulatorEnv",
    "action_tuple": [
        "left_wheel_velocity_rad_s",
        "right_wheel_velocity_rad_s",
        "shoulder_velocity_rad_s",
        "elbow_velocity_rad_s",
        "gripper_velocity_rad_s",
    ],
}
POLICY_ARTIFACT_REQUIRED_FIELDS = (
    "schema_version",
    "algorithm",
    "observation_schema_hash",
    "action_schema_hash",
    "observation_schema",
    "action_schema",
    "policy_features",
    "policy_outputs",
    "normalization",
    "action_scaling",
    "task_compatibility",
    "engine_compatibility",
    "param_dim",
    "action_limit_rad_s",
    "params",
    "best_reward",
    "training_iterations",
)


def stable_hash(payload):
    """Return the stable SHA-256 hash used by policy schema descriptors."""
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return "sha256:" + hashlib.sha256(encoded).hexdigest()


POLICY_OBSERVATION_SCHEMA_HASH = stable_hash(POLICY_OBSERVATION_SCHEMA)
POLICY_ACTION_SCHEMA_HASH = stable_hash(POLICY_ACTION_SCHEMA)


def policy_metadata_payload():
    """Return a deep-copied policy metadata envelope for JSON artifacts."""
    return {
        "schema_version": POLICY_ARTIFACT_VERSION,
        "algorithm": POLICY_ARTIFACT_ALGORITHM,
        "observation_schema_hash": POLICY_OBSERVATION_SCHEMA_HASH,
        "action_schema_hash": POLICY_ACTION_SCHEMA_HASH,
        "observation_schema": copy.deepcopy(POLICY_OBSERVATION_SCHEMA),
        "action_schema": copy.deepcopy(POLICY_ACTION_SCHEMA),
        "policy_features": list(POLICY_FEATURES),
        "policy_outputs": list(POLICY_OUTPUTS),
        "normalization": copy.deepcopy(POLICY_NORMALIZATION),
        "action_scaling": copy.deepcopy(POLICY_ACTION_SCALING),
        "task_compatibility": copy.deepcopy(POLICY_TASK_COMPATIBILITY),
        "engine_compatibility": copy.deepcopy(POLICY_ENGINE_COMPATIBILITY),
    }
