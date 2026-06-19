"""Canonical rollout CSV schema for the mobile-manipulator RL example."""

import hashlib
import json


ROLLOUT_CSV_SCHEMA_VERSION = 1
ROLLOUT_CSV_FIELDS = (
    "step",
    "base_x",
    "base_y",
    "base_yaw",
    "ee_x",
    "ee_y",
    "ee_z",
    "target_dx",
    "target_dy",
    "target_dz",
    "shoulder_action",
    "elbow_action",
    "reward",
    "total_reward",
    "done",
)
ROLLOUT_CSV_HEADER = ",".join(ROLLOUT_CSV_FIELDS)
ROLLOUT_NUMERIC_FIELDS = tuple(field for field in ROLLOUT_CSV_FIELDS if field != "done")
ROLLOUT_CSV_SCHEMA = {
    "schema_version": ROLLOUT_CSV_SCHEMA_VERSION,
    "format": "csv",
    "header": list(ROLLOUT_CSV_FIELDS),
    "numeric_fields": list(ROLLOUT_NUMERIC_FIELDS),
    "boolean_fields": ["done"],
}


def stable_hash(payload):
    """Return the stable SHA-256 hash used by rollout schema descriptors."""
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return "sha256:" + hashlib.sha256(encoded).hexdigest()


ROLLOUT_CSV_SCHEMA_HASH = stable_hash(ROLLOUT_CSV_SCHEMA)
