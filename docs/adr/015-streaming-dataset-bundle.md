# ADR 015: Keep dataset evidence streaming and renderer-neutral

## Status

Accepted for the v0.5 dataset contract. Initial depth-pair slice implemented
on 2026-08-15; full reference capture coverage remains in progress.

## Context

RNE already publishes typed DataBus frames with capture and availability
timestamps and has lossless binary codecs for RGB8, depth, and LiDAR. A dataset
format that stores only images would discard latency, sequence gaps, seeded
noise, calibration, task identity, and source assets. A recorder that retains
the whole run would also duplicate the DataBus and fail at long sensor runs.

The format must be useful to training and evaluation tools without pulling
wgpu, ROS2, or a physics backend into `rne_data`. It must distinguish a sensor
drop from a stream that was never recorded and make partial corruption visible
before metrics are trusted.

## Decision

Use a directory bundle containing one JSON manifest and one append-only binary
record shard for schema v1. The writer retains only stream counters and the
incremental shard hasher. Every record carries simulation-time metadata and a
payload hash; the manifest carries the full shard hash and sorted stream
summaries.

Sensor stream manifests require explicit calibration and seeded noise
contracts. Fixed and bounded per-frame latency are separate models. Missing
sequences are valid only through an explicit `Gap` record. Existing RGB8,
depth, and LiDAR transport codecs remain the one binary definition for those
payloads.

Offline metrics live in `rne_data::offline`. The first committed evaluator
matches predicted and ground-truth depth by sequence and capture tick, then
computes deterministic MAE, RMSE, and maximum error without initializing a
renderer. Report verification recomputes the metric from the referenced
bundle rather than trusting a replaceable JSON self-hash.

## Consequences

Long runs have bounded recorder and evaluator memory, and the same bundle can
be checked on Windows or Linux in headless CI. Dataset types add only JSON and
SHA-256 dependencies to `rne_data`; no adapter, renderer, or external robotics
type enters core crates.

Schema v1 uses one shard and convenience writers only for the three existing
lossless sensor codecs. More payload codecs, capture examples, annotations,
and seeded reproduction evidence must be added without weakening the frozen
ordering, timing, gap, or digest rules. Self-hashes provide integrity, not
authenticity; signed provenance belongs to the later ecosystem milestone.
