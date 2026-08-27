# Independent metric anchor for a real-capture 3DGS scene

COLMAP reconstruction coordinates have no physical scale by themselves. RNE
therefore keeps a real-capture scene non-qualifying until a measurement made
independently of the fixture authoring is bound to two registered COLMAP 3D
points.

## Required field procedure

1. An operator who did not author the RNE fixture chooses two static, sharply
   identifiable endpoints that are both visible in a retained source frame.
2. The operator measures the straight-line endpoint distance with a tape,
   laser distance meter, total station, or calibrated RGB-D instrument and
   records the method, UTC time, value in metres, and instrument/process
   uncertainty.
3. Retain a photo or signed measurement report which visibly identifies both
   endpoints. Store it beside the scene fixture and record its byte length and
   lowercase SHA-256.
4. In the same retained camera frame, bind each endpoint to an existing COLMAP
   observation: camera ID, exact pixel coordinate, and `points3D` ID. Do not
   create a point from the desired physical length.
5. Fill a JSON record conforming to
   `docs/contracts/rne-independent-metric-scale-anchor-v1.schema.json` and
   store it beside `drjohnson.validation.json`.

The operator's organization, role, independence statement, raw evidence, and
declared uncertainty are part of the retained evidence. An assumed door width,
camera height, furniture size, or RealityCapture export is not an independent
measurement.

## Verification

Run:

```text
python tools/prepare_drjohnson_validation_fixture.py \
  --source-archive E:\RNE-tools\tandt_db.zip \
  --metric-anchor assets\environments\voxel51_drjohnson_3dgs\<anchor>.json
```

The generator rejects an endpoint unless its point ID and pixel are an exact
observation in the declared retained COLMAP camera. It computes:

```text
source_distance = distance(endpoint_a_xyz, endpoint_b_xyz)
derived_scale = measured_distance_m / source_distance
scale_uncertainty = uncertainty_m / source_distance
```

The scene manifest scale must lie inside `derived_scale ± scale_uncertainty`.
The Rust fixture auditor independently repeats the distance and scale checks,
rehashes the anchor record and every evidence artifact, and compares the raw
record with the resolved fixture fields. Missing evidence remains `missing`;
malformed, substituted, or inconsistent evidence is an error and cannot turn
the contract green.

The registered sparse-depth report (`IMG_6293.depth.json`) is intentionally
evaluated in COLMAP reconstruction units before this anchor exists. Its PLY,
camera calibration, full-frame hash, coverage, and semantic landmark residuals
are useful alignment evidence, but they do not convert those values to metres
or satisfy the independent metric-scale contract.
