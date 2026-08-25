#!/usr/bin/env python3
"""Build the content-addressed Dr Johnson 3DGS validation fixture.

The source archive is the official INRIA 3DGS Deep Blending input bundle.  The
tool extracts two reference frames, decodes their COLMAP intrinsics/extrinsics,
binds semantic image annotations to reconstructed 3D points, and projects the
RNE pickup collision proxy back into the real frame.  It deliberately leaves
the fixture non-qualifying until an independent metric scale anchor and an RNE
render-vs-reference comparison are retained.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
import pathlib
import statistics
import struct
import tomllib
import zipfile


SOURCE_URL = (
    "https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/"
    "datasets/input/tandt_db.zip"
)
SOURCE_BYTES = 682_628_995
SOURCE_SHA256 = "816e62f22a161abbfe841d2a6b10cdf036e297c9fa289b3bfeee9c6ec526d7e1"
CAMERAS_MEMBER = "db/drjohnson/sparse/0/cameras.bin"
IMAGES_MEMBER = "db/drjohnson/sparse/0/images.bin"
POINTS_MEMBER = "db/drjohnson/sparse/0/points3D.bin"
REFERENCE_MEMBERS = (
    "db/drjohnson/images/IMG_6292.jpg",
    "db/drjohnson/images/IMG_6293.jpg",
)
REFERENCE_SHA256 = {
    "IMG_6292.jpg": "3b9aae68c29fb2fcb528fe554fa6ecb2f64a091af84875473c1f0d46cc518d43",
    "IMG_6293.jpg": "278a543e1755beab9cef1ac708c587ffef12f5cc418db7afea91f52205157cca",
}
SEMANTIC_TARGETS = {
    "IMG_6293.jpg": (
        ("rug_front_left", "rug", (100.0, 842.0)),
        ("rug_front_center", "rug", (520.0, 842.0)),
        ("wood_floor_center", "wood_floor", (650.0, 780.0)),
        ("radiator_lower_left", "radiator", (255.0, 765.0)),
        ("window_sill_center", "window_sill", (665.0, 548.0)),
        ("right_door_latch", "door", (1044.0, 689.0)),
    ),
}
RUG_POLYGON_PX = ((0.0, 790.0), (760.0, 790.0), (895.0, 875.0), (0.0, 875.0))
FLOOR_REGION_POLYGON_PX = (
    (0.0, 735.0),
    (900.0, 735.0),
    (940.0, 875.0),
    (0.0, 875.0),
)
PICK_SUPPORT = {
    "id": "mobile_lift_pick_support",
    "semantic_surface": "rug",
    "center_world_m": [-0.593675032775816, 0.015, -4.025096749341726],
    "half_extents_m": [0.35, 0.015, 0.005],
}


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_exact(stream: io.BufferedIOBase | io.BytesIO, count: int) -> bytes:
    data = stream.read(count)
    if len(data) != count:
        raise ValueError(f"truncated binary input: wanted {count}, got {len(data)}")
    return data


def read_cameras(data: bytes) -> dict[int, dict[str, object]]:
    stream = io.BytesIO(data)
    count = struct.unpack("<Q", read_exact(stream, 8))[0]
    cameras: dict[int, dict[str, object]] = {}
    param_counts = {0: 3, 1: 4}
    model_names = {0: "SIMPLE_PINHOLE", 1: "PINHOLE"}
    for _ in range(count):
        camera_id, model_id = struct.unpack("<ii", read_exact(stream, 8))
        width, height = struct.unpack("<QQ", read_exact(stream, 16))
        if model_id not in param_counts:
            raise ValueError(f"unsupported COLMAP camera model {model_id}")
        params = struct.unpack(
            f"<{param_counts[model_id]}d", read_exact(stream, 8 * param_counts[model_id])
        )
        if model_id == 0:
            fx = fy = params[0]
            cx, cy = params[1:]
        else:
            fx, fy, cx, cy = params
        cameras[camera_id] = {
            "model": model_names[model_id],
            "width_px": width,
            "height_px": height,
            "fx_px": fx,
            "fy_px": fy,
            "cx_px": cx,
            "cy_px": cy,
            "fov_y_rad": 2.0 * math.atan(height / (2.0 * fy)),
        }
    if stream.read(1):
        raise ValueError("unexpected trailing camera bytes")
    return cameras


def read_c_string(stream: io.BytesIO) -> str:
    value = bytearray()
    while True:
        byte = read_exact(stream, 1)
        if byte == b"\0":
            return value.decode("utf-8")
        value.extend(byte)


def read_images(data: bytes, wanted: set[str]) -> dict[str, dict[str, object]]:
    stream = io.BytesIO(data)
    count = struct.unpack("<Q", read_exact(stream, 8))[0]
    images: dict[str, dict[str, object]] = {}
    for _ in range(count):
        image_id = struct.unpack("<i", read_exact(stream, 4))[0]
        qvec = struct.unpack("<4d", read_exact(stream, 32))
        tvec = struct.unpack("<3d", read_exact(stream, 24))
        camera_id = struct.unpack("<i", read_exact(stream, 4))[0]
        name = read_c_string(stream)
        point_count = struct.unpack("<Q", read_exact(stream, 8))[0]
        observations = []
        for _ in range(point_count):
            x_px, y_px, point_id = struct.unpack("<ddq", read_exact(stream, 24))
            if name in wanted and point_id >= 0:
                observations.append((x_px, y_px, point_id))
        if name in wanted:
            images[name] = {
                "image_id": image_id,
                "camera_id": camera_id,
                "qvec_wxyz": qvec,
                "tvec": tvec,
                "observations": observations,
            }
    missing = wanted.difference(images)
    if missing:
        raise ValueError(f"COLMAP images missing: {sorted(missing)}")
    return images


def read_points(data: bytes, wanted: set[int]) -> dict[int, tuple[float, float, float]]:
    stream = io.BytesIO(data)
    count = struct.unpack("<Q", read_exact(stream, 8))[0]
    points: dict[int, tuple[float, float, float]] = {}
    for _ in range(count):
        point_id = struct.unpack("<Q", read_exact(stream, 8))[0]
        xyz = struct.unpack("<3d", read_exact(stream, 24))
        read_exact(stream, 3 + 8)
        track_count = struct.unpack("<Q", read_exact(stream, 8))[0]
        read_exact(stream, track_count * 8)
        if point_id in wanted:
            points[point_id] = xyz
    missing = wanted.difference(points)
    if missing:
        raise ValueError(f"COLMAP points missing: {sorted(missing)[:8]}")
    return points


def qvec_to_rotation(qvec: tuple[float, ...]) -> list[list[float]]:
    w, x, y, z = qvec
    return [
        [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y],
        [2 * x * y + 2 * w * z, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * w * x],
        [2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x * x - 2 * y * y],
    ]


def quaternion_to_rotation_xyzw(qvec: list[float]) -> list[list[float]]:
    x, y, z, w = qvec
    return qvec_to_rotation((w, x, y, z))


def transpose(matrix: list[list[float]]) -> list[list[float]]:
    return [list(row) for row in zip(*matrix)]


def mat_vec(matrix: list[list[float]], vector) -> list[float]:
    return [sum(a * b for a, b in zip(row, vector)) for row in matrix]


def mat_mul(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    right_t = transpose(right)
    return [[sum(a * b for a, b in zip(row, col)) for col in right_t] for row in left]


def rotation_to_quaternion_xyzw(matrix: list[list[float]]) -> list[float]:
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return [
            (matrix[2][1] - matrix[1][2]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[1][0] - matrix[0][1]) / scale,
            0.25 * scale,
        ]
    candidates = (matrix[0][0], matrix[1][1], matrix[2][2])
    index = max(range(3), key=candidates.__getitem__)
    if index == 0:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        return [0.25 * scale, (matrix[0][1] + matrix[1][0]) / scale,
                (matrix[0][2] + matrix[2][0]) / scale,
                (matrix[2][1] - matrix[1][2]) / scale]
    if index == 1:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        return [(matrix[0][1] + matrix[1][0]) / scale, 0.25 * scale,
                (matrix[1][2] + matrix[2][1]) / scale,
                (matrix[0][2] - matrix[2][0]) / scale]
    scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
    return [(matrix[0][2] + matrix[2][0]) / scale,
            (matrix[1][2] + matrix[2][1]) / scale, 0.25 * scale,
            (matrix[1][0] - matrix[0][1]) / scale]


def add(left, right) -> list[float]:
    return [a + b for a, b in zip(left, right)]


def scale(vector, factor: float) -> list[float]:
    return [value * factor for value in vector]


def camera_pose(image: dict[str, object]) -> tuple[list[float], list[list[float]]]:
    world_to_camera = qvec_to_rotation(image["qvec_wxyz"])
    camera_to_world_colmap = transpose(world_to_camera)
    center = scale(mat_vec(camera_to_world_colmap, image["tvec"]), -1.0)
    # RNE cameras use +X right, +Y up and -Z forward. COLMAP uses +X right,
    # +Y down and +Z forward.
    rne_basis = [
        [camera_to_world_colmap[row][0], -camera_to_world_colmap[row][1],
         -camera_to_world_colmap[row][2]]
        for row in range(3)
    ]
    return center, rne_basis


def transform_point(point, rotation, translation, uniform_scale: float) -> list[float]:
    return add(mat_vec(rotation, scale(point, uniform_scale)), translation)


def inverse_transform_point(point, rotation, translation, uniform_scale: float) -> list[float]:
    centered = [a - b for a, b in zip(point, translation)]
    return scale(mat_vec(transpose(rotation), centered), 1.0 / uniform_scale)


def project_source(point, image, camera) -> tuple[float, float, float]:
    camera_point = add(mat_vec(qvec_to_rotation(image["qvec_wxyz"]), point), image["tvec"])
    if camera_point[2] <= 0.0:
        raise ValueError("point is behind COLMAP camera")
    u_px = camera["fx_px"] * camera_point[0] / camera_point[2] + camera["cx_px"]
    v_px = camera["fy_px"] * camera_point[1] / camera_point[2] + camera["cy_px"]
    return u_px, v_px, camera_point[2]


def nearest_observation(observations, target) -> tuple[float, float, int, float]:
    candidate = min(
        observations,
        key=lambda item: math.hypot(item[0] - target[0], item[1] - target[1]),
    )
    distance = math.hypot(candidate[0] - target[0], candidate[1] - target[1])
    if distance > 20.0:
        raise ValueError(
            f"no COLMAP observation within 20 px of {target}: {distance}; "
            f"nearest=({candidate[0]}, {candidate[1]})"
        )
    return (*candidate, distance)


def point_in_polygon(point, polygon) -> bool:
    x, y = point
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if (y1 > y) != (y2 > y):
            crossing = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing:
                inside = not inside
        previous = current
    return inside


def file_artifact(path: pathlib.Path, relative: str) -> dict[str, object]:
    return {
        "path": relative,
        "size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
    }


def build(source_archive: pathlib.Path, repository: pathlib.Path) -> dict[str, object]:
    if source_archive.stat().st_size != SOURCE_BYTES:
        raise ValueError(f"unexpected source archive size: {source_archive.stat().st_size}")
    archive_hash = sha256_file(source_archive)
    if archive_hash != SOURCE_SHA256:
        raise ValueError(f"unexpected source archive SHA-256: {archive_hash}")

    asset_root = repository / "assets/environments/voxel51_drjohnson_3dgs"
    manifest_path = asset_root / "voxel51_drjohnson.rne.splat.toml"
    manifest = tomllib.loads(manifest_path.read_text(encoding="utf-8"))
    translation = manifest["translation_m"]
    uniform_scale = float(manifest["scale"])
    environment_rotation = quaternion_to_rotation_xyzw(manifest["rotation_xyzw"])

    with zipfile.ZipFile(source_archive) as archive:
        camera_bytes = archive.read(CAMERAS_MEMBER)
        image_bytes = archive.read(IMAGES_MEMBER)
        point_bytes = archive.read(POINTS_MEMBER)
        reference_bytes = {
            pathlib.PurePosixPath(member).name: archive.read(member)
            for member in REFERENCE_MEMBERS
        }

    for name, data in reference_bytes.items():
        if sha256_bytes(data) != REFERENCE_SHA256[name]:
            raise ValueError(f"unexpected reference image SHA-256: {name}")
        output = asset_root / name.replace(".jpg", ".reference.jpg")
        output.write_bytes(data)

    cameras = read_cameras(camera_bytes)
    images = read_images(image_bytes, set(reference_bytes))
    selected = {}
    selected_ids: set[int] = set()
    for image_name, targets in SEMANTIC_TARGETS.items():
        selected[image_name] = []
        for landmark_id, semantic_class, target in targets:
            x_px, y_px, point_id, target_error = nearest_observation(
                images[image_name]["observations"], target
            )
            selected_ids.add(point_id)
            selected[image_name].append(
                (landmark_id, semantic_class, target, x_px, y_px, point_id, target_error)
            )
    floor_observations = [
        observation
        for observation in images["IMG_6293.jpg"]["observations"]
        if point_in_polygon((observation[0], observation[1]), FLOOR_REGION_POLYGON_PX)
    ]
    floor_ids = {observation[2] for observation in floor_observations}
    points = read_points(point_bytes, selected_ids | floor_ids)

    floor_world_y = [
        transform_point(points[point_id], environment_rotation, translation, uniform_scale)[1]
        for point_id in floor_ids
    ]
    bin_width = 0.02
    histogram: dict[int, int] = {}
    for value in floor_world_y:
        key = round(value / bin_width)
        histogram[key] = histogram.get(key, 0) + 1
    dominant_key = max(histogram, key=lambda key: (histogram[key], -abs(key)))
    dominant_center = dominant_key * bin_width
    floor_inliers = [
        value for value in floor_world_y if abs(value - dominant_center) <= 0.03
    ]
    floor_height = statistics.median(floor_inliers)
    floor_rmse = math.sqrt(
        sum((value - floor_height) ** 2 for value in floor_inliers) / len(floor_inliers)
    )

    camera_entries = []
    reprojection_errors = []
    semantic_landmarks = []
    for name in sorted(images):
        image = images[name]
        camera = cameras[image["camera_id"]]
        source_center, source_basis = camera_pose(image)
        world_center = transform_point(
            source_center, environment_rotation, translation, uniform_scale
        )
        world_basis = mat_mul(environment_rotation, source_basis)
        reference_name = name.replace(".jpg", ".reference.jpg")
        reference_path = asset_root / reference_name
        camera_entries.append(
            {
                "camera_id": f"colmap.{name}",
                "source_image_name": name,
                "reference_image": file_artifact(reference_path, reference_name),
                "intrinsics": camera,
                "colmap_world_to_camera": {
                    "qvec_wxyz": list(image["qvec_wxyz"]),
                    "tvec": list(image["tvec"]),
                },
                "rne_camera_to_world": {
                    "translation_m": world_center,
                    "rotation_xyzw": rotation_to_quaternion_xyzw(world_basis),
                },
            }
        )
        for item in selected.get(name, []):
            landmark_id, semantic_class, target, x_px, y_px, point_id, target_error = item
            source_point = points[point_id]
            projected_u, projected_v, depth = project_source(source_point, image, camera)
            reprojection_error = math.hypot(projected_u - x_px, projected_v - y_px)
            reprojection_errors.append(reprojection_error)
            semantic_landmarks.append(
                {
                    "landmark_id": landmark_id,
                    "semantic_class": semantic_class,
                    "annotation_source": "manual reference-image target snapped to nearest registered COLMAP point",
                    "camera_id": f"colmap.{name}",
                    "target_pixel_uv": list(target),
                    "observed_pixel_uv": [x_px, y_px],
                    "target_snap_error_px": target_error,
                    "colmap_point3d_id": point_id,
                    "source_position": list(source_point),
                    "world_position_m": transform_point(
                        source_point, environment_rotation, translation, uniform_scale
                    ),
                    "optical_depth_source_units": depth,
                    "reprojection_error_px": reprojection_error,
                }
            )

    comparison_name = "IMG_6293.jpg"
    comparison_image = images[comparison_name]
    comparison_camera = cameras[comparison_image["camera_id"]]
    support_top_center = list(PICK_SUPPORT["center_world_m"])
    support_top_center[1] += PICK_SUPPORT["half_extents_m"][1]
    support_source = inverse_transform_point(
        support_top_center, environment_rotation, translation, uniform_scale
    )
    support_u, support_v, support_depth = project_source(
        support_source, comparison_image, comparison_camera
    )
    support_inside_rug = point_in_polygon((support_u, support_v), RUG_POLYGON_PX)

    ply_path = asset_root / manifest["ply_path"]
    reprojection_rmse = math.sqrt(
        sum(error * error for error in reprojection_errors) / len(reprojection_errors)
    )
    reprojection_tolerance_px = 2.0
    camera_calibration_passed = max(reprojection_errors) <= reprojection_tolerance_px
    fixture = {
        "kind": "rne_gaussian_splat_validation_fixture",
        "schema_version": 1,
        "environment_id": manifest["environment_id"],
        "renderer_identity": manifest["renderer_identity"],
        "status": "incomplete",
        "qualifying": False,
        "provenance": {
            "source_archive_url": SOURCE_URL,
            "source_archive_size_bytes": SOURCE_BYTES,
            "source_archive_sha256": SOURCE_SHA256,
            "colmap_cameras_sha256": sha256_bytes(camera_bytes),
            "colmap_images_sha256": sha256_bytes(image_bytes),
            "colmap_points3d_sha256": sha256_bytes(point_bytes),
            "splat_manifest": file_artifact(
                manifest_path, "voxel51_drjohnson.rne.splat.toml"
            ),
            "splat_ply": file_artifact(ply_path, manifest["ply_path"]),
        },
        "source_to_world": {
            "translation_m": translation,
            "rotation_xyzw": manifest["rotation_xyzw"],
            "scale": uniform_scale,
            "source_units": "COLMAP reconstruction units",
            "world_units_claim": "m",
        },
        "metric_scale": {
            "status": "unverified",
            "scale_to_m": uniform_scale,
            "independent_physical_anchor": None,
            "reason": "The retained COLMAP model has no independently measured physical length; plausible room scale is not a metric calibration.",
        },
        "floor_alignment": {
            "status": "verified" if abs(floor_height) <= 0.03 else "failed",
            "reference_camera_id": "colmap.IMG_6293.jpg",
            "manual_floor_region_vertices_pixel_uv": [
                list(point) for point in FLOOR_REGION_POLYGON_PX
            ],
            "registered_candidate_count": len(floor_world_y),
            "dominant_plane_inlier_count": len(floor_inliers),
            "dominant_plane_world_y_claimed_m": floor_height,
            "dominant_plane_rmse_claimed_m": floor_rmse,
            "world_y_tolerance_claimed_m": 0.03,
            "note": "Lengths remain COLMAP reconstruction units until metric_scale has an independent anchor.",
        },
        "camera_calibration": {
            "status": "verified_colmap_reprojection" if camera_calibration_passed else "failed",
            "cameras": camera_entries,
            "semantic_landmark_count": len(semantic_landmarks),
            "reprojection_rmse_px": reprojection_rmse,
            "reprojection_max_error_px": max(reprojection_errors),
            "tolerance_px": reprojection_tolerance_px,
        },
        "semantic_landmarks": semantic_landmarks,
        "collision_semantic_alignment": {
            "status": "verified_reference_projection" if support_inside_rug else "failed",
            "proxy": PICK_SUPPORT,
            "camera_id": f"colmap.{comparison_name}",
            "projected_top_center_pixel_uv": [support_u, support_v],
            "projected_optical_depth_source_units": support_depth,
            "expected_semantic_polygon": {
                "semantic_class": "rug",
                "vertices_pixel_uv": [list(point) for point in RUG_POLYGON_PX],
                "annotation_source": "manual polygon on retained real reference image",
            },
            "top_center_inside_expected_semantic_polygon": support_inside_rug,
        },
        "real_sim_observation_comparison": {
            "status": "pending",
            "reference_camera_id": f"colmap.{comparison_name}",
            "reference_image": next(
                camera["reference_image"]
                for camera in camera_entries
                if camera["source_image_name"] == comparison_name
            ),
            "rne_render": None,
            "metrics": None,
        },
        "contracts": [
            {
                "id": "floor_world_alignment",
                "status": "passed" if abs(floor_height) <= 0.03 else "failed",
            },
            {
                "id": "camera_intrinsics_extrinsics",
                "status": "passed" if camera_calibration_passed else "failed",
            },
            {
                "id": "semantic_landmark_reprojection",
                "status": "passed" if camera_calibration_passed else "failed",
            },
            {
                "id": "collision_semantic_alignment",
                "status": "passed" if support_inside_rug else "failed",
            },
            {"id": "independent_metric_scale_anchor", "status": "missing"},
            {"id": "real_sim_observation_comparison", "status": "missing"},
        ],
    }
    return fixture


def canonical_json(value: object) -> str:
    return json.dumps(value, indent=2, sort_keys=False, ensure_ascii=False) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source-archive",
        type=pathlib.Path,
        default=pathlib.Path("E:/RNE-tools/tandt_db.zip"),
    )
    parser.add_argument("--repository", type=pathlib.Path, default=pathlib.Path("."))
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    repository = args.repository.resolve(strict=True)
    output = (
        repository
        / "assets/environments/voxel51_drjohnson_3dgs/drjohnson.validation.json"
    )
    fixture = build(args.source_archive, repository)
    encoded = canonical_json(fixture)
    if args.check:
        if not output.is_file():
            raise SystemExit(f"missing validation fixture: {output}")
        if output.read_text(encoding="utf-8") != encoded:
            raise SystemExit("committed Dr Johnson validation fixture drifted")
        print(
            f"validated={output} qualifying={fixture['qualifying']} "
            f"camera_rmse_px={fixture['camera_calibration']['reprojection_rmse_px']:.9f}"
        )
        return
    output.write_text(encoded, encoding="utf-8", newline="\n")
    print(
        f"wrote={output} qualifying={fixture['qualifying']} "
        f"camera_rmse_px={fixture['camera_calibration']['reprojection_rmse_px']:.9f}"
    )


if __name__ == "__main__":
    main()
