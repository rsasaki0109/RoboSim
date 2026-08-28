#!/usr/bin/env python3
"""Build deterministic real-camera correspondences for Dr Johnson depth audit."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import tomllib
import zipfile

import prepare_drjohnson_validation_fixture as fixture


CAMERA_NAMES = ("IMG_6292.jpg", "IMG_6293.jpg")
SELECTION_CAMERA = "IMG_6293.jpg"
GRID_WIDTH = 8
GRID_HEIGHT = 6
IMAGE_MARGIN_PX = 4.0
MAX_REPROJECTION_ERROR_PX = 2.0


def registered_camera(
    name: str,
    image: dict[str, object],
    camera: dict[str, object],
    environment_rotation: list[list[float]],
    translation: list[float],
    uniform_scale: float,
) -> dict[str, object]:
    source_center, source_basis = fixture.camera_pose(image)
    world_center = fixture.transform_point(
        source_center, environment_rotation, translation, uniform_scale
    )
    world_basis = fixture.mat_mul(environment_rotation, source_basis)
    return {
        "camera_id": f"colmap.{name}",
        "source_image_name": name,
        "intrinsics": camera,
        "colmap_world_to_camera": {
            "qvec_wxyz": list(image["qvec_wxyz"]),
            "tvec": list(image["tvec"]),
        },
        "rne_camera_to_world": {
            "translation_source_units": world_center,
            "rotation_xyzw": fixture.rotation_to_quaternion_xyzw(world_basis),
        },
    }


def build(source_archive: pathlib.Path, repository: pathlib.Path) -> dict[str, object]:
    if source_archive.stat().st_size != fixture.SOURCE_BYTES:
        raise ValueError("unexpected source archive size")
    if fixture.sha256_file(source_archive) != fixture.SOURCE_SHA256:
        raise ValueError("unexpected source archive SHA-256")

    asset_root = repository / "assets/environments/voxel51_drjohnson_3dgs"
    manifest_path = asset_root / "voxel51_drjohnson.rne.splat.toml"
    manifest = tomllib.loads(manifest_path.read_text(encoding="utf-8"))
    translation = manifest["translation_m"]
    uniform_scale = float(manifest["scale"])
    environment_rotation = fixture.quaternion_to_rotation_xyzw(
        manifest["rotation_xyzw"]
    )
    with zipfile.ZipFile(source_archive) as archive:
        camera_bytes = archive.read(fixture.CAMERAS_MEMBER)
        image_bytes = archive.read(fixture.IMAGES_MEMBER)
        point_bytes = archive.read(fixture.POINTS_MEMBER)
    cameras = fixture.read_cameras(camera_bytes)
    images = fixture.read_images(image_bytes, set(CAMERA_NAMES))
    observations = {
        name: {item[2]: item[:2] for item in images[name]["observations"]}
        for name in CAMERA_NAMES
    }
    shared_ids = set(observations[CAMERA_NAMES[0]]) & set(
        observations[CAMERA_NAMES[1]]
    )
    points = fixture.read_points(point_bytes, shared_ids)

    candidates = []
    for point_id in sorted(shared_ids):
        source_position = points[point_id]
        views = []
        valid = True
        for name in CAMERA_NAMES:
            image = images[name]
            camera = cameras[image["camera_id"]]
            observed = observations[name][point_id]
            projected_u, projected_v, depth = fixture.project_source(
                source_position, image, camera
            )
            reprojection_error = math.hypot(
                projected_u - observed[0], projected_v - observed[1]
            )
            valid = valid and depth > 0.0 and reprojection_error <= MAX_REPROJECTION_ERROR_PX
            views.append(
                {
                    "camera_id": f"colmap.{name}",
                    "observed_pixel_uv": list(observed),
                    "reference_depth_source_units": depth,
                    "reprojection_error_px": reprojection_error,
                }
            )
        if valid:
            candidates.append(
                {
                    "colmap_point3d_id": point_id,
                    "source_position": list(source_position),
                    "views": views,
                }
            )

    selection_camera = cameras[images[SELECTION_CAMERA]["camera_id"]]
    width = int(selection_camera["width_px"])
    height = int(selection_camera["height_px"])
    selected = []
    for grid_y in range(GRID_HEIGHT):
        for grid_x in range(GRID_WIDTH):
            center_x = (grid_x + 0.5) * width / GRID_WIDTH
            center_y = (grid_y + 0.5) * height / GRID_HEIGHT
            in_cell = []
            for track in candidates:
                selection_view = next(
                    view
                    for view in track["views"]
                    if view["camera_id"] == f"colmap.{SELECTION_CAMERA}"
                )
                pixel_x, pixel_y = selection_view["observed_pixel_uv"]
                if not (
                    IMAGE_MARGIN_PX <= pixel_x < width - IMAGE_MARGIN_PX
                    and IMAGE_MARGIN_PX <= pixel_y < height - IMAGE_MARGIN_PX
                    and int(pixel_x * GRID_WIDTH / width) == grid_x
                    and int(pixel_y * GRID_HEIGHT / height) == grid_y
                ):
                    continue
                distance_sq = (pixel_x - center_x) ** 2 + (pixel_y - center_y) ** 2
                in_cell.append((distance_sq, track["colmap_point3d_id"], track))
            if in_cell:
                selected.append(min(in_cell, key=lambda item: (item[0], item[1]))[2])

    camera_entries = [
        registered_camera(
            name,
            images[name],
            cameras[images[name]["camera_id"]],
            environment_rotation,
            translation,
            uniform_scale,
        )
        for name in CAMERA_NAMES
    ]
    return {
        "kind": "rne_registered_colmap_multiview_tracks",
        "schema_version": 1,
        "environment_id": manifest["environment_id"],
        "source_artifacts": {
            "colmap_cameras_sha256": fixture.sha256_bytes(camera_bytes),
            "colmap_images_sha256": fixture.sha256_bytes(image_bytes),
            "colmap_points3d_sha256": fixture.sha256_bytes(point_bytes),
        },
        "cameras": camera_entries,
        "selection": {
            "selection_camera_id": f"colmap.{SELECTION_CAMERA}",
            "grid_width": GRID_WIDTH,
            "grid_height": GRID_HEIGHT,
            "image_margin_px": IMAGE_MARGIN_PX,
            "max_reprojection_error_px": MAX_REPROJECTION_ERROR_PX,
            "shared_track_count": len(shared_ids),
            "valid_track_count": len(candidates),
            "occupied_grid_cell_count": len(selected),
            "tie_break": "minimum squared distance to cell center, then lowest COLMAP point ID",
        },
        "tracks": selected,
        "track_count": len(selected),
        "status": "verified",
        "units_note": "Depth values are COLMAP reconstruction units, not metres, until an independent physical scale anchor is retained.",
    }


def canonical_json(value: object) -> str:
    return json.dumps(value, indent=2, ensure_ascii=False) + "\n"


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
    output = repository / (
        "assets/environments/voxel51_drjohnson_3dgs/"
        "IMG_6292-IMG_6293.multiview-tracks.json"
    )
    encoded = canonical_json(build(args.source_archive, repository))
    if args.check:
        if not output.is_file() or output.read_text(encoding="utf-8") != encoded:
            raise SystemExit("committed Dr Johnson multi-view tracks drifted")
        print(f"validated={output}")
        return
    output.write_text(encoded, encoding="utf-8", newline="\n")
    print(f"wrote={output}")


if __name__ == "__main__":
    main()
