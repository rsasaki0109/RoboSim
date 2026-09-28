"""Download the Poly Haven (CC0) props and textures the examples use.

Each model is fetched at 1k texture resolution in glTF form through the Poly
Haven public API and written under assets/props/polyhaven_<set>/<id>/.
`--set` chooses the collection (`warehouse` for the logistics examples,
`racing` for the race circuit); `--pin` rewrites that set's SHA-256 manifest
from what was downloaded; without it, every file must match the manifest.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import urllib.request
from pathlib import Path

PROPS = Path(__file__).resolve().parents[1] / "assets" / "props"
SETS = {
    "warehouse": {
        "models": [
            "cardboard_box_01",
            "wooden_crate_01",
            "hand_truck",
            "industrial_storage_cart",
            "mounted_fluorescent_lights",
            "korean_fire_extinguisher_01",
            "WetFloorSign_01",
            "power_box_01",
            "rollershutter_door",
            "steel_frame_shelves_02",
        ],
        "textures": ["box_profile_metal_sheet"],
        # Nodes removed from a model's scene after download (the pinned hashes
        # are of the downloaded files). The shutter ships with a graffiti
        # variant two meters to its side.
        "drop_nodes": {"rollershutter_door": ["rollershutter_door_graffiti"]},
    },
    "racing": {
        "models": ["old_tyre", "concrete_road_barrier"],
        "textures": ["asphalt_track", "leafy_grass"],
        "drop_nodes": {},
    },
}
# Surface textures, fetched as colour, OpenGL normal and roughness maps.
TEXTURE_MAPS = {"Diffuse": "diff", "nor_gl": "nor_gl", "Rough": "rough"}
RESOLUTION = "1k"


def fetch(url: str) -> bytes:
    request = urllib.request.Request(url, headers={"User-Agent": "RNE asset prep"})
    with urllib.request.urlopen(request, timeout=60) as response:
        return response.read()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pin", action="store_true", help="rewrite the manifest")
    parser.add_argument("--set", default="warehouse", choices=sorted(SETS))
    args = parser.parse_args()
    chosen = SETS[args.set]
    models, textures = chosen["models"], chosen["textures"]
    drop_nodes = chosen["drop_nodes"]
    root = PROPS / f"polyhaven_{args.set}"
    manifest = root / "manifest.json"
    expected = {} if args.pin else json.loads(manifest.read_text())["files"]
    pinned: dict[str, str] = {}
    authors: dict[str, dict] = {}
    for model in models:
        files = json.loads(fetch(f"https://api.polyhaven.com/files/{model}"))
        info = json.loads(fetch(f"https://api.polyhaven.com/info/{model}"))
        authors[model] = {"name": info["name"], "authors": sorted(info["authors"])}
        gltf = files["gltf"][RESOLUTION]["gltf"]
        entries = {Path(gltf["url"]).name: gltf["url"]}
        entries.update({relative: item["url"] for relative, item in gltf["include"].items()})
        for relative, url in entries.items():
            data = fetch(url)
            digest = hashlib.sha256(data).hexdigest()
            key = f"{model}/{relative}"
            if not args.pin and expected.get(key) != digest:
                raise SystemExit(f"unexpected SHA-256 for {key}: {digest}")
            target = root / model / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            if relative.endswith(".gltf") and model in drop_nodes:
                document = json.loads(data)
                drop = {
                    index
                    for index, node in enumerate(document["nodes"])
                    if node.get("name") in drop_nodes[model]
                }
                for scene in document["scenes"]:
                    scene["nodes"] = [n for n in scene["nodes"] if n not in drop]
                data = (json.dumps(document, indent=2) + "\n").encode()
            target.write_bytes(data)
            pinned[key] = digest
    for texture in textures:
        files = json.loads(fetch(f"https://api.polyhaven.com/files/{texture}"))
        info = json.loads(fetch(f"https://api.polyhaven.com/info/{texture}"))
        authors[texture] = {"name": info["name"], "authors": sorted(info["authors"])}
        for kind, suffix in TEXTURE_MAPS.items():
            url = files[kind][RESOLUTION]["jpg"]["url"]
            data = fetch(url)
            digest = hashlib.sha256(data).hexdigest()
            key = f"textures/{texture}_{suffix}_{RESOLUTION}.jpg"
            if not args.pin and expected.get(key) != digest:
                raise SystemExit(f"unexpected SHA-256 for {key}: {digest}")
            target = root / key
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
            pinned[key] = digest
    if args.pin:
        manifest.write_text(
            json.dumps(
                {"source": "https://polyhaven.com", "license": "CC0-1.0",
                 "resolution": RESOLUTION, "models": authors, "files": pinned},
                indent=2, sort_keys=True,
            )
            + "\n"
        )
    print(f"prepared {len(pinned)} files for {len(models)} models and {len(textures)} textures")


if __name__ == "__main__":
    main()
